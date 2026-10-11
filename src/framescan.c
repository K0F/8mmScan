/* framescan.c - extract and stabilize 8mm film frames from scanner JPEGs.
 *
 * 8mm-only pipeline.  Per scan:
 *   1. decode to RGB and derive an 8-bit luma raster
 *   2. sf_find_band()            -> the horizontal film band
 *   3. sf_extract_band(angle=0)  -> pull the band, apply only the global
 *      position-dependent correction (no skew rotation)
 *   4. sf_find_sprockets() + sf_fit_strip_geometry() -> hole row: pitch,
 *      phase and strip angle (from the holes, never the picture)
 *   5. place frame boxes on the hole lattice
 *   6. stabilize each frame against the first using sprocket fiducials
 *   7. rotate upright, invert, deduplicate and write numbered PNGs
 *
 * The frame size, pitch and offsets are measured once, over a sample of the
 * whole set, so every emitted PNG has identical dimensions.  A per-frame size
 * would need rescaling downstream, which is what reads as the film jumping.
 */
#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <jpeglib.h>   /* must follow <stdio.h> */

#include "pngwrite.h"
#include "sprocketfind.h"
#include "stabilize.h"

#define SIG_DIM 32
#define MIN_HP_RMS 1.5
#define DEF_DEDUP_THRESH 0.97
#define DEF_OUTDIR "out"
#define DEF_GEOM_SAMPLE 16
#define DEF_SMOOTH 3

/* Along-strip pitch search window.  8mm at this scanner is ~700px, so this
 * rejects the sub-harmonics the old wide search used to lock onto. */
#define PITCH_LO 550.0
#define PITCH_HI 950.0

/* Super 8 geometry, millimetres.  The frame advance (one sprocket pitch) is
 * the ruler; the rest scale from it.  Regular 8 uses the same pitch. */
#define MM_FRAME_ADVANCE 4.234
#define MM_FRAME_W       4.01
#define MM_FRAME_H       5.79

/* The scanner is fixed, so the whole set shares one geometry: a reference
 * pitch, the fractional pitch change across the strip (RODFS50 images the
 * bottom edge a little larger than the top) and the band height.  Only the
 * strip's vertical position and along-strip phase change from scan to scan. */
struct geo8 {
    double pitch;     /* global reference sprocket pitch in px */
    double grad;      /* fractional pitch change top..bottom    */
    double band_h;    /* reference film band thickness in px    */
    int frame_w;      /* frame size along the strip             */
    int frame_h;      /* frame size across the strip            */
    int offset_x;     /* frame left edge - comb phase           */
    int offset_y;     /* vertical shift from the centred frame  */
    int n_scans;
};

/* ------------------------------------------------------------------ JPEG */

struct jerr_mgr {
    struct jpeg_error_mgr pub;
    jmp_buf setjmp_buffer;
};

static void jpeg_fail(j_common_ptr cinfo)
{
    struct jerr_mgr *e = (struct jerr_mgr *) cinfo->err;
    longjmp(e->setjmp_buffer, 1);
}

static void jpeg_quiet(j_common_ptr cinfo)
{
    (void) cinfo;
}

static int read_jpeg_rgb(const char *path, unsigned char **out_rgb,
                         int *out_w, int *out_h)
{
    struct jpeg_decompress_struct cinfo;
    struct jerr_mgr jerr;
    FILE *fp = NULL;
    unsigned char *rgb = NULL;
    JSAMPROW rowptr[1];

    fp = fopen(path, "rb");
    if (!fp)
        return -1;

    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = jpeg_fail;
    jerr.pub.output_message = jpeg_quiet;
    if (setjmp(jerr.setjmp_buffer)) {
        jpeg_destroy_decompress(&cinfo);
        free(rgb);
        fclose(fp);
        return -1;
    }

    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, fp);
    jpeg_read_header(&cinfo, TRUE);
    cinfo.out_color_space = JCS_RGB;
    jpeg_start_decompress(&cinfo);

    rgb = malloc((size_t) cinfo.output_width * cinfo.output_height * 3);
    if (!rgb) {
        jpeg_destroy_decompress(&cinfo);
        fclose(fp);
        return -1;
    }

    while (cinfo.output_scanline < cinfo.output_height) {
        rowptr[0] = rgb + (size_t) cinfo.output_scanline * cinfo.output_width * 3;
        jpeg_read_scanlines(&cinfo, rowptr, 1);
    }

    *out_w = (int) cinfo.output_width;
    *out_h = (int) cinfo.output_height;
    *out_rgb = rgb;

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    fclose(fp);
    return 0;
}

/* ------------------------------------------------------------------ misc */

static void rgb_to_gray(const unsigned char *rgb, unsigned char *gray, int npix)
{
    int i;
    for (i = 0; i < npix; i++) {
        int r = rgb[3 * i], g = rgb[3 * i + 1], b = rgb[3 * i + 2];
        gray[i] = (unsigned char) ((299 * r + 587 * g + 114 * b) / 1000);
    }
}

static void oriented_size(int w, int h, int rot, int *out_w, int *out_h)
{
    if (rot == 90 || rot == 270) {
        *out_w = h;
        *out_h = w;
    } else {
        *out_w = w;
        *out_h = h;
    }
}

static void orient_rgb(const unsigned char *src, int w, int h, int rot,
                       int negative, unsigned char *dst,
                       int *out_w, int *out_h)
{
    int x, y, c, ow, oh;

    oriented_size(w, h, rot, &ow, &oh);
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            const unsigned char *s = src + ((size_t) y * w + x) * 3;
            unsigned char *d;
            int dx = x, dy = y;

            if (rot == 90) {
                dx = h - 1 - y;
                dy = x;
            } else if (rot == 180) {
                dx = w - 1 - x;
                dy = h - 1 - y;
            } else if (rot == 270) {
                dx = y;
                dy = w - 1 - x;
            }
            d = dst + ((size_t) dy * ow + dx) * 3;
            for (c = 0; c < 3; c++)
                d[c] = negative ? (unsigned char) (255 - s[c]) : s[c];
        }
    }
    *out_w = ow;
    *out_h = oh;
}

static void crop_rgb(const unsigned char *rgb, int w, int h, const ff_box *bx,
                     unsigned char *dst)
{
    int y;
    for (y = 0; y < bx->h; y++) {
        const unsigned char *src = rgb + ((size_t) (bx->y0 + y) * w + bx->x0) * 3;
        unsigned char *d = dst + (size_t) y * bx->w * 3;
        int n = bx->w;
        if (bx->y0 + y >= h)
            break;
        if (bx->x0 + n > w)
            n = w - bx->x0;
        memset(d, 0, (size_t) bx->w * 3);
        memcpy(d, src, (size_t) n * 3);
        if (n < bx->w)
            memset(d + (size_t) n * 3, 0, (size_t) (bx->w - n) * 3);
    }
}

/* High-pass, mean-removed, unit-norm 32x32 signature.  See the 35mm pipeline
 * notes: the high-pass is what keeps two different frames from matching. */
static int crop_signature(const unsigned char *gray, int w, const ff_box *bx,
                          double *sig)
{
    int bw = bx->w, bh = bx->h, r, i, j, x, y, k, ok;
    double *hblur = NULL, *vbuf = NULL, *vmean = NULL, *pref = NULL;
    double *acc = NULL;
    long *cnt = NULL;
    double sumsq = 0.0, mean = 0.0, norm = 0.0;
    long npix = 0;

    if (bw <= 0 || bh <= 0)
        return 0;

    r = bw / 45;
    if (r < 2)
        r = 2;

    hblur = malloc(sizeof(double) * (size_t) bw * bh);
    vbuf = calloc((size_t) bw, sizeof(double));
    vmean = malloc(sizeof(double) * (size_t) bw);
    pref = malloc(sizeof(double) * (size_t) (bw + 1));
    acc = calloc(SIG_DIM * SIG_DIM, sizeof(double));
    cnt = calloc(SIG_DIM * SIG_DIM, sizeof(long));
    if (!hblur || !vbuf || !vmean || !pref || !acc || !cnt)
        goto out;

    for (y = 0; y < bh; y++) {
        const unsigned char *p = gray + (size_t) (bx->y0 + y) * w + bx->x0;
        double *d = hblur + (size_t) y * bw;
        double run = 0.0;
        int lo = 0, hi = -1;
        for (x = 0; x < bw; x++) {
            int want_hi = x + r < bw - 1 ? x + r : bw - 1;
            int want_lo = x - r > 0 ? x - r : 0;
            while (hi < want_hi)
                run += p[++hi];
            while (lo < want_lo)
                run -= p[lo++];
            d[x] = run / (hi - lo + 1);
        }
    }

    {
        int lo = 0, hi = -1;
        for (y = 0; y < bh; y++) {
            int want_hi = y + r < bh - 1 ? y + r : bh - 1;
            int want_lo = y - r > 0 ? y - r : 0;
            const unsigned char *prow = gray + (size_t) (bx->y0 + y) * w + bx->x0;
            double inv;
            while (hi < want_hi) {
                const double *hr = hblur + (size_t) (++hi) * bw;
                for (x = 0; x < bw; x++)
                    vbuf[x] += hr[x];
            }
            while (lo < want_lo) {
                const double *hr = hblur + (size_t) lo * bw;
                for (x = 0; x < bw; x++)
                    vbuf[x] -= hr[x];
                lo++;
            }
            inv = 1.0 / (hi - lo + 1);
            pref[0] = 0.0;
            for (x = 0; x < bw; x++) {
                double d = (double) prow[x] - vbuf[x] * inv;
                vmean[x] = d;
                pref[x + 1] = pref[x] + d;
                sumsq += d * d;
            }
            npix += bw;
            j = y * SIG_DIM / bh;
            for (i = 0; i < SIG_DIM; i++) {
                int x0 = i * bw / SIG_DIM, x1 = (i + 1) * bw / SIG_DIM;
                if (x1 <= x0)
                    x1 = x0 + 1;
                if (x1 > bw)
                    x1 = bw;
                acc[j * SIG_DIM + i] += pref[x1] - pref[x0];
                cnt[j * SIG_DIM + i] += x1 - x0;
            }
        }
    }

    for (k = 0; k < SIG_DIM * SIG_DIM; k++) {
        sig[k] = cnt[k] ? acc[k] / (double) cnt[k] : 0.0;
        mean += sig[k];
    }
    mean /= (double) (SIG_DIM * SIG_DIM);
    for (k = 0; k < SIG_DIM * SIG_DIM; k++) {
        sig[k] -= mean;
        norm += sig[k] * sig[k];
    }
    norm = sqrt(norm);

    ok = (npix > 0) && (sumsq / (double) npix >= MIN_HP_RMS * MIN_HP_RMS);
    if (ok && norm > 1e-9) {
        for (k = 0; k < SIG_DIM * SIG_DIM; k++)
            sig[k] /= norm;
    } else {
        ok = 0;
    }

out:
    free(hblur);
    free(vbuf);
    free(vmean);
    free(pref);
    free(acc);
    free(cnt);
    return ok;
}

static double sig_sim(const double *a, const double *b)
{
    double acc = 0.0;
    int i;
    for (i = 0; i < SIG_DIM * SIG_DIM; i++)
        acc += a[i] * b[i];
    return acc;
}

static int cmp_names(const void *a, const void *b)
{
    return strcmp(*(const char *const *) a, *(const char *const *) b);
}

static int has_jpeg_ext(const char *n)
{
    size_t l = strlen(n);
    if (l < 5)
        return 0;
    return strcmp(n + l - 4, ".JPG") == 0 || strcmp(n + l - 4, ".jpg") == 0 ||
           strcmp(n + l - 5, ".JPEG") == 0 || strcmp(n + l - 5, ".jpeg") == 0;
}

static void usage(FILE *fp, const char *prog)
{
    fprintf(fp,
"Usage: %s [options] <scan.jpg|dir> [more...]\n"
"\n"
"Extracts, stabilizes and writes 8mm film frames as numbered PNGs.\n"
"\n"
"Options:\n"
"  -o, --outdir DIR        output directory (default %s)\n"
"      --render-dir DIR    PNG subdirectory (default <outdir>/render)\n"
"      --prefix STR        output filename prefix (default frame)\n"
"      --rotate DEG        rotate frames clockwise: 0, 90, 180 or 270 (default 90)\n"
"      --no-rotate         do not rotate (same as --rotate 0)\n"
"      --positive          keep the scanned tonality instead of inverting it\n"
"      --no-stabilize      skip sprocket-fiducial stabilization\n"
"      --frame-w PX        frame width along the strip (default measured)\n"
"      --frame-h PX        frame height across the strip (default measured)\n"
"      --offset-x PX       first frame left edge minus comb phase (default -w/2)\n"
"      --offset-y PX       shift from the centred frame (default 0)\n"
"      --pitch PX          force sprocket pitch instead of measuring it\n"
"      --sample N          scans sampled to measure global geometry (default %d)\n"
"      --include-partial   also emit frames clipped by the image edge\n"
"      --no-dedup          keep every frame, including overlaps\n"
"      --dedup-thresh F    cosine similarity treated as duplicate\n"
"                         (default %.2f, 1.0 disables)\n"
"      --report            print per-scan geometry to stderr\n"
"      --dry-run           detect and report, write nothing\n"
"  -h, --help              this text\n",
            prog, DEF_OUTDIR, DEF_GEOM_SAMPLE, DEF_DEDUP_THRESH);
}

/* ------------------------------------------------------- geometry pass */

static int cmp_double(const void *a, const void *b)
{
    double x = *(const double *) a, y = *(const double *) b;
    return x < y ? -1 : (x > y ? 1 : 0);
}

static double median_of(double *v, int n)
{
    if (n <= 0)
        return 0.0;
    qsort(v, (size_t) n, sizeof(double), cmp_double);
    return (n & 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

/* Measures the global model over a sample of scans: the reference pitch, the
 * fractional pitch change across the strip, and the band height.  Only the
 * strip's position and phase vary per scan; everything else is fixed so every
 * PNG has identical dimensions. */
static int measure_geometry(struct geo8 *geo, char **paths, int n_paths,
                            int sample, int verbose)
{
    double *pitch, *grad, *bandh;
    int np = 0, ng = 0, nb = 0, i, step, bad = 0, fw, fh;

    if (n_paths <= 0)
        return -1;
    pitch = malloc(sizeof(double) * (size_t) n_paths);
    grad = malloc(sizeof(double) * (size_t) n_paths);
    bandh = malloc(sizeof(double) * (size_t) n_paths);
    if (!pitch || !grad || !bandh) {
        free(pitch);
        free(grad);
        free(bandh);
        return -1;
    }

    if (sample < 3)
        sample = 3;
    step = n_paths / sample;
    if (step < 1)
        step = 1;

    for (i = 0; i < n_paths; i += step) {
        unsigned char *rgb = NULL, *gray = NULL;
        int w = 0, h = 0, y0 = 0, y1 = 0;
        sf_band b;
        sf_strip_geometry strip;
        sf_sprocket sp[SF_MAX_SPROCKETS], spf[SF_MAX_SPROCKETS];
        int nsp = 0, nspf = 0;
        const char *base = strrchr(paths[i], '/');
        base = base ? base + 1 : paths[i];

        if (read_jpeg_rgb(paths[i], &rgb, &w, &h) != 0) {
            bad++;
            continue;
        }
        gray = malloc((size_t) w * h);
        if (!gray) {
            free(rgb);
            bad++;
            continue;
        }
        rgb_to_gray(rgb, gray, w * h);
        free(rgb);

        if (sf_find_band(gray, w, h, &y0, &y1) != 0) {
            free(gray);
            bad++;
            continue;
        }

        /* Reference pitch from the sprocket holes; grad and band height from
         * the band profile, because the single Super-8 hole row cannot by
         * itself measure the across-strip pitch change. */
        memset(&strip, 0, sizeof strip);
        if (sf_find_sprockets(gray, w, h, y0, y1, sp, &nsp) >= 0 && nsp >= 2) {
            sf_filter_sprockets(sp, nsp, spf, &nspf);
            if (nspf >= 2 && sf_fit_strip_geometry(spf, nspf, &strip) != 0)
                strip.pitch = 0.0;
        }

        if (sf_analyze_band(gray, w, h, y0, y1, PITCH_LO, PITCH_HI, &b) != 0) {
            free(gray);
            bad++;
            continue;
        }
        free(gray);

        if (strip.pitch > PITCH_LO && strip.pitch < PITCH_HI)
            pitch[np++] = strip.pitch;
        if (b.pitch_top > 0.0 && b.pitch_bot > 0.0)
            grad[ng++] = (b.pitch_bot - b.pitch_top) / b.pitch;
        bandh[nb++] = (double) (y1 - y0 + 1);

        if (verbose)
            fprintf(stderr, "  geom %-16s band_h=%4d pitch=%7.2f "
                            "grad=%+.4f holes=%d\n",
                    base, y1 - y0 + 1, strip.pitch,
                    (b.pitch_top > 0.0 && b.pitch_bot > 0.0) ?
                        (b.pitch_bot - b.pitch_top) / b.pitch : 0.0,
                    nspf);
    }

    if (np < 1) {
        fprintf(stderr, "framescan: no scan yielded a usable sprocket pitch\n");
        free(pitch);
        free(grad);
        free(bandh);
        return -1;
    }

    geo->pitch = median_of(pitch, np);
    geo->grad = ng ? median_of(grad, ng) : 0.0;
    geo->band_h = nb ? median_of(bandh, nb) : 0.0;
    geo->n_scans = np;
    fw = (int) (MM_FRAME_W / MM_FRAME_ADVANCE * geo->pitch + 0.5);
    fh = (int) (MM_FRAME_H / MM_FRAME_ADVANCE * geo->pitch + 0.5);
    if (fw < 8)
        fw = 8;
    if (fh < 8)
        fh = 8;
    geo->frame_w = fw;
    geo->frame_h = fh;
    geo->offset_x = -fw / 2;

    fprintf(stderr, "geometry: pitch %.2f px (%.2f px/mm) grad %+.4f "
                    "band_h~%.0f frame %dx%d offset_x %d over %d scans, "
                    "%d skipped\n",
            geo->pitch, geo->pitch / MM_FRAME_ADVANCE, geo->grad, geo->band_h,
            geo->frame_w, geo->frame_h, geo->offset_x, np, bad);

    free(pitch);
    free(grad);
    free(bandh);
    return 0;
}

/* ------------------------------------------------------------------ main */

int main(int argc, char **argv)
{
    const char *outdir = DEF_OUTDIR;
    const char *render_dir = NULL;
    char *render_buf = NULL;
    const char *prefix = "frame";
    int geom_sample = DEF_GEOM_SAMPLE;
    struct geo8 geo;
    int rotate = 90, negative = 1, stabilize = 1;
    double dedup_thresh = DEF_DEDUP_THRESH;
    int include_partial = 0, dry_run = 0, report = 0;
    int pitch_given = 0;
    double pitch_override = 0.0;
    int fw_given = 0, fh_given = 0, ox_given = 0, oy_given = 0;
    int fw_opt = 0, fh_opt = 0, ox_opt = 0, oy_opt = 0;

    char **inputs;
    int n_inputs = 0, i, s;
    double **sigs = NULL;
    int *sig_frame = NULL, n_sigs = 0, sig_cap = 0;
    FILE *csv = NULL;
    int index = 0;
    int n_bad = 0;
    char *csv_path = NULL;

    inputs = calloc((size_t) argc, sizeof(char *));
    if (!inputs)
        return 1;

    for (i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (!strcmp(a, "-h") || !strcmp(a, "--help")) {
            usage(stdout, argv[0]);
            free(inputs);
            return 0;
        } else if (!strcmp(a, "-o") || !strcmp(a, "--outdir")) {
            if (++i >= argc) goto badarg;
            outdir = argv[i];
        } else if (!strcmp(a, "--render-dir")) {
            if (++i >= argc) goto badarg;
            render_dir = argv[i];
        } else if (!strcmp(a, "--prefix")) {
            if (++i >= argc) goto badarg;
            prefix = argv[i];
        } else if (!strcmp(a, "--rotate")) {
            if (++i >= argc) goto badarg;
            rotate = atoi(argv[i]);
            if (rotate != 0 && rotate != 90 && rotate != 180 && rotate != 270) {
                fprintf(stderr, "%s: --rotate must be 0, 90, 180 or 270\n",
                        argv[0]);
                return 2;
            }
        } else if (!strcmp(a, "--no-rotate")) {
            rotate = 0;
        } else if (!strcmp(a, "--positive")) {
            negative = 0;
        } else if (!strcmp(a, "--negative")) {
            negative = 1;
        } else if (!strcmp(a, "--no-stabilize")) {
            stabilize = 0;
        } else if (!strcmp(a, "--frame-w")) {
            if (++i >= argc) goto badarg;
            fw_opt = atoi(argv[i]);
            fw_given = 1;
        } else if (!strcmp(a, "--frame-h")) {
            if (++i >= argc) goto badarg;
            fh_opt = atoi(argv[i]);
            fh_given = 1;
        } else if (!strcmp(a, "--offset-x")) {
            if (++i >= argc) goto badarg;
            ox_opt = atoi(argv[i]);
            ox_given = 1;
        } else if (!strcmp(a, "--offset-y")) {
            if (++i >= argc) goto badarg;
            oy_opt = atoi(argv[i]);
            oy_given = 1;
        } else if (!strcmp(a, "--pitch")) {
            if (++i >= argc) goto badarg;
            pitch_override = atof(argv[i]);
            pitch_given = 1;
        } else if (!strcmp(a, "--sample")) {
            if (++i >= argc) goto badarg;
            geom_sample = atoi(argv[i]);
            if (geom_sample < 3) {
                fprintf(stderr, "%s: --sample must be at least 3\n", argv[0]);
                return 2;
            }
        } else if (!strcmp(a, "--dedup-thresh")) {
            if (++i >= argc) goto badarg;
            dedup_thresh = atof(argv[i]);
        } else if (!strcmp(a, "--include-partial")) {
            include_partial = 1;
        } else if (!strcmp(a, "--no-dedup")) {
            dedup_thresh = 1.0;
        } else if (!strcmp(a, "--report")) {
            report = 1;
        } else if (!strcmp(a, "--dry-run")) {
            dry_run = 1;
        } else if (a[0] == '-' && a[1] != '\0') {
            goto badarg;
        } else {
            inputs[n_inputs++] = argv[i];
        }
    }
    goto okargs;

badarg:
    fprintf(stderr, "%s: bad or missing argument near '%s'\n", argv[0], argv[i]);
    usage(stderr, argv[0]);
    free(inputs);
    return 2;

okargs:
    if (n_inputs == 0) {
        usage(stderr, argv[0]);
        free(inputs);
        return 2;
    }

    /* Expand directories into their JPEG files. */
    {
        int cap = n_inputs, n = 0;
        char **list = malloc(sizeof(char *) * (size_t) cap);
        if (!list) {
            free(inputs);
            return 1;
        }
        for (i = 0; i < n_inputs; i++) {
            struct stat st;
            if (stat(inputs[i], &st) == 0 && S_ISDIR(st.st_mode)) {
                DIR *d = opendir(inputs[i]);
                struct dirent *e;
                if (!d)
                    continue;
                while ((e = readdir(d)) != NULL) {
                    if (!has_jpeg_ext(e->d_name))
                        continue;
                    if (n >= cap) {
                        cap *= 2;
                        list = realloc(list, sizeof(char *) * (size_t) cap);
                        if (!list) {
                            closedir(d);
                            free(inputs);
                            return 1;
                        }
                    }
                    list[n++] = malloc(strlen(inputs[i]) + strlen(e->d_name) + 2);
                    sprintf(list[n - 1], "%s/%s", inputs[i], e->d_name);
                }
                closedir(d);
            } else {
                if (n >= cap) {
                    cap *= 2;
                    list = realloc(list, sizeof(char *) * (size_t) cap);
                }
                list[n++] = strdup(inputs[i]);
            }
        }
        free(inputs);
        inputs = list;
        n_inputs = n;
        qsort(inputs, (size_t) n_inputs, sizeof(char *), cmp_names);
    }

    memset(&geo, 0, sizeof geo);
    if (pitch_given && pitch_override > 0.0) {
        geo.pitch = pitch_override;
        geo.n_scans = 0;
    } else {
        fprintf(stderr, "%s: measuring global geometry from %d scans\n",
                argv[0], n_inputs);
        if (measure_geometry(&geo, inputs, n_inputs, geom_sample, report) != 0) {
            fprintf(stderr, "%s: cannot measure sprocket pitch; pass --pitch PX\n",
                    argv[0]);
            return 1;
        }
    }
    if (fw_given) geo.frame_w = fw_opt;
    if (fh_given) geo.frame_h = fh_opt;
    if (!fw_given && geo.frame_w <= 0)
        geo.frame_w = (int) (MM_FRAME_W / MM_FRAME_ADVANCE * geo.pitch + 0.5);
    if (!fh_given && geo.frame_h <= 0)
        geo.frame_h = (int) (MM_FRAME_H / MM_FRAME_ADVANCE * geo.pitch + 0.5);
    if (ox_given)
        geo.offset_x = ox_opt;
    else if (geo.offset_x == 0)
        geo.offset_x = -geo.frame_w / 2;
    geo.offset_y = oy_given ? oy_opt : 0;
    fprintf(stderr, "%s: pitch=%.2f px frame=%dx%d offset_x=%d offset_y=%d\n",
            argv[0], geo.pitch, geo.frame_w, geo.frame_h, geo.offset_x,
            geo.offset_y);

    if (!dry_run) {
        mkdir(outdir, 0777);
        if (!render_dir) {
            render_buf = malloc(strlen(outdir) + 16);
            sprintf(render_buf, "%s/render", outdir);
            render_dir = render_buf;
        }
        mkdir(render_dir, 0777);
        csv_path = malloc(strlen(outdir) + 16);
        sprintf(csv_path, "%s/manifest.csv", outdir);
        csv = fopen(csv_path, "wb");
        if (!csv) {
            fprintf(stderr, "%s: cannot write %s: %s\n", argv[0], csv_path,
                    strerror(errno));
            free(csv_path);
            return 1;
        }
        fprintf(csv, "index,source,frame_in_scan,x,y,w,h,status,out_w,out_h\n");
    }

    for (s = 0; s < n_inputs; s++) {
        unsigned char *rgb = NULL, *gray = NULL;
        unsigned char *band_rgb = NULL, *band_gray = NULL;
        int w = 0, h = 0, y0 = 0, y1 = 0, bw = 0, bh = 0;
        int nf = 0, f = 0;
        double grad = 0.0;
        double pitch = 0.0, phase = 0.0;
        sf_strip_geometry strip;
        int n_holes = 0;
        ff_box boxes[SF_MAX_SPROCKETS];
        ff_box lb;
        unsigned char **rgb_crops = NULL, **gray_crops = NULL;
        unsigned char **stab_rgb = NULL;
        st_sequence *seq = NULL;
        const char *base = strrchr(inputs[s], '/');
        base = base ? base + 1 : inputs[s];

        if (read_jpeg_rgb(inputs[s], &rgb, &w, &h) != 0) {
            fprintf(stderr, "%s: cannot read %s\n", argv[0], inputs[s]);
            n_bad++;
            continue;
        }
        gray = malloc((size_t) w * h);
        if (!gray) {
            fprintf(stderr, "%s: out of memory\n", argv[0]);
            free(rgb);
            break;
        }
        rgb_to_gray(rgb, gray, w * h);

        if (sf_find_band(gray, w, h, &y0, &y1) != 0) {
            fprintf(stderr, "%s: %s: no film band found\n", argv[0], base);
            free(rgb);
            free(gray);
            continue;
        }
        /* Extract the band with the global position-dependent correction only
         * (angle = 0): no skew rotation, so the geometry is never shifted. */
        grad = geo.grad;
        band_rgb = sf_extract_band(rgb, w, h, 3, y0, y1, 0.0, grad, &bw, &bh);
        band_gray = sf_extract_band(gray, w, h, 1, y0, y1, 0.0, grad, &bw, &bh);
        free(rgb);
        rgb = NULL;
        free(gray);
        gray = NULL;
        if (!band_rgb || !band_gray) {
            fprintf(stderr, "%s: %s: rectify failed\n", argv[0], base);
            free(band_rgb);
            free(band_gray);
            continue;
        }

        /* Lattice from the sprocket holes on the rectified band: the hole row
         * is content-independent, so the pitch and phase come from the holes,
         * not from a band profile or a skew estimate. */
        {
            sf_sprocket sp[SF_MAX_SPROCKETS], spf[SF_MAX_SPROCKETS];
            int nsp = 0, nspf = 0;
            if (sf_find_sprockets(band_gray, bw, bh, 0, bh - 1, sp, &nsp) < 0 ||
                nsp < 2) {
                fprintf(stderr, "%s: %s: no sprocket holes in the band\n",
                        argv[0], base);
                free(band_rgb);
                free(band_gray);
                continue;
            }
            sf_filter_sprockets(sp, nsp, spf, &nspf);
            if (nspf < 2 || sf_fit_strip_geometry(spf, nspf, &strip) != 0) {
                fprintf(stderr, "%s: %s: hole row too short in the band\n",
                        argv[0], base);
                free(band_rgb);
                free(band_gray);
                continue;
            }
            pitch = strip.pitch;
            phase = strip.phase;
            n_holes = nspf;
        }

        /* Lattice spacing: this scan's own hole pitch, with the global model
         * as a sanity fallback. */
        if (!(pitch > PITCH_LO && pitch < PITCH_HI))
            pitch = geo.pitch;

        if (getenv("SF_DEBUG"))
            fprintf(stderr, "%s band %d..%d rect %dx%d pitch=%.2f "
                            "grad=%+.4f phase=%.0f holes=%d\n",
                    base, y0, y1, bw, bh, pitch, grad, phase, n_holes);

        if (report)
            fprintf(stderr, "%-16s band=%d..%d pitch=%6.1f phase=%6.0f "
                            "holes=%d frames=",
                    base, y0, y1, pitch, phase, n_holes);

        /* Fixed grid on the rectified band: vertical position from the band
         * centre, x from the hole lattice phase. */
        lb.x0 = 0;
        lb.y0 = 0;
        lb.w = geo.frame_w;
        lb.h = geo.frame_h;
        {
            int ytop = (bh - geo.frame_h) / 2 + geo.offset_y;
            int imax = (int) (bw / pitch) + 3;
            int k;
            for (k = -2; k <= imax; k++) {
                double x = phase + (double) k * pitch + geo.offset_x;
                int x0 = (int) floor(x + 0.5);
                if (x0 + geo.frame_w <= 0)
                    continue;
                if (x0 >= bw)
                    break;
                if (!include_partial &&
                    (x0 < 0 || x0 + geo.frame_w > bw))
                    continue;
                if (!include_partial &&
                    (ytop < 0 || ytop + geo.frame_h > bh))
                    continue;
                if (nf >= SF_MAX_SPROCKETS)
                    break;
                boxes[nf].x0 = x0;
                boxes[nf].y0 = ytop;
                boxes[nf].w = geo.frame_w;
                boxes[nf].h = geo.frame_h;
                nf++;
            }
        }

        if (report)
            fprintf(stderr, "%d\n", nf);

        if (nf <= 0) {
            fprintf(stderr, "%s: %s: no frames fit (pitch %.1f band %d)\n",
                    argv[0], base, geo.pitch, bh);
            free(band_rgb);
            free(band_gray);
            continue;
        }

        /* Crop every frame once; keep RGB and a luma copy. */
        rgb_crops = malloc((size_t) nf * sizeof(unsigned char *));
        gray_crops = malloc((size_t) nf * sizeof(unsigned char *));
        stab_rgb = calloc((size_t) nf, sizeof(unsigned char *));
        if (!rgb_crops || !gray_crops || !stab_rgb) {
            fprintf(stderr, "%s: out of memory\n", argv[0]);
            goto scan_done;
        }
        for (f = 0; f < nf; f++) {
            rgb_crops[f] = malloc((size_t) geo.frame_w * geo.frame_h * 3);
            gray_crops[f] = malloc((size_t) geo.frame_w * geo.frame_h);
            if (!rgb_crops[f] || !gray_crops[f]) {
                fprintf(stderr, "%s: out of memory\n", argv[0]);
                goto scan_done;
            }
            crop_rgb(band_rgb, bw, bh, &boxes[f], rgb_crops[f]);
            rgb_to_gray(rgb_crops[f], gray_crops[f],
                        geo.frame_w * geo.frame_h);
        }

        if (stabilize)
            seq = st_stabilize_sequence(boxes, nf,
                                        (const unsigned char **) gray_crops,
                                        geo.frame_w, geo.frame_h);

        for (f = 0; f < nf; f++) {
            unsigned char *src_rgb = rgb_crops[f];
            unsigned char *sig_gray = gray_crops[f];
            unsigned char *sig_owned = NULL;
            double sig[SIG_DIM * SIG_DIM];
            int dup = -1, has_sig = 0, ow = 0, oh = 0;
            unsigned char *out = NULL;

            if (seq) {
                st_affine *A = &seq->cum[f];
                stab_rgb[f] = sf_apply_affine_rgb(src_rgb, geo.frame_w,
                                                  geo.frame_h, A->a, A->b, A->c,
                                                  A->d, A->e, A->f, &ow, &oh);
                if (stab_rgb[f]) {
                    src_rgb = stab_rgb[f];
                    sig_owned = malloc((size_t) geo.frame_w * geo.frame_h);
                    if (sig_owned) {
                        rgb_to_gray(src_rgb, sig_owned,
                                    geo.frame_w * geo.frame_h);
                        sig_gray = sig_owned;
                    }
                }
            }

            if (dedup_thresh < 1.0)
                has_sig = crop_signature(sig_gray, geo.frame_w, &lb, sig);

            if (has_sig) {
                for (i = 0; i < n_sigs; i++) {
                    if (sig_sim(sig, sigs[i]) >= dedup_thresh) {
                        dup = sig_frame[i];
                        break;
                    }
                }
            }

            if (dup >= 0) {
                int dw, dh;
                oriented_size(geo.frame_w, geo.frame_h, rotate, &dw, &dh);
                if (csv)
                    fprintf(csv, "%d,%s,%d,%d,%d,%d,%d,dup_of_%d,%d,%d\n",
                            dup, base, f, boxes[f].x0, y0 + boxes[f].y0,
                            boxes[f].w, boxes[f].h, dup, dw, dh);
                free(sig_owned);
                continue;
            }

            if (has_sig) {
                if (n_sigs == sig_cap) {
                    int nc = sig_cap ? sig_cap * 2 : 256;
                    double **ns = realloc(sigs, sizeof(double *) * (size_t) nc);
                    int *nf2 = realloc(sig_frame, sizeof(int) * (size_t) nc);
                    if (!ns || !nf2) {
                        fprintf(stderr, "%s: out of memory\n", argv[0]);
                        exit(1);
                    }
                    sigs = ns;
                    sig_frame = nf2;
                    sig_cap = nc;
                }
                sigs[n_sigs] = malloc(sizeof sig);
                if (!sigs[n_sigs]) {
                    fprintf(stderr, "%s: out of memory\n", argv[0]);
                    exit(1);
                }
                memcpy(sigs[n_sigs], sig, sizeof sig);
                sig_frame[n_sigs] = index;
                n_sigs++;
            }

            if (!dry_run) {
                char *path = malloc(strlen(render_dir) + strlen(prefix) + 32);
                out = malloc((size_t) geo.frame_w * geo.frame_h * 3);
                if (!path || !out) {
                    fprintf(stderr, "%s: out of memory\n", argv[0]);
                    exit(1);
                }
                orient_rgb(src_rgb, geo.frame_w, geo.frame_h, rotate, negative,
                           out, &ow, &oh);
                sprintf(path, "%s/%s_%04d.png", render_dir, prefix, index);
                if (png_write_rgb(path, out, ow, oh) != 0) {
                    fprintf(stderr, "%s: cannot write %s\n", argv[0], path);
                } else if (csv) {
                    fprintf(csv, "%d,%s,%d,%d,%d,%d,%d,kept,%d,%d\n", index,
                            base, f, boxes[f].x0, y0 + boxes[f].y0,
                            boxes[f].w, boxes[f].h, ow, oh);
                }
                free(path);
                free(out);
            } else if (csv) {
                int dw, dh;
                oriented_size(geo.frame_w, geo.frame_h, rotate, &dw, &dh);
                fprintf(csv, "%d,%s,%d,%d,%d,%d,%d,kept,%d,%d\n", index, base,
                        f, boxes[f].x0, y0 + boxes[f].y0, boxes[f].w,
                        boxes[f].h, dw, dh);
            }
            index++;
            free(sig_owned);
        }

scan_done:
        st_free_sequence(seq);
        if (rgb_crops)
            for (f = 0; f < nf; f++)
                free(rgb_crops[f]);
        if (gray_crops)
            for (f = 0; f < nf; f++)
                free(gray_crops[f]);
        if (stab_rgb)
            for (f = 0; f < nf; f++)
                free(stab_rgb[f]);
        free(rgb_crops);
        free(gray_crops);
        free(stab_rgb);
        free(band_rgb);
        free(band_gray);
        free(rgb);
        free(gray);
    }

    if (csv)
        fclose(csv);
    fprintf(stderr, "%s: %d frames written to %s%s\n", argv[0], index,
            dry_run ? "(dry run, nothing written)" : render_dir,
            n_bad ? " [some inputs unreadable]" : "");

    for (i = 0; i < n_inputs; i++)
        free(inputs[i]);
    free(inputs);
    free(render_buf);
    for (i = 0; i < n_sigs; i++)
        free(sigs[i]);
    free(sigs);
    free(sig_frame);
    free(csv_path);
    return n_bad ? 1 : 0;
}
