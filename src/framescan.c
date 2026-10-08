/* framescan.c - detect 35mm film frames in scanner JPEGs, crop them and
 * write sequentially numbered lossless PNGs.
 *
 * Pipeline per scan:
 *   1. decode to RGB + derive an 8-bit luma raster
 *   2. ff_find_band()      -> the film strip, by row-median vs background
 *   3. ff_fit_pitch()      -> frame pitch, by autocorrelating the horizontal
 *                             gradient profile of the picture area
 *   4. ff_build_frames()   -> butted frame boxes across the strip
 *   5. deduplicate against every frame already kept (normalised 16x16
 *      signature, cosine similarity) and write the survivors as PNG
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

#include "filmfind.h"
#include "pngwrite.h"

#define SIG_DIM 32
#define DEF_GEOM_SAMPLE_DEFAULT 24
#define DEF_DEDUP_THRESH 0.97
#define MIN_HP_RMS 1.5      /* below this a frame carries no usable identity */
#define DEF_OUTDIR      "out"

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

/* Dimensions of a crop after a clockwise rotation. */
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

/* Rotates a source-space crop clockwise by `rot` degrees (0, 90, 180 or 270)
 * and optionally inverts it, writing out_w x out_h packed RGB pixels to `dst`.
 *
 * The inversion is not cosmetic: a negative original scans as a positive,
 * so the frame lines come out black on white and the film reads the wrong way
 * round. Rotating puts the strip upright, since the frames run along it. */
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

/* Copy the RGB pixels of `bx` out of the full-frame raster. */
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
        /* Zero first: a frame may overhang the raster by the couple of pixels
         * ff_build_frames() tolerates for rounding, and those columns must not
         * be whatever the allocator handed us. */
        memset(d, 0, (size_t) bx->w * 3);
        memcpy(d, src, (size_t) n * 3);
        if (n < bx->w)
            memset(d + (size_t) n * 3, 0, (size_t) (bx->w - n) * 3);
    }
}

/* Deduplication signature: a 32x32 grid of mean values of the high-pass
 * filtered crop, mean-removed and unit-norm, so the cosine between two
 * signatures is a brightness- and contrast-invariant match score.
 *
 * The high-pass matters. On a plain mean-luma signature two *different*
 * frames of these washed-out scans reach cosine 0.988 because global
 * brightness dominates; high-passing drops the distinct-frame ceiling to
 * 0.923, which leaves the threshold somewhere sensible to sit.
 *
 * Returns 1 if the crop carries enough texture to identify it, 0 if it is
 * effectively flat. A blank frame has no identity and would otherwise match
 * every other blank frame, so the caller must not deduplicate those.
 */
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

    r = bw / 45;                    /* ~ one signature cell */
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

    /* Pass 1: horizontal running-sum box blur of the crop. */
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

    /* Pass 2: vertical blur, high-pass, and accumulation into the grid. */
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

    /* Too little high-pass energy to identify anything. */
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
"Detects 35mm film frames, crops them and writes numbered PNGs.\n"
"\n"
"Options:\n"
"  -o, --outdir DIR        output directory (default %s)\n"
"      --render-dir DIR    PNG subdirectory (default <outdir>/render)\n"
"      --prefix STR        output filename prefix (default frame)\n"
"      --rotate DEG        rotate frames clockwise: 0, 90, 180 or 270 (default 90)\n"
"      --no-rotate         do not rotate (same as --rotate 0)\n"
"      --positive          keep the scanned tonality instead of inverting it\n"
"      --phase PX          x of the first frame's left edge (default autodetect)\n"
"      --no-phase          do not autodetect phase; start at the strip edge\n"
"      --pitch PX          force frame pitch instead of measuring it\n"
"      --sample N          scans sampled to measure the global geometry\n"
"                         (default %d; every frame gets the same size)\n"
"      --film-mm F         film width in mm (default %.1f)\n"
"      --top-mm F          strip top edge to image top, mm (default %.1f)\n"
"      --height-mm F       image height, mm (default %.1f)\n"
"      --include-partial   also emit frames clipped by the image edge\n"
"      --no-dedup          keep every frame, including overlaps\n"
"      --dedup-thresh F    cosine similarity treated as duplicate\n"
"                         (default %.2f, 1.0 disables)\n"
"      --report            print per-scan geometry to stderr\n"
"      --dry-run           detect and report, write nothing\n"
"  -h, --help              this text\n",
            prog, DEF_OUTDIR, DEF_GEOM_SAMPLE_DEFAULT, FF_FILM_MM, FF_TOP_MM,
            FF_HEIGHT_MM, DEF_DEDUP_THRESH);
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

/* Measures the frame pitch, and the strip-top-to-image-top distance, over a
 * sample of scans spread across the whole set.
 *
 * This has to be global rather than per scan. Every per-scan estimate of the
 * film scale carries a couple of percent of error -- the strip edge is soft, so
 * its measured height ranged from 1254 to 1409px on this roll -- and letting
 * the frame size follow it produced PNGs of 902 to 954px. ffmpeg then had to
 * rescale every frame that did not match the render canvas, and a rescale is
 * exactly what reads as the film jumping.
 *
 * A median over the sample also rejects the scans where the rebate comb locks
 * onto something else, which a per-scan estimate would simply believe.
 * Returns 0 if a pitch was established. */
static int measure_geometry(ff_geometry *geo, char **paths, int n_paths,
                            int sample, int verbose)
{
    double *pitch, *topoff;
    int np = 0, nt = 0, i, step, bad = 0;

    pitch = malloc(sizeof(double) * (size_t) n_paths);
    topoff = malloc(sizeof(double) * (size_t) n_paths);
    if (!pitch || !topoff) {
        free(pitch);
        free(topoff);
        return -1;
    }

    if (sample < 3)
        sample = 3;
    step = n_paths / sample;
    if (step < 1)
        step = 1;

    for (i = 0; i < n_paths; i += step) {
        unsigned char *rgb = NULL, *gray = NULL;
        int w = 0, h = 0;
        double p = 0.0, conf = 0.0;
        ff_strip strip;
        const char *base = strrchr(paths[i], '/');
        base = base ? base + 1 : paths[i];

        if (read_jpeg_rgb(paths[i], &rgb, &w, &h) != 0) {
            bad++;
            continue;
        }
        gray = malloc((size_t) w * h);
        if (!gray) {
            free(rgb);
            break;
        }
        rgb_to_gray(rgb, gray, w * h);

        if (ff_find_strip(gray, w, h, &strip) == 0 &&
            ff_fit_rebate_pitch(gray, w, h, &strip, &p, &conf) == 0 &&
            conf >= 0.20) {
            pitch[np++] = p;
            topoff[nt++] = (double) strip.h;
        } else {
            bad++;
        }

        if (verbose)
            fprintf(stderr, "  geom %-14s strip_h=%4d pitch=%7.2f conf=%.2f\n",
                    base, strip.h, p, conf);
        free(rgb);
        free(gray);
    }

    if (np < 1) {
        fprintf(stderr, "framescan: no scan yielded a usable frame pitch\n");
        free(pitch);
        free(topoff);
        return -1;
    }

    geo->pitch = median_of(pitch, np);
    geo->n_scans = np;
    /* The strip height should come out at FF_FILM_MM * px_per_mm. Log the
     * spread rather than acting on it: the soft film edge makes the coverage
     * test overshoot by a row or two, and the median over the sample is what
     * keeps that from reaching the crop. */
    if (nt > 0) {
        double mh = median_of(topoff, nt);
        if (verbose)
            fprintf(stderr, "geometry: strip height median %.0f px vs %.0f px "
                            "expected\n",
                    mh, geo->pitch / FF_FRAME_MM * FF_FILM_MM);
    }

    if (verbose)
        fprintf(stderr, "geometry: pitch %.2f px over %d scans "
                        "(%.2f px/mm), %d skipped\n",
                geo->pitch, np, geo->pitch / FF_FRAME_MM, bad);

    free(pitch);
    free(topoff);
    return 0;
}

/* ------------------------------------------------------------------ main */

int main(int argc, char **argv)
{
    const char *outdir = DEF_OUTDIR;
    const char *render_dir = NULL;
    char *render_buf = NULL;
    const char *prefix = "frame";
    double phase = 0.0, pitch_override = 0.0;
    int phase_explicit = 0, phase_auto = 1;
    int geom_sample = DEF_GEOM_SAMPLE_DEFAULT;
    ff_geometry geo;
    int rotate = 90, negative = 1;
    double film_mm = FF_FILM_MM, top_mm = FF_TOP_MM, height_mm = FF_HEIGHT_MM;
    double dedup_thresh = DEF_DEDUP_THRESH;
    int include_partial = 0, dry_run = 0, report = 0;

    char **inputs;
    int n_inputs = 0, i, s;
    double **sigs = NULL;
    int *sig_frame = NULL, n_sigs = 0, sig_cap = 0;
    FILE *csv = NULL;
    int index = 0;
    int n_bad = 0;          /* inputs that could not be read at all */
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
        } else if (!strcmp(a, "--phase")) {
            if (++i >= argc) goto badarg;
            phase = atof(argv[i]);
            phase_explicit = 1;
        } else if (!strcmp(a, "--no-phase")) {
            phase_auto = 0;
        } else if (!strcmp(a, "--pitch")) {
            if (++i >= argc) goto badarg;
            pitch_override = atof(argv[i]);
        } else if (!strcmp(a, "--sample")) {
            if (++i >= argc) goto badarg;
            geom_sample = atoi(argv[i]);
            if (geom_sample < 3) {
                fprintf(stderr, "%s: --sample must be at least 3\n", argv[0]);
                return 2;
            }
        } else if (!strcmp(a, "--film-mm")) {
            if (++i >= argc) goto badarg;
            film_mm = atof(argv[i]);
        } else if (!strcmp(a, "--top-mm")) {
            if (++i >= argc) goto badarg;
            top_mm = atof(argv[i]);
        } else if (!strcmp(a, "--height-mm")) {
            if (++i >= argc) goto badarg;
            height_mm = atof(argv[i]);
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

    ff_geometry_defaults(&geo);
    if (pitch_override > 0.0) {
        geo.pitch = pitch_override;
        geo.n_scans = 0;
    } else {
        fprintf(stderr, "%s: measuring global geometry from %d scans\n",
                argv[0], n_inputs);
        if (measure_geometry(&geo, inputs, n_inputs, geom_sample, report) != 0) {
            fprintf(stderr, "%s: cannot measure frame pitch; pass --pitch PX\n",
                    argv[0]);
            return 1;
        }
    }
    if (geo.img_top_mm <= 0.0)
        geo.img_top_mm = top_mm;
    ff_geometry_finish(&geo, height_mm);
    fprintf(stderr, "%s: geometry pitch=%.2f px  %.2f px/mm  output %dx%d\n",
            argv[0], geo.pitch, geo.px_per_mm, geo.out_w, geo.out_h);

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
        int w = 0, h = 0, y0 = 0, y1 = 0, nf, f, img_top = 0;
        int ap = 0, step = 0;
        double as = 0.0, n_lines_found = -1.0;
        ff_result res;
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

        memset(&res, 0, sizeof res);
        if (ff_find_strip(gray, w, h, &res.strip) != 0) {
            fprintf(stderr, "%s: %s: no film strip found\n", argv[0], base);
            free(rgb); free(gray);
            continue;
        }
        y0 = res.strip.y0;
        y1 = res.strip.y1;

        /* Vertical anchor: the image area starts a fixed distance down the
         * film, so it follows from the strip top edge and the global scale.
         *
         * The old code scaled that distance by this scan's own strip height,
         * which is the actual bug behind the drifting y offset: strip heights
         * across this roll run 1255 to 1350px, so a constant 7.1mm became
         * anything from 254 to 274px and the crop edge moved with it. Holding
         * px_per_mm global pins the offset at 264px on every scan.
         */
        img_top = res.strip.y0 +
                  (int) (geo.img_top_mm * geo.px_per_mm + 0.5);
        if (img_top + geo.out_h > h) {
            fprintf(stderr, "%s: %s: image area runs off the bottom\n",
                    argv[0], base);
            free(rgb); free(gray);
            continue;
        }

        /* The strip's horizontal extent comes from a column-median test that
         * occasionally collapses onto a short bright section of an otherwise
         * full-width strip. Widen it rather than trust it. */
        if (res.strip.w < (int) (3.0 * geo.pitch)) {
            res.strip.x0 = 0;
            res.strip.x1 = w - 1;
            res.strip.w = w;
            fprintf(stderr, "%s: %s: strip width implausible, "
                            "assuming full-width film\n", argv[0], base);
        }

        /* Locate the frame lines and put the grid on them, unless the user
         * pinned the phase or asked for the old single-column comb.
         *
         * The frame line is a 150-200px band of clear film base spanning the
         * full image height, so its position can be found directly. Sampling
         * one column per frame, as ff_fit_phase() does, cannot tell positions
         * tens of pixels apart inside the same band, and that indeterminacy is
         * what put picture inside the crop edges on 86% of a full render.
         *
         * The lattice is fitted per scan because each scan's own pitch differs
         * from the roll median by a percent or two, but the crop is still the
         * global width, so every frame emitted stays the same size. */
        if (!phase_explicit && phase_auto) {
            double fx0 = 0.0, fpitch = geo.pitch, fscore = 0.0;
            if (ff_fit_frame_lines(gray, w, h, y0, y1, geo.pitch,
                                   &fx0, &fpitch, &fscore) == 0 &&
                fpitch > 0.5 * geo.pitch && fpitch < 2.0 * geo.pitch) {
                phase = fx0;
                while (phase < 0.0)
                    phase += fpitch;
                res.phase = (int) phase;
                res.phase_score = fscore;
                res.pitch = fpitch;
                n_lines_found = fscore;
            } else if (ff_fit_phase(gray, w, h, y0, y1, geo.pitch, &ap, &as) == 0) {
                /* Fall back to the comb, but only when the frame lines are not
                 * measurable at all; the comb is a guess inside a band, not a
                 * measurement of it. */
                step = (int) (geo.pitch + 0.5);
                if (step > 0) {
                    phase = (double) (ap % step);
                    if (phase < 0.0)
                        phase += step;
                    res.phase = (int) phase;
                    res.phase_score = as;
                    n_lines_found = -1.0;
                }
            } else {
                phase = 0.0;
            }
        }

        nf = ff_build_frames(&res, w, h, &geo, img_top, phase, film_mm,
                             include_partial);
        if (nf <= 0) {
            fprintf(stderr, "%s: %s: no frames fit (pitch %.1f)\n", argv[0],
                    base, res.pitch);
            free(rgb); free(gray);
            continue;
        }

        if (report) {
            fprintf(stderr,
                    "%-14s strip=x%4d..%-4d y%4d..%-4d (%dx%d) pitch=%6.1f "
                    "phase=%4d score=%.2f frames=%d found=%.2f\n",
                    base, res.strip.x0, res.strip.x1, res.strip.y0,
                    res.strip.y1, res.strip.w, res.strip.h, res.pitch,
                    res.phase, res.phase_score, nf, n_lines_found);
        }

        for (f = 0; f < nf; f++) {
            const ff_box *bx = &res.frames[f];
            double sig[SIG_DIM * SIG_DIM];
            int dup = -1;
            int has_sig = 0;

            /* Only crops with enough texture get a signature. A flat frame
             * leaves sig unnormalised and must be neither compared nor
             * stored, or it would match everything. */
            if (dedup_thresh < 1.0)
                has_sig = crop_signature(gray, w, bx, sig);

            if (has_sig) {
                double best = -1.0;
                for (i = 0; i < n_sigs; i++) {
                    double sv = sig_sim(sig, sigs[i]);
                    if (sv > best)
                        best = sv;
                    if (sv >= dedup_thresh) {
                        dup = sig_frame[i];
                        break;
                    }
                }
                if (getenv("FRAMESCAN_DEBUG"))
                    fprintf(stderr, "dbg %-14s f%d bestsim=%.4f\n", base, f, best);
            }

            if (dup >= 0) {
                if (csv) {
                    int ow, oh;
                    oriented_size(bx->w, bx->h, rotate, &ow, &oh);
                    /* The row carries the index of the frame it duplicates and
                     * does not consume one of its own. A duplicate has no PNG,
                     * so spending an index on it would leave a hole in the
                     * frame_%04d sequence, and ffmpeg's image2 demuxer stops
                     * dead at the first gap -- which is how two deduplicated
                     * frames once truncated the render at 1457 of 2805. */
                    fprintf(csv, "%d,%s,%d,%d,%d,%d,%d,dup_of_%d,%d,%d\n", dup,
                            base, f, bx->x0, bx->y0, bx->w, bx->h, dup, ow, oh);
                }
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
                char *path =
                    malloc(strlen(render_dir) + strlen(prefix) + 32);
                unsigned char *crop, *out;
                int ow = 0, oh = 0;
                crop = malloc((size_t) bx->w * bx->h * 3);
                out = malloc((size_t) bx->w * bx->h * 3);
                if (!crop || !out) {
                    fprintf(stderr, "%s: out of memory\n", argv[0]);
                    exit(1);
                }
                crop_rgb(rgb, w, h, bx, crop);
                orient_rgb(crop, bx->w, bx->h, rotate, negative, out, &ow, &oh);
                sprintf(path, "%s/%s_%04d.png", render_dir, prefix, index);
                if (png_write_rgb(path, out, ow, oh) != 0) {
                    fprintf(stderr, "%s: cannot write %s\n", argv[0], path);
                } else if (csv) {
                    fprintf(csv, "%d,%s,%d,%d,%d,%d,%d,kept,%d,%d\n", index, base,
                            f, bx->x0, bx->y0, bx->w, bx->h, ow, oh);
                }
                free(crop);
                free(out);
                free(path);
            } else if (csv) {
                int ow, oh;
                oriented_size(bx->w, bx->h, rotate, &ow, &oh);
                fprintf(csv, "%d,%s,%d,%d,%d,%d,%d,kept,%d,%d\n", index, base, f,
                        bx->x0, bx->y0, bx->w, bx->h, ow, oh);
            }
            index++;
        }

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
