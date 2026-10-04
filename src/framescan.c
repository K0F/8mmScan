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
"      --pitch PX          force frame pitch instead of autodetecting\n"
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
            prog, DEF_OUTDIR, FF_FILM_MM, FF_TOP_MM, FF_HEIGHT_MM,
            DEF_DEDUP_THRESH);
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
        int w = 0, h = 0, y0 = 0, y1 = 0, nf, f;
        double pitch = 0.0;
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

        if (pitch_override > 0.0) {
            res.pitch = pitch_override;
        } else if (ff_fit_pitch(gray, w, h, y0, y1, &pitch) != 0) {
            fprintf(stderr, "%s: %s: cannot estimate frame pitch\n", argv[0],
                    base);
            free(rgb); free(gray);
            continue;
        } else {
            res.pitch = pitch;
        }

        /* The strip's horizontal extent comes from a column-median test that
         * occasionally collapses onto a short bright section of an otherwise
         * full-width strip. The autocorrelation is also fooled by the short
         * strip, so re-estimate the pitch once the strip has been widened. */
        if (res.strip.w < (int) (3.0 * res.pitch)) {
            res.strip.x0 = 0;
            res.strip.x1 = w - 1;
            res.strip.w = w;
            if (pitch_override <= 0.0)
                ff_fit_pitch(gray, w, h, y0, y1, &res.pitch);
            fprintf(stderr, "%s: %s: strip width implausible, "
                            "assuming full-width film\n", argv[0], base);
        }

        /* Autodetect the frame phase unless the user pinned it. The comb of
         * frame boundaries has to land on the bright film-base gaps; starting
         * the grid at the strip edge instead lands it mid-frame. It has to be
         * fitted with the pitch the grid will use, not the raw estimate. */
        if (pitch_override <= 0.0)
            res.pitch = ff_tiled_pitch(&res);
        if (!phase_explicit && phase_auto && pitch_override <= 0.0) {
            int ap = 0, step, rel;
            double as = 0.0;
            if (ff_fit_phase(gray, w, h, y0, y1, res.pitch, &ap, &as) == 0) {
                step = (int) (res.pitch + 0.5);
                if (step > 0) {
                    rel = (ap - res.strip.x0) % step;
                    if (rel < 0)
                        rel += step;
                    phase = (double) rel;
                    res.phase = rel;
                    res.phase_score = as;
                }
            }
        }

        nf = ff_build_frames(&res, w, h, pitch_override, phase, film_mm,
                             top_mm, height_mm, include_partial);
        if (nf <= 0) {
            fprintf(stderr, "%s: %s: no frames fit (pitch %.1f)\n", argv[0],
                    base, res.pitch);
            free(rgb); free(gray);
            continue;
        }

        if (report) {
            fprintf(stderr,
                    "%-14s strip=x%4d..%-4d y%4d..%-4d (%dx%d) pitch=%6.1f "
                    "phase=%4d score=%.2f frames=%d\n",
                    base, res.strip.x0, res.strip.x1, res.strip.y0,
                    res.strip.y1, res.strip.w, res.strip.h, res.pitch,
                    res.phase, res.phase_score, nf);
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
                    fprintf(csv, "%d,%s,%d,%d,%d,%d,%d,dup_of_%d,%d,%d\n", index,
                            base, f, bx->x0, bx->y0, bx->w, bx->h, dup, ow, oh);
                }
                index++;
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
