/* mm8scan.c - 8mm film scanner with frame-to-frame stabilization and skew correction.
 *
 * Pipeline per scan:
 *   1. read_jpeg_rgb()        -> rgb, gray, w, h
 *   2. rgb_to_gray()          -> gray
 *   3. sf_find_sprockets()    -> detect sprocket holes
 *   4. sf_filter_sprockets()  -> filter & sort
 *   5. sf_fit_strip_geometry() -> angle, pitch, centre
 *   6. sf_deskew()            -> rotate full scan to correct skew
 *   7. sf_build_frames()      -> frame boxes at corrected positions
 *   8. Per-frame crops + signatures
 *   9. st_stabilize_sequence() -> frame-to-frame affine transforms
 *   10. st_apply_cum_transform() -> stabilized frames
 *   11. Write PNGs + manifest.csv
 *
 * Stabilization uses sprockets as fiducial markers:
 *   - Detect sprockets in each frame crop
 *   - Match sprockets between adjacent frames
 *   - Solve 6-DOF affine (translation + rotation + scale)
 *   - Accumulate transforms with optional smoothing
 *   - Inverse-warp each frame by its cumulative transform
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <setjmp.h>

#include <jpeglib.h>

#include "sprocketfind.h"
#include "stabilize.h"
#include "pngwrite.h"
#include "filmfind.h"

/* -------------------------------------------------------------- JPEG reading (copied from framescan.c) */

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

/* -------------------------------------------------------------- defaults */

#define DEFAULT_FRAME_W 400
#define DEFAULT_FRAME_H 300
#define DEFAULT_OFFSET_X 50
#define DEFAULT_OFFSET_Y -150
#define DEFAULT_SPROCKET_THRESH 220
#define DEFAULT_MAX_AFFINE_RMS 3.0
#define DEFAULT_SMOOTH_WINDOW 3

/* -------------------------------------------------------------- main */

int main(int argc, char **argv)
{
    const char *outdir = "out";
    const char *prefix = "frame";
    int rotate = 90;
    int dry_run = 0;
    int include_partial = 0;
    double film_mm = 35.0;

    /* Parse simple CLI options */
    int argi = 1;
    while (argi < argc) {
        if (!strcmp(argv[argi], "-o") || !strcmp(argv[argi], "--outdir")) {
            if (++argi >= argc) { fprintf(stderr, "bad -o argument\n"); return 2; }
            outdir = argv[argi++];
        } else if (!strcmp(argv[argi], "--prefix")) {
            if (++argi >= argc) { fprintf(stderr, "bad --prefix argument\n"); return 2; }
            prefix = argv[argi++];
        } else if (!strcmp(argv[argi], "--rotate")) {
            if (++argi >= argc) { fprintf(stderr, "bad --rotate argument\n"); return 2; }
            rotate = atoi(argv[argi++]);
        } else if (!strcmp(argv[argi], "--no-rotate")) {
            rotate = 0;
            argi++;
        } else if (!strcmp(argv[argi], "--positive")) {
            /* not used in mm8scan yet */
            argi++;
        } else if (!strcmp(argv[argi], "--negative")) {
            /* not used in mm8scan yet */
            argi++;
        } else if (!strcmp(argv[argi], "--no-stabilize")) {
            /* no-op: stabilization always on */
            argi++;
        } else if (argv[argi][0] == '-' && argv[argi][1] != '\0') {
            fprintf(stderr, "mm8scan: unknown option: %s\n", argv[argi]);
            return 2;
        } else {
            break; /* positional: scan file or directory */
        }
    }

    if (argi >= argc) {
        fprintf(stderr, "Usage: mm8scan [options] <scan.jpg|dir>\n");
        return 2;
    }

    /* Expand directories into JPEG files (simple: treat each arg as file for now) */
    int n_inputs = argc - argi;
    char **inputs = malloc((size_t)n_inputs * sizeof(char *));
    if (!inputs) return 1;
    for (int j = 0; j < n_inputs; j++) {
        inputs[j] = strdup(argv[argi + j]);
        if (!inputs[j]) { /* cleanup */ while (j-- > 0) free(inputs[j]); free(inputs); return 1; }
    }

    /* Create output dirs */
    mkdir(outdir, 0755);
    char render_dir[512];
    snprintf(render_dir, sizeof(render_dir), "%s/render", outdir);
    mkdir(render_dir, 0755);

    /* Open CSV manifest */
    char csv_path[512];
    snprintf(csv_path, sizeof(csv_path), "%s/manifest.csv", outdir);
    FILE *csv = fopen(csv_path, "w");
    if (!csv) { fprintf(stderr, "mm8scan: cannot create %s\n", csv_path); csv = NULL; }
    fprintf(csv, "index,source,frame_in_scan,x,y,w,h,status,out_w,out_h\n");

    /* Process each scan file */
    for (int s = 0; s < n_inputs; s++) {
        const char *path = inputs[s];
        fprintf(stderr, "mm8scan: processing %s\n", path);

        /* Read JPEG via filmfind */
        unsigned char *rgb = NULL;
        int w = 0, h = 0;
        if (read_jpeg_rgb(path, &rgb, &w, &h) != 0) {
            fprintf(stderr, "mm8scan: cannot read %s\n", path);
            continue;
        }

        /* Convert to grayscale */
        unsigned char *gray = malloc((size_t)w * h);
        if (!gray) { free(rgb); continue; }
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                int r = rgb[3 * (y * w + x)];
                int g = rgb[3 * (y * w + x) + 1];
                int b = rgb[3 * (y * w + x) + 2];
                gray[y * w + x] = (unsigned char)((299 * r + 587 * g + 114 * b) / 1000);
            }
        }

        /* Step 1: find sprockets */
        sf_sprocket *sprockets = malloc(SF_MAX_SPROCKETS * sizeof(sf_sprocket));
        int n_sprockets = 0;
        if (sf_find_sprockets(gray, w, h, sprockets, &n_sprockets) < 0) {
            fprintf(stderr, "mm8scan: sprocket detection failed for %s\n", path);
            free(rgb); free(gray); free(inputs[s]); continue;
        }

        /* Step 2: filter */
        sf_sprocket *filtered = malloc(SF_MAX_SPROCKETS * sizeof(sf_sprocket));
        int n_filtered = 0;
        sf_filter_sprockets(sprockets, n_sprockets, filtered, &n_filtered);

        /* Step 3: fit strip geometry (angle, pitch, centre) */
        sf_strip_geometry geo;
        if (sf_fit_strip_geometry(filtered, n_filtered, &geo) < 0) {
            fprintf(stderr, "mm8scan: strip geometry fit failed\n");
            free(rgb); free(gray); free(sprockets); free(filtered); free(inputs[s]); continue;
        }

        /* Step 4: deskew - rotate full scan by -angle */
        unsigned char *upright = sf_deskew(gray, w, h, geo.angle,
                                          &w, &h);
        if (!upright) {
            fprintf(stderr, "mm8scan: deskew failed\n");
            free(rgb); free(gray); free(sprockets); free(filtered); free(inputs[s]); continue;
        }
        free(gray); /* replaced by upright */
        gray = upright;

        /* Step 5: re-detect sprout on deskewed (use filtered coords approx) */
        /* In production: sf_find_sprockets(gray, w, h, ...) */

        /* Step 6: build frames from geometry */
        ff_box *frames = malloc(SF_MAX_SPROCKETS * sizeof(ff_box));
        int n_frames = 0;
        int img_top_px = (int)(geo.pitch * 7.1 / 19.05 + 0.5); /* approximate */
        double phase = 0.0;

        if (sf_build_frames(&geo, w, h, img_top_px, phase, frames, &n_frames) < 0) {
            fprintf(stderr, "mm8scan: frame build failed\n");
            free(rgb); free(gray); free(sprockets); free(filtered); free(inputs[s]); continue;
        }

        /* Step 7: extract frame crops (grayscale) */
        unsigned char **crop_gray = malloc((size_t)n_frames * sizeof(unsigned char *));
        if (!crop_gray) { /* error */ }
        int crop_w = DEFAULT_FRAME_W;
        int crop_h = DEFAULT_FRAME_H;

        for (int f = 0; f < n_frames; f++) {
            /* Crop using frames[f] from deskewed gray */
            /* Simplified: just allocate placeholder; full crop code needed */
            crop_gray[f] = NULL;
            /* TODO: implement actual crop from gray using frames[f].x0, frames[f].y0, w, h */
        }

        /* Step 8: frame-to-frame stabilization */
        st_sequence *seq = st_stabilize_sequence((const ff_box *)frames, n_frames,
                                                  (const unsigned char **)crop_gray,
                                                  crop_w, crop_h);
        if (!seq) {
            fprintf(stderr, "mm8scan: stabilization failed\n");
        }

        /* Step 9: apply transforms and write stabilized frames */
        for (int f = 0; f < n_frames; f++) {
            unsigned char *stab = malloc((size_t)crop_w * crop_h);
            if (seq && st_apply_cum_transform(gray, w, h, f, seq, stab) == 0) {
                /* Write PNG */
                char path[512];
                snprintf(path, sizeof(path), "%s/%s_%04d.png", render_dir, prefix, f);
                if (png_write_rgb(path, stab, crop_w, crop_h) != 0) {
                    fprintf(stderr, "mm8scan: cannot write %s\n", path);
                }
                if (csv) {
                    fprintf(csv, "%d,%s,%d,%d,%d,%d,%d,%d,%d\n", f, path, f,
                            frames[f].x0, frames[f].y0, crop_w, crop_h, crop_w, crop_h);
                }
            }
            free(stab);
        }

        st_free_sequence(seq);

        /* Cleanup per scan */
        free(rgb);
        free(gray);
        free(sprockets);
        free(filtered);
        free(frames);
        for (int f = 0; f < n_frames; f++) free(crop_gray[f]);
        free(crop_gray);
        free(inputs[s]);
    }

    if (csv) fclose(csv);
    fprintf(stderr, "mm8scan: done.\n");

    /* Free inputs */
    for (int j = 0; j < n_inputs; j++) free(inputs[j]);
    free(inputs);

    return 0;
}