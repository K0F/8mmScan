/* stabilize.c - frame-to-frame affine stabilization using sprocket-hole fiducials.
 *
 * Algorithm:
 *   1. For each adjacent pair (frame i, frame i+1):
 *      a. Detect sprockets within each frame's box (sf_frame_sprockets)
 *      b. Match sprockets by vertical proximity (expected pitch spacing)
 *      c. If ≥3 matches, solve 6-DOF affine via least-squares
 *      d. If <3 matches or RMS > threshold, use identity transform
 *   2. Accumulate transforms: C_i = T_{i-1} ∘ ... ∘ T_0
 *   3. Optional: smooth cumulative trajectory (moving average window)
 *   4. Each frame stabilized by applying C_i⁻¹ (inverse warp)
 *
 * All functions operate on 8-bit grayscale rasters, pure C99, no extra deps.
 */

#include "stabilize.h"
#include "sprocketfind.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/* -------------------------------------------------------------- constants */

#define ST_MIN_MATCHES 3          /* minimum inliers to solve affine */
#define ST_MAX_RMS    3.0         /* reject affine if RMS > this px   */
#define ST_RANSAC_ITERS 20        /* RANSAC iterations for robust fit   */
#define ST_SMOOTH_WIN  3          /* moving-average window for trajectory */

/* -------------------------------------------------------------- helpers */

/* Compute mean and std of residuals for affine solve.
 * Returns 0 on success, -1 if n < 3. */
static int st_residual_stats(int n, const int *x, const int *y,
                              const int *mx, const int *my,
                              double *mean_rms, double *rms)
{
    if (n < 3) return -1;
    double sum_sq = 0.0;
    for (int i = 0; i < n; i++) {
        double dx = (double)mx[i] - (double)x[i];
        double dy = (double)my[i] - (double)y[i];
        sum_sq += dx * dx + dy * dy;
    }
    *rms = sqrt(sum_sq / (double)n);
    *mean_rms = *rms;
    return 0;
}

/* Least-squares solve for affine from point correspondences:
 *   x' = a*x + b*y + c
 *   y' = d*x + e*y + f
 * Known: (x_i, y_i) → (x'_i, y'_i), i=0..n-1
 * Unknown: a,b,c,d,e,f
 * Sets up 2n equations in 6 unknowns, solves via normal equations. */
static st_affine *st_solve_ls(int n, const int *x, const int *y,
                               const int *xp, const int *yp)
{
    if (n < 3) return NULL;

    /* Normal equation matrices: 6x6 M, 6-vector B */
    double M[6][6] = {{0}};
    double B[6] = {0};

    for (int i = 0; i < n; i++) {
        int xi = x[i], yi = y[i];
        int xpi = xp[i], ypi = yp[i];

        /* Row for x' equation:  [x, y, 1, 0, 0, 0]  *  [a] = [x'] */
        /* Row for y' equation:  [0, 0, 0, x, y, 1]  *  [d] = [y'] */
        /* We'll interleave them. */
        /* Actually let's do standard: for each correspondence, add 2 rows. */
        /* x' = a*x + b*y + c  =>  [x y 1 0 0 0] • [a b c d e f]^T = x' */
        /* y' = d*x + e*y + f  =>  [0 0 0 x y 1] • [a b c d e f]^T = y' */

        M[0][0] += (double)(xi * xi);
        M[0][1] += (double)(xi * yi);
        M[0][2] += (double)xi;
        M[1][0] += (double)(xi * yi);
        M[1][1] += (double)(yi * yi);
        M[1][2] += (double)yi;
        M[1][3] += (double)xi;   /* y' row starts at col 3 */
        M[1][4] += (double)yi;
        M[1][5] += 1.0;

        M[2][0] += (double)xi;
        M[2][2] += 1.0;

        M[3][3] += (double)(xi * xi);
        M[3][4] += (double)(xi * yi);
        M[3][5] += (double)xi;
        M[4][4] += (double)(yi * yi);
        M[4][5] += (double)yi;
        M[5][5] += 1.0;

        B[0] += (double)xpi;
        B[1] += (double)ypi;
        B[2] += (double)xpi;
        B[3] += (double)xpi;
        B[4] += (double)ypi;
        B[5] += (double)ypi;
    }

    /* Gaussian elimination on 6x6 */
    /* ... simplified: just return identity for now, full impl later */
    st_affine *id = malloc(sizeof(st_affine));
    if (!id) return NULL;
    id->a = id->b = id->c = 0.0;
    id->d = id->e = id->f = 0.0;
    /* identity */
    id->a = 1.0; id->c = 0.0; id->b = 0.0;
    id->d = 0.0; id->f = 0.0; id->e = 1.0;
    return id;
}

/* -------------------------------------------------------------- st_solve_affine */

st_affine *st_solve_affine(int n, const int *matches_x, const int *matches_y)
{
    /* TODO: implement proper least-squares affine from n ≥ 3 point pairs.
     * For now return identity so the pipeline doesn't break. */
    (void)n; (void)matches_x; (void)matches_y;
    st_affine *a = malloc(sizeof(st_affine));
    if (!a) return NULL;
    a->a = 1.0; a->b = 0.0; a->c = 0.0;
    a->d = 0.0; a->e = 1.0; a->f = 0.0;
    return a;
}

/* -------------------------------------------------------------- st_stabilize_sequence */

st_sequence *st_stabilize_sequence(
    const ff_box *frames, int n_frames,
    const unsigned char **gray_crops, int crop_w, int crop_h)
{
    st_sequence *seq = calloc(1, sizeof(st_sequence));
    if (!seq) return NULL;

    seq->n_frames = n_frames;
    seq->pairwise = calloc((size_t)(n_frames - 1), sizeof(st_affine));
    seq->cum_x = calloc((size_t)n_frames, sizeof(double));
    seq->cum_y = calloc((size_t)n_frames, sizeof(double));
    seq->cum_angle = calloc((size_t)n_frames, sizeof(double));
    seq->cum_scale = calloc((size_t)n_frames, sizeof(double));
    seq->valid = calloc((size_t)n_frames, sizeof(int));
    if (!seq->pairwise || !seq->cum_x || !seq->cum_y ||
        !seq->cum_angle || !seq->cum_scale || !seq->valid) {
        free(seq->pairwise); free(seq->cum_x); free(seq->cum_y);
        free(seq->cum_angle); free(seq->cum_scale); free(seq->valid);
        free(seq);
        return NULL;
    }

    /* For each adjacent pair, detect sprockets and solve affine */
    /* NOTE: sf_frame_sprockets is a stub returning 0; integrate full CCL later.
     * Here we simulate with identity transforms so the pipeline runs. */
    for (int i = 0; i < n_frames - 1; i++) {
        /* Detect sprockets in frame i and i+1 within their boxes */
        /* In production: */
        /*   sf_frame_sprockets(gray_crops[i], crop_w, crop_h, &frames[i],
                        ST_MAX_SPROCKETS, tmp_x, tmp_y); */
        /*   ... match ... */
        /*   seq->pairwise[i] = *st_solve_affine(matches_n, ...); */

        /* Stub: identity */
        seq->pairwise[i].a = 1.0; seq->pairwise[i].b = 0.0; seq->pairwise[i].c = 0.0;
        seq->pairwise[i].d = 0.0; seq->pairwise[i].e = 1.0; seq->pairwise[i].f = 0.0;

        /* Cumulative accumulation */
        if (i == 0) {
            seq->cum_x[i] = 0.0;
            seq->cum_y[i] = 0.0;
            seq->cum_angle[i] = 0.0;
            seq->cum_scale[i] = 1.0;
            seq->valid[i] = 1;
        } else {
            seq->cum_x[i] = seq->cum_x[i-1] + 0.0;
            seq->cum_y[i] = seq->cum_y[i-1] + 0.0;
            seq->cum_angle[i] = seq->cum_angle[i-1] + 0.0;
            seq->cum_scale[i] = seq->cum_scale[i-1] + 1.0;
            seq->valid[i] = 1;
        }
    }

    /* Frame 0 is always valid (no prior transform) */
    seq->valid[0] = 1;
    seq->cum_x[0] = 0.0;
    seq->cum_y[0] = 0.0;
    seq->cum_angle[0] = 0.0;
    seq->cum_scale[0] = 1.0;

    return seq;
}

/* -------------------------------------------------------------- st_free_sequence */

void st_free_sequence(st_sequence *seq)
{
    if (!seq) return;
    free(seq->pairwise);
    free(seq->cum_x);
    free(seq->cum_y);
    free(seq->cum_angle);
    free(seq->cum_scale);
    free(seq->valid);
    free(seq);
}

/* -------------------------------------------------------------- st_apply_cum_transform */

int st_apply_cum_transform(
    const unsigned char *src, int crop_w, int crop_h,
    int frame_idx, const st_sequence *seq,
    unsigned char *dst)
{
    if (!seq || frame_idx < 0 || frame_idx >= seq->n_frames) return -1;

    /* For now: if valid and we have identity, just copy */
    if (!seq->valid[frame_idx]) return -1;

    /* Identity copy */
    if (dst && src && crop_w > 0 && crop_h > 0) {
        int n = crop_w * crop_h;
        for (int i = 0; i < n; i++)
            dst[i] = src[i];
        return 0;
    }
    return -1;
}

/* -------------------------------------------------------------- end */