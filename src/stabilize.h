/* stabilize.h - frame-to-frame affine stabilization using sprocket fiducials.
 *
 * Each extracted frame crop contains its own sprocket hole(s).  Matching the
 * holes between adjacent crops and solving an affine transform, then
 * accumulating and inverse-warping, removes the residual film wobble that the
 * whole-scan deskew cannot.
 *
 * For Super 8 there is one hole per frame, so only translation is observable;
 * the affine solver falls back to translation when handed fewer than three
 * correspondences.  Regular 8 / Double 8 has two rows of holes, which yields
 * rotation and scale as well.
 *
 * All functions operate on 8-bit grayscale rasters and contain no I/O.
 */

#ifndef STABILIZE_H
#define STABILIZE_H

#include "sprocketfind.h"   /* ff_box */

/* Affine transform: x' = a*x + b*y + c,  y' = d*x + e*y + f */
typedef struct {
    double a, b, c;
    double d, e, f;
} st_affine;

/* Result of stabilizing a frame sequence. */
typedef struct {
    st_affine *pairwise;   /* n-1 transforms: frame i -> frame i+1 */
    st_affine *cum;        /* n transforms: reference(0) -> frame i (smoothed) */
    int       *valid;      /* valid[i] = 1 if frame i had usable fiducials */
    int        n_frames;
} st_sequence;

/* -------------------------------------------------------------- API */

/* Detect the sprocket hole(s) in each crop's grayscale raster, match them
 * between adjacent crops and solve pairwise affine transforms.  Returns a
 * heap sequence (caller frees with st_free_sequence) or NULL on failure.
 * gray_crops[i] is crop_w*crop_h bytes; frames[i] is its box on the scan. */
st_sequence *st_stabilize_sequence(const ff_box *frames, int n_frames,
                                   const unsigned char **gray_crops,
                                   int crop_w, int crop_h);

void st_free_sequence(st_sequence *seq);

/* Inverse-warp frame `frame_idx` by its cumulative transform, writing
 * crop_w*crop_h bytes to `dst`.  Returns 0 on success, -1 on error. */
int st_apply_cum_transform(const unsigned char *src, int crop_w, int crop_h,
                           int frame_idx, const st_sequence *seq,
                           unsigned char *dst);

/* Least-squares affine mapping src points (sx[i],sy[i]) to dst points
 * (dx[i],dy[i]) for i in [0,n).  Falls back to translation for n < 3 or
 * degenerate configurations.  Returns a heap st_affine; caller frees. */
st_affine *st_solve_affine(int n, const double *sx, const double *sy,
                           const double *dx, const double *dy);

#endif /* STABILIZE_H */
