/* stabilize.h - frame-to-frame affine stabilization using sprocket-hole fiducials.
 *
 * Core idea: each extracted frame contains sprocket holes at known positions.
 * Match sprockets between adjacent frames → solve 6-DOF affine transform →
 * accumulate transforms → inverse-warp frames to stabilize.
 *
 * All functions operate on 8-bit grayscale rasters, contain no I/O.
 * Uses ff_box from filmfind.h for frame box I/O.
 */

#ifndef STABILIZE_H
#define STABILIZE_H

#include "sprocketfind.h"
#include "filmfind.h"

/* -------------------------------------------------------------- affine */

 /* Affine transform matrix: x' = a*x + b*y + c,  y' = d*x + e*y + f */
typedef struct {
    double a, b, c; /* x-mapping coefficients */
    double d, e, f; /* y-mapping coefficients */
} st_affine;

/* Result of stabilizing a frame sequence */
typedef struct {
    st_affine *pairwise;   /* n-1 transforms: frame i → i+1   */
    double *cum_x;         /* cumulative translation x per frame  */
    double *cum_y;         /* cumulative translation y per frame  */
    double *cum_angle;     /* cumulative rotation per frame       */
    double *cum_scale;     /* cumulative scale per frame          */
    int   *valid;          /* valid[i] = 1 if frame i stable    */
    int   n_frames;
} st_sequence;

/* -------------------------------------------------------------- API */

/* Compute pairwise affine transforms between adjacent frames using sprocket matching.
 * frames[i] are the extracted frame boxes (from sf_build_frames), and the
 * corresponding grayscale crops must be provided in `gray_crops[i]` (w*h).
 * Returns sequence on heap; caller frees with st_free_sequence().
 */
st_sequence *st_stabilize_sequence(
    const ff_box *frames, int n_frames,
    const unsigned char **gray_crops, int crop_w, int crop_h);

/* Free sequence allocated by st_stabilize_sequence() */
void st_free_sequence(st_sequence *seq);

/* Apply the cumulative transform for frame i to stabilize it.
 * src is the original frame crop (crop_w*crop_h), dst gets the stabilized result.
 * Returns 0 on success, -1 on error.
 */
int st_apply_cum_transform(
    const unsigned char *src, int crop_w, int crop_h,
    int frame_idx, const st_sequence *seq,
    unsigned char *dst);

/* Solve affine from matched point pairs (at least 3 non-collinear).
 * matches_x[], matches_y[] have `n` entries.
 * Returns affine on heap; caller frees.
 */
st_affine *st_solve_affine(int n, const int *matches_x, const int *matches_y);

/* -------------------------------------------------------------- end */

#endif /* STABILIZE_H */