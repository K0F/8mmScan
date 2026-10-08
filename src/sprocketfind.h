/* sprocketfind.h - detect 8mm film sprockets and compute strip geometry.
 *
 * All functions operate on a plain 8-bit grayscale raster (w*h bytes, row
 * major) and contain no I/O, so they are directly unit testable.
 *
 * Includes filmfind types (ff_box, ff_strip, ff_result) for frame box reuse.
 *
 * Sprocket holes are the bright circular/oval regions along the film edge
 * (Super 8: one row of holes beside the frame; Regular 8: holes along both edges).
 *
 * The detected strip geometry includes:
 *   - angle: radians, film strip angle vs horizontal (for deskew)
 *   - pitch: typical vertical spacing between sprockets (px)
 *   - cx, cy: estimated center of the strip
 *
 * Sprockets are also used as fiducial markers for frame-to-frame
 * affine stabilization (matching sprockets between adjacent frames).
 */

#ifndef SPROCKETFIND_H
#define SPROCKETFIND_H

#include "filmfind.h"

/* -------------------------------------------------------------- geometry */

#define SF_MAX_SPROCKETS 256

typedef struct {
    int x, y;          /* sprocket centre (pixel)            */
    int r;             /* bounding radius (pixel)            */
    double score;      /* detection confidence (0..1)        */
} sf_sprocket;

typedef struct {
    double angle;      /* radians, film strip tilt vs horizontal  */
    /* negative angle = film leans left-top, positive = right-top   */
    double cx, cy;     /* estimated strip centre (pixel)        */
    double pitch;      /* median vertical sprocket spacing (px) */
    int    n_sprockets;
} sf_strip_geometry;

/* -------------------------------------------------------------- API */

int sf_find_sprockets(const unsigned char *gray, int w, int h,
                      sf_sprocket *out, int *out_n);

int sf_filter_sprockets(const sf_sprocket *in, int n_in,
                        sf_sprocket *out, int *out_n);

int sf_fit_strip_geometry(const sf_sprocket *sprockets, int n,
                          sf_strip_geometry *out);

void sf_print_geometry(const sf_strip_geometry *g);

/* -------------------------------------------------------------- frame extraction */

#define SF_FRAME_W 400   /* default output frame width  (px) */
#define SF_FRAME_H 300   /* default output frame height (px) */
#define SF_OFFSET_X  50  /* sprocket-centre -> left-edge of frame (px) */
#define SF_OFFSET_Y -150 /* sprocket-centre -> top-edge of frame  (px) */

/*
 * Build frame boxes from strip geometry.  For each frame i (0..n-1):
 *   - sprocket centre at x_i = phase + i * pitch
 *   - frame left edge   = x_i + offset_x
 *   - frame top edge    = centre_y + offset_y
 *   - frame size        = frame_w x frame_h
 *
 * Returns number of frames, or -1 on error.
 */
int sf_build_frames(const sf_strip_geometry *geo, int raster_w,
                    int raster_h, int img_top_px, double phase,
                    ff_box *out, int *out_n);

/* -------------------------------------------------------------- skew correction */

 /*
  * Rotate the grayscale raster by -angle (deskew) around the image centre.
  * Writes `out_w * out_h` packed RGB bytes (same format as rgb_to_gray
  * output, i.e. not actually RGB but caller can treat as single channel).
  * Returns 0 on success, -1 on allocation failure.
  *
  * Caller must free `corrected`.
  */
unsigned char *sf_deskew(const unsigned char *gray, int w, int h, double angle,
                         int *out_w, int *out_h);

/* -------------------------------------------------------------- stab helpers */

 /*
  * Detect sprockets within a single frame crop.  The crop `box` is
  * positioned relative to a sprocket centre (as produced by sf_build_frames).
  * Writes up to `max_pts` matched point pairs to `out_pts` (x,y in crop coords).
  * Returns number of points found (0 if none).
  */
int sf_frame_sprockets(const unsigned char *gray, int w, int h, const ff_box *box,
                       int max_pts, int *out_pts_x, int *out_pts_y);

/*
 * Apply an affine transform (6 params: a b c d e f where
 *   x' = a*x + b*y + c,  y' = d*x + e*y + f)
 * to src (w*h grayscale) producing dst (out_w*out_h).
 * Bilinear interpolation.  Caller frees dst.
 */
unsigned char *sf_apply_affine(const unsigned char *src, int w, int h,
                                double a, double b, double c,
                                double d, double e, double f,
                                int *out_w, int *out_h);

/* -------------------------------------------------------------- deduplication reuse */

 /*
  * Reuse: compute a high-pass signature over a frame box, mean-removed and
  * unit-norm, for deduplication.  Same algorithm as filmfind's crop_signature()
  * but operates on 8mm-extracted frame coords.
  * Returns 1 if enough texture, 0 if flat.
  */
int sf_frame_signature(const unsigned char *gray, int w, int h,
                       const ff_box *bx, double *sig);

/* -------------------------------------------------------------- end */

#endif /* SPROCKETFIND_H */