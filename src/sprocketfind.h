/* sprocketfind.h - locate 8mm film sprocket holes and compute strip geometry.
 *
 * All functions operate on a plain 8-bit grayscale raster (w*h bytes, row
 * major) and contain no I/O, so they are directly unit testable.
 *
 * An 8mm scan shows the film as a horizontal band with the picture frames
 * running along it.  The sprocket holes sit in the rebate along one edge
 * (Super 8) or both edges (Regular 8 / Double 8); they are the one reference
 * that does not depend on picture content, so they carry both the strip skew
 * and the frame pitch.
 *
 * The detected strip geometry includes:
 *   - angle: radians, film strip angle vs horizontal (for deskew)
 *   - pitch: along-strip spacing between sprockets (px) = frame advance
 *   - cx, cy: centroid of the sprocket row
 *   - film_w: thickness of the film band across the strip (px)
 */

#ifndef SPROCKETFIND_H
#define SPROCKETFIND_H

#define SF_MAX_SPROCKETS 512

/* A crop box in raster coordinates. */
typedef struct {
    int x0, y0;   /* inclusive crop origin */
    int w, h;     /* crop size             */
} ff_box;

typedef struct {
    int x, y;          /* sprocket centre (pixel)            */
    int r;             /* bounding radius (pixel)            */
    double score;      /* detection confidence (0..1)        */
} sf_sprocket;

typedef struct {
    double angle;      /* radians, film strip tilt vs horizontal  */
    double cx, cy;     /* centroid of the sprocket row (pixel)    */
    double pitch;      /* median along-strip sprocket spacing     */
    double phase;      /* along-strip x of the first sprocket     */
    int    film_w;     /* film band thickness across the strip    */
    int    band_y0;    /* film band top in the raster             */
    int    band_y1;    /* film band bottom in the raster          */
    int    n_sprockets;
} sf_strip_geometry;

/* Whole-band geometry model.  The RODFS50 (and scanners like it) image the
 * film with a fixed, position-dependent scale: the along-strip perforation
 * pitch is a little larger along the bottom edge of the strip than along the
 * top.  `pitch_top`/`pitch_bot` capture that; `grad` in the global model is
 * the fractional difference, and rectify_* resamples every row back to a
 * single reference pitch so the frame lattice is uniform. */
typedef struct {
    double pitch_top;   /* along-strip pitch near the band top edge         */
    double pitch_bot;   /* along-strip pitch near the band bottom edge      */
    double pitch;       /* reference pitch, mean of the two                 */
    double phase;       /* comb phase of the stronger edge (raster px)      */
    double angle;       /* strip tilt from the top/bottom phase shift (rad) */
    double cy;          /* y of the stronger (perforation) edge in raster   */
    int    band_y0, band_y1, band_h;
    int    hole_dir;    /* 0 = stronger edge near top, 1 = near bottom      */
    double score;       /* normalised autocorrelation strength of that edge */
} sf_band;

/* -------------------------------------------------------------- API */

/* Finds the horizontal film band by coverage: a row belongs to the film when
 * most of its width differs from the modal scanner background.  Writes the
 * inclusive row bounds.  Returns 0 on success, -1 if no band was found. */
int sf_find_band(const unsigned char *gray, int w, int h,
                 int *out_y0, int *out_y1);

/* Modal scanner background level (the value just outside the film band). */
int sf_background_level(const unsigned char *gray, int w, int h);

/* Strip tilt in radians, fitted from the top and bottom edges of the band as
 * functions of x.  Robust for the small (0..2 deg) tilts a flatbed film
 * holder produces, where fitting the perforation row is noisy. */
double sf_band_angle(const unsigned char *gray, int w, int h,
                     int band_y0, int band_y1, int bg);

/* Detects sprocket holes inside the band [band_y0, band_y1].  Both dark and
 * bright polarities are tried and the one forming the most regular row is
 * kept.  Writes up to SF_MAX_SPROCKETS centres, sorted along the strip.
 * Returns the number found (>= 0), or -1 on allocation failure. */
int sf_find_sprockets(const unsigned char *gray, int w, int h,
                      int band_y0, int band_y1,
                      sf_sprocket *out, int *out_n);

int sf_filter_sprockets(const sf_sprocket *in, int n_in,
                        sf_sprocket *out, int *out_n);

/* Analyses the periodic feature (perforations or frame lines) of the strip in
 * [band_y0, band_y1].  Measures the along-strip pitch separately near the top
 * and bottom edges, the comb phase of the stronger edge, and the strip tilt
 * from the shift of that phase between the two edges.  pitch_min/pitch_max
 * bound the autocorrelation lag search (use a narrow window once the pitch is
 * known, e.g. 550..950 for 8mm at this scanner's resolution).  Returns 0 on
 * success, -1 if no periodicity was found. */
int sf_analyze_band(const unsigned char *gray, int w, int h,
                    int band_y0, int band_y1,
                    double pitch_min, double pitch_max, sf_band *out);

/* ---------------------------------------------------------- rectification */

/* Resample the band [band_y0, band_y1] so that the along-strip pitch no longer
 * varies across it: row `v` (0..1 top..bottom) is sampled about the horizontal
 * centre with a factor 1 + grad*(v-0.5), cancelling the model's position
 * dependence.  The result is band_h rows tall.  Caller frees. */
unsigned char *sf_rectify_gray(const unsigned char *gray, int w, int h,
                               int band_y0, int band_y1, double grad,
                               int *out_w, int *out_h);
unsigned char *sf_rectify_rgb(const unsigned char *rgb, int w, int h,
                              int band_y0, int band_y1, double grad,
                              int *out_w, int *out_h);

/* Deskew (by the strip `angle`) and rectify (by `grad`) in a single resample,
 * returning just the band.  Cheaper and avoids rotating the whole scan.
 * Caller frees. */
unsigned char *sf_extract_band(const unsigned char *src, int w, int h, int ch,
                               int band_y0, int band_y1, double angle,
                               double grad, int *out_w, int *out_h);

/* Fits strip geometry (angle, pitch, phase, centre) from sprocket centres.
 * A straight line is fitted robustly with Theil-Sen over all pairs, then the
 * sprockets are projected onto it for the pitch.  Returns 0 on success, -1 if
 * there are too few points. */
int sf_fit_strip_geometry(const sf_sprocket *sprockets, int n,
                          sf_strip_geometry *out);

void sf_print_geometry(const sf_strip_geometry *g);

/* -------------------------------------------------------------- frames */

/* Builds frame boxes along the strip.  Frames are placed at the sprocket
 * pitch starting from geo->phase and offset from the sprocket row by
 * `offset_x` (along) and `offset_y` (across).  Every frame is frame_w x
 * frame_h so the render never rescales.  Frames that do not fit entirely are
 * dropped unless include_partial is set.  Returns the frame count, or -1. */
int sf_build_frames(const sf_strip_geometry *geo, int raster_w, int raster_h,
                    int frame_w, int frame_h, int offset_x, int offset_y,
                    int include_partial, ff_box *out, int *out_n);

/* -------------------------------------------------------------- skew */

/* Rotate a grayscale / packed-RGB raster by -angle about its centre, bilinear.
 * Output has the same dimensions.  Caller frees. */
unsigned char *sf_deskew_gray(const unsigned char *gray, int w, int h,
                              double angle, int *out_w, int *out_h);
unsigned char *sf_deskew_rgb(const unsigned char *rgb, int w, int h,
                             double angle, int *out_w, int *out_h);

/* -------------------------------------------------------------- stab */

/* Detect sprocket fiducials within a frame crop `gray` (w*h).  `box` is the
 * frame in the crop's own coordinates (usually the whole crop).  Writes up to
 * max_pts matched centres to out_pts_x/out_pts_y (crop coords).  Returns the
 * number found. */
int sf_frame_sprockets(const unsigned char *gray, int w, int h,
                       const ff_box *box, int max_pts,
                       int *out_pts_x, int *out_pts_y);

/* Bilinearly sample src at out(x,y) = src(a*x + b*y + c, d*x + e*y + f).
 * Output is the same size as src.  Caller frees. */
unsigned char *sf_apply_affine(const unsigned char *src, int w, int h,
                               double a, double b, double c,
                               double d, double e, double f,
                               int *out_w, int *out_h);

/* Same transform, packed RGB input/output.  Caller frees. */
unsigned char *sf_apply_affine_rgb(const unsigned char *src, int w, int h,
                                   double a, double b, double c,
                                   double d, double e, double f,
                                   int *out_w, int *out_h);

/* -------------------------------------------------------------- end */

#endif /* SPROCKETFIND_H */
