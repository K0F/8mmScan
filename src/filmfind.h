/* filmfind.h - locate the film strip and the frame grid inside a scan.
 *
 * All functions operate on a plain 8-bit grayscale raster (w*h bytes, row
 * major) and contain no I/O, so they are directly unit testable.
 */
#ifndef FILMFIND_H
#define FILMFIND_H

#define FF_MAX_FRAMES 512

/* Sanity bounds for an Academy-ratio frame on a 35mm strip. */
#define FF_PITCH_MIN 300
#define FF_PITCH_MAX 1200

typedef struct {
    int x0, y0;   /* inclusive crop origin                                */
    int w, h;     /* crop size                                            */
} ff_box;

/* The film strip itself, as a rectangle of the raster. */
typedef struct {
    int x0, y0, x1, y1;   /* inclusive bounds                              */
    int w, h;
} ff_strip;

typedef struct {
    ff_strip strip;
    double pitch;           /* frame pitch in pixels                        */
    double px_per_mm;       /* strip height / film_mm                       */
    int phase;              /* detected frame phase, raster x of frame 0    */
    double phase_score;     /* comb brightness score at that phase (0..1)   */
    int nframes;
    ff_box frames[FF_MAX_FRAMES];
} ff_result;

/* Default strip geometry, in millimetres of the 35mm film width. */
#define FF_FILM_MM    35.0
#define FF_TOP_MM      7.1   /* strip top edge -> top of the image area      */
#define FF_HEIGHT_MM  23.7   /* image area height                            */

/* Finds the strip by comparing row and column medians against the background
 * level sampled at the raster edges, bridging dark content inside frames.
 * Returns 0 on success, -1 if no plausible strip was found. */
int ff_find_strip(const unsigned char *gray, int w, int h, ff_strip *out);

/* Estimates the frame pitch by autocorrelating the horizontal-gradient
 * profile of the picture area. Returns 0 on success, -1 on failure. */
int ff_fit_pitch(const unsigned char *gray, int w, int h, int y0, int y1,
                 double *out_pitch);

/* Builds the frame grid.
 *
 * In autodetect mode (pitch_override <= 0) the strip width is divided by the
 * number of whole frames the estimated pitch allows, and the grid is snapped
 * so that `n` frames tile the strip exactly. That removes the +-2% jitter of
 * the autocorrelation estimate, which otherwise decides erratically between
 * 7 and 8 frames on a 5728px-wide scan.
 *
 * `phase` is an extra x offset applied on top of the strip origin. Frames that
 * do not fit entirely inside the raster are dropped unless include_partial is
 * set. Returns the frame count, or -1 on error. */
int ff_build_frames(ff_result *out, int raster_w, int raster_h,
                    double pitch_override, double phase, double film_mm,
                    double top_mm, double height_mm, int include_partial);

/* The pitch ff_build_frames() will actually use in autodetect mode: the
 * autocorrelation pitch rounded to a whole number of frames tiling the strip.
 * Use this, not ff_result.pitch, when anything depends on the frame grid. */
double ff_tiled_pitch(const ff_result *out);

/* Tolerance, in grey levels, for calling a column uniform film base. */
#define FF_BASE_TOL 8

/* Finds the frame phase, i.e. the raster x of the first frame's left edge.
 *
 * Inter-frame gaps on a 35mm negative are clear film base: brighter than the
 * picture and, more reliably, almost perfectly uniform from the top of the
 * image area to the bottom. The comb of frame boundaries therefore has to land
 * on columns that are uniform over the picture-area rows, which is what this
 * maximises. Keying on uniformity rather than brightness is what keeps the
 * comb working on bright, high-key scenes.
 *
 * Searches every integer phase in [0, pitch) and keeps the best. `out_score`
 * is the mean uniformity under the comb and doubles as a confidence figure: a
 * low score means the scan has no clear base gaps to lock onto. Returns 0 on
 * success, -1 on failure. */
int ff_fit_phase(const unsigned char *gray, int w, int h, int y0, int y1,
                 double pitch, int *out_phase, double *out_score);

/* Median of n bytes, via a 256-bin histogram. */
int ff_median_u8(const unsigned char *v, int n);

#endif /* FILMFIND_H */
