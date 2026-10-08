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

/* Bell & Howell 35mm: perforations on a 4.7625mm pitch, four per frame. The
 * perforations themselves are washed out on most of this roll -- the rebate is
 * clear base in many scans and blown out in others, so the holes carry no
 * usable signal and cannot be located directly. What survives is the frame
 * period they imply, which is what FF_FRAME_MM encodes: measuring that period
 * in the rebate and converting it through FF_PERF_MM is how the crop gets its
 * scale. */
#define FF_PERF_MM      4.7625
#define FF_PERFS_FRAME  4
#define FF_FRAME_MM    (FF_PERF_MM * FF_PERFS_FRAME)   /* 19.05 */

/* Grey levels away from the background that count as "not the background". */
#define FF_BG_TOL     8
/* Fraction of the raster width a row must cover to count as film. */
#define FF_COVER_MIN  0.90

/* Finds the strip by measuring, for every row, what fraction of its width is
 * not the flat scanner background, and keeping the longest run of rows that
 * cover more than FF_COVER_MIN of it.
 *
 * Coverage rather than a brightness threshold is what makes this survive real
 * scans: the area around the film is lit very unevenly (measured background
 * levels across this roll range from 0 to 81), so any fixed threshold either
 * swallows several hundred rows of background or cuts into the film itself.
 * A row that is film is non-background along its whole width no matter how
 * bright it happens to be, which also rejects dust specks and scanner blemishes
 * that a per-row median would happily include.
 *
 * Returns 0 on success, -1 if no plausible strip was found. */
int ff_find_strip(const unsigned char *gray, int w, int h, ff_strip *out);

/* Geometry measured once over a sample of scans and then applied to all of
 * them, so that every frame the tool emits is exactly the same size.
 *
 * Per-scan estimates are not good enough for this. The strip edge is soft, so
 * a per-scan strip height carries a few percent of error; letting the frame
 * height follow it produced PNGs ranging from 902 to 954px tall, and ffmpeg
 * then had to rescale each one to the render canvas, which is what made the
 * film appear to jump between frames. */
typedef struct {
    double pitch;        /* frame pitch in pixels, median of the sample   */
    double px_per_mm;    /* pitch / FF_FRAME_MM                           */
    double img_top_mm;   /* strip top edge -> image area top               */
    int    out_w, out_h; /* even output size, valid after the crop rotates  */
    int    n_scans;      /* scans that contributed to the median           */
} ff_geometry;

/* Fills in the constants that follow from the film format. */
void ff_geometry_defaults(ff_geometry *g);

/* Derives px_per_mm from the measured pitch and rounds the rotated output size
 * down to the nearest even numbers, as yuv420p requires. Call once the sample
 * median pitch is known. `height_mm` is the image-area height. */
void ff_geometry_finish(ff_geometry *g, double height_mm);

/* Measures the frame pitch from the top rebate, i.e. the 3mm band just inside
 * the strip's top edge where the perforations sit.
 *
 * The rebate is the only part of the film that is free of picture content, so
 * its column profile carries the frame period and nothing else; across this
 * roll that estimate held to 705.20 +- 3.75px (0.5%) on 49 scans. The
 * picture-area gradient autocorrelation in ff_fit_pitch() is the less reliable
 * of the two, as it can lock onto a subharmonic of the frame rate.
 *
 * Returns 0 on success with `out_conf` set to the normalised autocorrelation
 * peak (0..1), -1 if no confident peak was found. */
int ff_fit_rebate_pitch(const unsigned char *gray, int w, int h,
                        const ff_strip *st, double *out_pitch,
                        double *out_conf);

/* Estimates the frame pitch by autocorrelating the horizontal-gradient
 * profile of the picture area. Returns 0 on success, -1 on failure. */
int ff_fit_pitch(const unsigned char *gray, int w, int h, int y0, int y1,
                 double *out_pitch);

/* Builds the frame grid.
 *
 * The grid uses the global `geo` geometry rather than anything derived from
 * this scan alone, so every frame of every scan comes out at the same size.
 * `img_top_px` is the per-scan image-area top from ff_find_image_top(); the
 * strip's own height is deliberately not used for scale.
 *
 * `phase` is an extra x offset applied on top of the detected frame phase.
 * Frames that do not fit entirely inside the raster are dropped unless
 * include_partial is set. Returns the frame count, or -1 on error. */
int ff_build_frames(ff_result *out, int raster_w, int raster_h,
                    const ff_geometry *geo, int img_top_px, double phase,
                    double film_mm, int include_partial);

/* Tolerance, in grey levels, for calling a column uniform film base. */
#define FF_BASE_TOL 8

/* Grey level at or above which a row counts as clear film base. Clear base is
 * the brightest thing on a negative scan, so this is a generous cut used only
 * to count rows, never as the sole test for a frame line. */
#define FF_CLEAR_LEVEL 245

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

/* Finds the frame lines themselves and fits a lattice to them.
 *
 * A frame line is a wide run of columns that are clear film base from the top
 * of the image area to the bottom, so it can be found and its position pinned
 * down directly. ff_fit_phase() below only samples a single column per frame
 * position, which cannot distinguish positions tens of pixels apart inside the
 * same 150-200px band; that is not good enough to put the crop edges on the
 * frame line.
 *
 * `pitch_hint` seeds the search. On success `out_x0` is the raster x of a frame
 * line, `out_pitch` this scan's own frame pitch, and `out_score` the fraction
 * of the expected frame lines that were actually located (1.0 = every one).
 * Returns 0 on success, -1 if too few frame lines were found. */
int ff_fit_frame_lines(const unsigned char *gray, int w, int h, int y0, int y1,
                       double pitch_hint, double *out_x0, double *out_pitch,
                       double *out_score);

/* Median of n bytes, via a 256-bin histogram. */
int ff_median_u8(const unsigned char *v, int n);

#endif /* FILMFIND_H */
