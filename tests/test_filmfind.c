/* test_filmfind.c - unit tests for the strip and frame-grid detection. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "filmfind.h"
#include "fixture.h"

static int failures;
static int checks;

static void ok(int cond, const char *what)
{
    checks++;
    if (!cond) {
        failures++;
        printf("    FAIL %s\n", what);
    }
}

static void near(double got, double want, double tol, const char *what)
{
    checks++;
    if (fabs(got - want) > tol) {
        failures++;
        printf("    FAIL %s: got %.2f want %.2f +/- %.2f\n", what, got, want, tol);
    }
}

/* Raster helper: `fill(y,x)` decides the pixel value. */
static unsigned char *make_raster(int w, int h, unsigned char (*fill)(int, int))
{
    unsigned char *g = malloc((size_t) w * h);
    int x, y;
    for (y = 0; y < h; y++)
        for (x = 0; x < w; x++)
            g[(size_t) y * w + x] = fill(y, x);
    return g;
}

static void test_median(void)
{
    unsigned char v[9] = { 200, 1, 1, 1, 1, 1, 1, 1, 250 };
    unsigned char u[4] = { 10, 20, 30, 40 };
    ok(ff_median_u8(v, 9) == 1, "median picks lower middle");
    ok(ff_median_u8(u, 4) == 20, "median of even count");
    ok(ff_median_u8(v, 0) == 0, "median of empty");
    printf("  median\n");
}

static unsigned char flat_bg(int y, int x)
{
    (void) x;
    return (y < 200 || y >= 700) ? 10 : 128;
}

static void test_band_basic(void)
{
    int w = 1200, h = 900;
    ff_strip st;
    unsigned char *g = make_raster(w, h, flat_bg);
    ok(ff_find_strip(g, w, h, &st) == 0, "strip found");
    ok(st.y0 == 200, "strip y0 exact");
    ok(st.y1 == 699, "strip y1 exact");
    ok(st.h == 500, "strip height");
    ok(st.x0 == 0 && st.x1 == w - 1, "strip spans full raster width");
    ok(st.w == w, "strip width");
    free(g);
    printf("  strip: basic\n");
}

static unsigned char band_with_hole(int y, int x)
{
    if (y < 300 || y >= 800)
        return 8;
    if (y >= 450 && y < 560)   /* dark content inside a frame */
        return 12;
    return 200;
}

static void test_band_gap_closing(void)
{
    int w = 1200, h = 1000;
    ff_strip st;
    unsigned char *g = make_raster(w, h, band_with_hole);
    ok(ff_find_strip(g, w, h, &st) == 0, "strip found across dark hole");
    ok(st.y0 == 300, "gap-closed strip y0");
    ok(st.y1 == 799, "gap-closed strip y1");
    free(g);
    printf("  strip: bridges dark content\n");
}

static unsigned char two_bands(int y, int x)
{
    (void) x;
    if (y >= 100 && y < 200)  return 240;   /* short bright decoy */
    if (y >= 400 && y < 900)  return 130;   /* the real film strip */
    return 5;
}

static void test_band_picks_tallest(void)
{
    int w = 800, h = 1000;
    ff_strip st;
    unsigned char *g = make_raster(w, h, two_bands);
    ok(ff_find_strip(g, w, h, &st) == 0, "strip found");
    ok(st.y0 == 400 && st.y1 == 899, "tallest strip chosen over short decoy");
    free(g);
    printf("  strip: tallest wins\n");
}

static void test_band_rejects(void)
{
    int w = 800, h = 1000;
    ff_strip st;
    unsigned char *g = make_raster(w, h, flat_bg);
    memset(g, 10, (size_t) w * h);           /* no strip at all */
    ok(ff_find_strip(g, w, h, &st) != 0, "uniform raster rejected");
    ok(ff_find_strip(NULL, 10, 10, &st) != 0, "NULL rejected");
    ok(ff_find_strip(g, 0, 0, &st) != 0, "zero size rejected");
    free(g);
    printf("  strip: rejects degenerate input\n");
}

static unsigned char *three_frames(int w, int h, int y0, int bh, int pitch,
                                   int *fw_out, int *fh_out, int *fy_out)
{
    unsigned char *g = fx_raster(w, h, y0, bh, pitch, (w + pitch - 1) / pitch, 0, -1, 0);
    *fw_out = pitch;
    *fh_out = (int) (FF_HEIGHT_MM * bh / FF_FILM_MM + 0.5);
    *fy_out = y0 + (int) (FF_TOP_MM * bh / FF_FILM_MM + 0.5);
    return g;
}

#define T_W 1200
#define T_H 1100
#define T_Y0 150
#define T_BH 770
#define T_PITCH 400

static void test_pitch(void)
{
    int fw, fh, fy;
    unsigned char *g = three_frames(T_W, T_H, T_Y0, T_BH, T_PITCH, &fw, &fh, &fy);
    double p = 0.0;
    ok(ff_fit_pitch(g, T_W, T_H, T_Y0, T_Y0 + T_BH - 1, &p) == 0,
       "pitch estimated");
    near(p, (double) T_PITCH, 6.0, "pitch recovered");
    free(g);
    printf("  pitch: autocorrelation\n");
}

static void set_strip(ff_result *r, int x0, int y0, int x1, int y1, double pitch)
{
    memset(r, 0, sizeof *r);
    r->strip.x0 = x0;
    r->strip.y0 = y0;
    r->strip.x1 = x1;
    r->strip.y1 = y1;
    r->strip.w = x1 - x0 + 1;
    r->strip.h = y1 - y0 + 1;
    r->pitch = pitch;
}

static void test_frames(void)
{
    int fw = 0, fh = 0, fy = 0, pitch = T_PITCH, bh = T_BH, y0 = T_Y0;
    int w = T_W, h = T_H;
    unsigned char *g = three_frames(w, h, y0, bh, pitch, &fw, &fh, &fy);
    ff_result r;
    int n;

    set_strip(&r, 0, y0, w - 1, y0 + bh - 1, (double) pitch);
    n = ff_build_frames(&r, w, h, 0.0, 0.0, FF_FILM_MM, FF_TOP_MM,
                        FF_HEIGHT_MM, 0);
    ok(n == 3, "three whole frames fit");
    ok(r.frames[0].x0 == 0, "first frame at strip left edge");
    ok(r.frames[1].x0 == T_PITCH, "second frame at x=pitch");
    ok(r.frames[2].x0 == 2 * T_PITCH, "third frame at x=2*pitch");
    ok(r.frames[0].w == T_PITCH,
       "crop width equals pitch (frames butt together)");
    ok(r.frames[0].h == fh, "crop height from mm geometry");
    ok(r.frames[0].y0 == fy, "crop top from mm geometry");
    near(r.px_per_mm, (double) bh / FF_FILM_MM, 1e-9, "px_per_mm");

    /* A strip that does not start at x=0 shifts the whole grid. The raster is
     * widened so the offset strip still has room for three whole frames. */
    set_strip(&r, 137, y0, 137 + 1199, y0 + bh - 1, (double) pitch);
    n = ff_build_frames(&r, 1400, h, 0.0, 0.0, FF_FILM_MM, FF_TOP_MM,
                        FF_HEIGHT_MM, 0);
    ok(n == 3, "offset strip still yields three frames");
    ok(r.frames[0].x0 == 137, "grid anchored to strip left edge");

    /* Phase shifts the grid on top of the strip origin. */
    set_strip(&r, 0, y0, w - 1, y0 + bh - 1, (double) pitch);
    n = ff_build_frames(&r, w, h, 0.0, 40.0, FF_FILM_MM, FF_TOP_MM,
                        FF_HEIGHT_MM, 0);
    ok(n == 2, "phase 40 drops the clipped third frame");
    ok(r.frames[0].x0 == 40, "phase honoured");

    /* Partial frames are opt-in and get clamped. */
    set_strip(&r, 0, y0, w - 1, y0 + bh - 1, (double) pitch);
    n = ff_build_frames(&r, w, h, 0.0, 40.0, FF_FILM_MM, FF_TOP_MM,
                        FF_HEIGHT_MM, 1);
    ok(n == 3, "partial mode keeps the trailing frame");
    ok(r.frames[2].x0 == 840 && r.frames[2].w == 360,
       "trailing frame clamped to raster width");

    free(g);
    printf("  frames: grid layout\n");
}

/* The autocorrelation estimate carries a couple of percent of jitter. On a
 * 5728px-wide scan that is the difference between 7 and 8 frames, so the grid
 * must be snapped to the strip width rather than trusting the raw estimate. */
static void test_frames_snap_to_strip(void)
{
    static const double est[] = { 703.8, 709.1, 716.0, 718.9, 722.5 };
    ff_result r;
    size_t i;

    for (i = 0; i < sizeof est / sizeof est[0]; i++) {
        int n;
        set_strip(&r, 0, 2026, 5727, 2026 + 1375, est[i]);
        n = ff_build_frames(&r, 5728, 3824, 0.0, 0.0, FF_FILM_MM, FF_TOP_MM,
                            FF_HEIGHT_MM, 0);
        checks++;
        if (n != 8) {
            failures++;
            printf("    FAIL pitch %.1f gave %d frames, want 8\n", est[i], n);
            continue;
        }
        near(r.pitch, 5728.0 / 8.0, 1e-6, "snapped pitch");
        ok(r.frames[0].x0 == 0 && r.frames[7].x0 + r.frames[7].w == 5728,
           "8 frames tile the strip exactly");
    }
    printf("  frames: snaps to strip width\n");
}

/* An explicit --pitch must be respected verbatim, not re-snapped. */
static void test_frames_manual_pitch(void)
{
    ff_result r;
    int n;
    set_strip(&r, 0, 2026, 5727, 2026 + 1375, 719.0);
    n = ff_build_frames(&r, 5728, 3824, 715.0, 0.0, FF_FILM_MM, FF_TOP_MM,
                        FF_HEIGHT_MM, 0);
    ok(n == 8, "manual pitch yields eight frames");
    near(r.pitch, 715.0, 1e-9, "manual pitch not re-snapped");
    printf("  frames: manual pitch respected\n");
}

static void test_frames_rejects(void)
{
    ff_result r;
    set_strip(&r, 0, 150, 1199, 150 + 769, 400.0);
    ok(ff_build_frames(&r, 1200, 1100, 5.0, 0.0, FF_FILM_MM, FF_TOP_MM,
                       FF_HEIGHT_MM, 0) == -1, "absurd pitch refused");
    set_strip(&r, 0, 150, 1199, 150 + 769, 400.0);
    r.strip.h = 0;
    ok(ff_build_frames(&r, 1200, 1100, 0.0, 0.0, FF_FILM_MM, FF_TOP_MM,
                       FF_HEIGHT_MM, 0) == -1, "zero strip height refused");
    ok(ff_build_frames(NULL, 1200, 1100, 0.0, 0.0, FF_FILM_MM, FF_TOP_MM,
                       FF_HEIGHT_MM, 0) == -1, "NULL result refused");
    printf("  frames: rejects bad input\n");
}

int main(void)
{
    printf("filmfind unit tests\n");
    test_median();
    test_band_basic();
    test_band_gap_closing();
    test_band_picks_tallest();
    test_band_rejects();
    test_pitch();
    test_frames();
    test_frames_snap_to_strip();
    test_frames_manual_pitch();
    test_frames_rejects();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
