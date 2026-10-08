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

/* Here the film band is the modal region of the raster, so a global-mode
 * background picks the wrong thing; the edge-row mode has to rescue it. */
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
    *fh_out = (int) (FF_HEIGHT_MM * pitch / FF_FRAME_MM + 0.5);
    *fy_out = y0 + (int) (FF_TOP_MM * pitch / FF_FRAME_MM + 0.5);
    return g;
}

/* The fixture is self-consistent with the film model: a 735px strip over 35mm
 * is 21.0 px/mm, and 21.0 * 19.05 = 400.05, so the frame pitch really is
 * T_PITCH. The old T_BH of 770 was not, which only showed up once the geometry
 * stopped being derived from the strip height. */
#define T_W    1200
#define T_H    1100
#define T_Y0   150
#define T_BH   735
#define T_PITCH 400

static void geom_for(ff_geometry *g, double pitch)
{
    ff_geometry_defaults(g);
    g->pitch = pitch;
    ff_geometry_finish(g, FF_HEIGHT_MM);
}

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

static void set_strip(ff_result *r, int x0, int y0, int x1, int y1)
{
    memset(r, 0, sizeof *r);
    r->strip.x0 = x0;
    r->strip.y0 = y0;
    r->strip.x1 = x1;
    r->strip.y1 = y1;
    r->strip.w = x1 - x0 + 1;
    r->strip.h = y1 - y0 + 1;
}

static void test_frames(void)
{
    int fw = 0, fh = 0, fy = 0, pitch = T_PITCH, bh = T_BH, y0 = T_Y0;
    int w = T_W, h = T_H;
    unsigned char *g = three_frames(w, h, y0, bh, pitch, &fw, &fh, &fy);
    ff_result r;
    ff_geometry geo;
    int n;

    geom_for(&geo, (double) pitch);

    set_strip(&r, 0, y0, w - 1, y0 + bh - 1);
    n = ff_build_frames(&r, w, h, &geo, fy, 0.0, FF_FILM_MM, 0);
    ok(n == 3, "three whole frames fit");
    ok(r.frames[0].x0 == 0, "first frame at x=0");
    ok(r.frames[1].x0 == T_PITCH, "second frame at x=pitch");
    ok(r.frames[2].x0 == 2 * T_PITCH, "third frame at x=2*pitch");
    ok(r.frames[0].w == geo.out_w,
       "crop width comes from the global geometry, not this scan");
    ok(r.frames[0].h == geo.out_h, "crop height comes from the global geometry");
    ok(r.frames[0].y0 == fy, "crop top is the image-area top");
    near(r.px_per_mm, (double) pitch / FF_FRAME_MM, 1e-9, "px_per_mm");

    /* Phase shifts the grid on top of x=0. */
    set_strip(&r, 0, y0, w - 1, y0 + bh - 1);
    n = ff_build_frames(&r, w, h, &geo, fy, 40.0, FF_FILM_MM, 0);
    ok(n == 2, "phase 40 drops the clipped third frame");
    ok(r.frames[0].x0 == 40, "phase honoured");

    /* Partial frames are opt-in and get clamped. */
    set_strip(&r, 0, y0, w - 1, y0 + bh - 1);
    n = ff_build_frames(&r, w, h, &geo, fy, 40.0, FF_FILM_MM, 1);
    ok(n == 3, "partial mode keeps the trailing frame");
    ok(r.frames[2].x0 == 840 && r.frames[2].w == w - 840,
       "trailing frame clamped to raster width");

    free(g);
    printf("  frames: grid layout\n");
}

/* The whole point of the geometry pass: whatever the individual strip heights
 * are, every frame comes out at the same size. */
static void test_frames_uniform_size(void)
{
    ff_result r;
    ff_geometry geo;
    int n, i;
    static const double strips[] = { 1255, 1294, 1322, 1350, 1409 };

    geom_for(&geo, 704.74);
    ok(geo.out_w == 704 && geo.out_h == 876, "real-roll output is 704x876");

    for (i = 0; i < (int) (sizeof strips / sizeof strips[0]); i++) {
        int bh = strips[i];
        int img_top = 100 + (int) (FF_TOP_MM * geo.px_per_mm + 0.5);
        set_strip(&r, 0, 100, 5727, 100 + bh - 1);
        n = ff_build_frames(&r, 5728, 3824, &geo, img_top, 0.0, FF_FILM_MM, 0);
        ok(n > 0, "frames found");
        ok(r.frames[0].w == 704 && r.frames[0].h == 876,
           "frame size independent of this scan's strip height");
    }
    printf("  frames: one size for every scan\n");
}

/* yuv420p needs both output dimensions even. */
static void test_geometry_even(void)
{
    static const double pitches[] = { 703.9, 704.74, 705.3, 716.0, 719.1,
                                      927.0, 936.5 };
    size_t i;
    for (i = 0; i < sizeof pitches / sizeof pitches[0]; i++) {
        ff_geometry geo;
        geom_for(&geo, pitches[i]);
        checks++;
        if (geo.out_w % 2 || geo.out_h % 2) {
            failures++;
            printf("    FAIL pitch %.2f gave odd output %dx%d\n",
                   pitches[i], geo.out_w, geo.out_h);
        }
    }
    printf("  geometry: output dimensions always even\n");
}

static void test_frames_rejects(void)
{
    ff_result r;
    ff_geometry geo, bad;

    geom_for(&geo, 400.0);
    set_strip(&r, 0, 150, 1199, 150 + 734);
    bad = geo; bad.pitch = 5.0;
    ok(ff_build_frames(&r, 1200, 1100, &bad, 200, 0.0, FF_FILM_MM, 0) == -1,
       "absurd pitch refused");
    set_strip(&r, 0, 150, 1199, 150 + 734);
    r.strip.h = 0;
    ok(ff_build_frames(&r, 1200, 1100, &geo, 200, 0.0, FF_FILM_MM, 0) == -1,
       "zero strip height refused");
    ok(ff_build_frames(NULL, 1200, 1100, &geo, 200, 0.0, FF_FILM_MM, 0) == -1,
       "NULL result refused");
    ok(ff_build_frames(&r, 1200, 1100, NULL, 200, 0.0, FF_FILM_MM, 0) == -1,
       "NULL geometry refused");
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
    test_frames_uniform_size();
    test_geometry_even();
    test_frames_rejects();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
