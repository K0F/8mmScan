/* fixture.h - synthetic Super-8 strip renderer shared by the e2e tests.
 *
 * A scan shows a horizontal film band: one perforation per frame sits in the
 * rebate along the top edge of the band, and the picture runs across the rest
 * of the band between the perforations.  The perforations are what make the
 * along-strip pitch show up as an autocorrelation peak (exactly as on real
 * film); each frame's picture is random vertical bands so neighbouring frames
 * and scans never look alike unless they are the same scan.
 *
 * `leader` shifts the whole frame run sideways; `blank` leaves one frame
 * without picture texture (an unexposed frame).  A detector that ignores the
 * actual perforation phase would land its grid mid-frame.
 */
#ifndef FIXTURE_H
#define FIXTURE_H

#include <math.h>
#include <stdlib.h>

static unsigned int fx_xs = 1u;

static void fx_seed(unsigned int s)
{
    fx_xs = s ? s : 1u;
}

static unsigned int fx_rand(void)
{
    fx_xs ^= fx_xs << 13;
    fx_xs ^= fx_xs >> 17;
    fx_xs ^= fx_xs << 5;
    return fx_xs;
}

static int fx_clamp(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* Draws frame `f` of an 8mm strip that starts at y0 with band height band_h
 * and pitch.  On the top edge a perforation is cut at the cell centre; the
 * picture fills the rest of the cell below it. */
static void fx_draw_frame(unsigned char *img, int w, int h, int y0, int band_h,
                          int pitch, int f, unsigned int seed_off, int flat,
                          int leader)
{
    int fx = leader + f * pitch;
    int fw = (fx + pitch <= w) ? pitch : w - fx;
    int perf_top = y0 + (int) (0.03 * band_h);
    int perf_bot = y0 + (int) (0.16 * band_h);
    int perf_w = (int) (0.24 * pitch);
    int perf_cx = fx + pitch / 2;
    unsigned char *base;
    int x, y;

    if (fw <= 0)
        return;

    base = malloc((size_t) fw);
    if (!base)
        return;

    fx_seed(7919u * (unsigned) (f + 1) + seed_off);

    /* Picture area: random vertical bands, distinctive per frame.  A blank
     * frame keeps a uniform field (with one seed-dependent marker so two of
     * them are not pixel-identical). */
    if (flat) {
        unsigned int r;
        int mx, my, mv;
        for (y = perf_bot; y < y0 + band_h && y < h; y++) {
            unsigned char *p = img + (size_t) y * w;
            for (x = 0; x < fw; x++)
                p[fx + x] = 140;
        }
        r = fx_rand();
        mx = 20 + (int) (r % (unsigned) (fw > 60 ? fw - 60 : 1));
        my = perf_bot + 20 + (int) ((r >> 8) % 60u);
        mv = 20 + (int) ((r >> 16) % 200u);
        for (y = my; y < my + 24 && y < y0 + band_h && y < h; y++) {
            unsigned char *p = img + (size_t) y * w;
            for (x = mx; x < mx + 24 && fx + x < w; x++)
                p[fx + x] = (unsigned char) mv;
        }
    } else {
        x = 0;
        while (x < fw) {
            int seg = 10 + (int) (fx_rand() % 50u);
            int val = 100 + (int) (fx_rand() % 50u);
            while (seg-- > 0 && x < fw)
                base[x++] = (unsigned char) val;
        }
        for (y = perf_bot; y < y0 + band_h && y < h; y++) {
            unsigned char *p = img + (size_t) y * w;
            for (x = 0; x < fw; x++) {
                int v = base[x] + (int) (fx_rand() % 25u) - 12;
                p[fx + x] = (unsigned char) fx_clamp(v);
            }
        }
    }

    /* Perforation: clear film base is bright, so the perf is a bright cut in
     * the rebate, centred on the cell (matches a real negative scan). */
    for (y = perf_top; y < perf_bot && y < h; y++) {
        unsigned char *p = img + (size_t) y * w;
        int px0 = perf_cx - perf_w / 2;
        int px1 = perf_cx + perf_w / 2;
        for (x = px0; x < px1 && x < w; x++)
            if (x >= 0)
                p[x] = 250;
    }

    /* Frame line at the cell boundary, from below the perf zone to the bottom
     * rebate: real film has a clear line between frames, and it is what lets
     * the (perf-less) bottom edge of the scan also show a periodic profile at
     * the pitch -- but it must stay out of the perf zone (top strip), or the
     * comb phase would blend the two bright features. */
    for (y = perf_bot + (int) (0.20 * band_h); y < y0 + band_h && y < h; y++) {
        unsigned char *p = img + (size_t) y * w;
        int gw = (int) (0.08 * pitch);
        int x;
        for (x = fx; x < fx + gw && x < w; x++)
            if (x >= 0)
                p[x] = 245;
    }

    free(base);
}

/* Builds a whole synthetic scan: dark background, mid-grey film band, `nf`
 * frames with one perforation each. */
static unsigned char *fx_raster(int w, int h, int y0, int band_h, int pitch,
                                int nf, unsigned int seed_off, int blank,
                                int leader)
{
    unsigned char *img = malloc((size_t) w * h);
    int x, y, f;

    if (!img)
        return NULL;
    for (y = 0; y < h; y++)
        for (x = 0; x < w; x++)
            img[(size_t) y * w + x] = 10;
    /* The strip spans the whole raster, as on the real scans. */
    for (y = y0; y < y0 + band_h && y < h; y++)
        for (x = 0; x < w; x++)
            img[(size_t) y * w + x] = 128;
    for (f = 0; f < nf && leader + f * pitch < w; f++)
        fx_draw_frame(img, w, h, y0, band_h, pitch, f, seed_off,
                      blank >= 0 && f == blank, leader);
    return img;
}

#endif /* FIXTURE_H */