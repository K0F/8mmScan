/* fixture.h - synthetic 35mm strip renderer shared by the unit and e2e tests.
 *
 * Each frame gets uncorrelated large-scale structure (random vertical bands),
 * a bright film-base gap at the cell boundary and a fiducial bar in the top
 * margin. The gap is what makes the frame pitch show up as an autocorrelation
 * peak, exactly as it does on real film, and its brightness is what lets the
 * phase detector find where the first frame starts; the random bands are what
 * keep neighbouring frames from looking like duplicates.
 *
 * `leader` shifts the whole frame run sideways, imitating film that starts
 * partway across the scan. A detector that ignores it lands the grid
 * mid-frame, which is the bug this models.
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

/* Renders frame `f` of a strip that starts at (y0, band_h) with the given
 * pitch. Only the parts of the frame inside the raster are drawn. */
static void fx_draw_frame(unsigned char *img, int w, int h, int y0, int band_h,
                          int pitch, int f, unsigned int seed_off, int flat,
                          int leader)
{
    int fx = leader + f * pitch;
    int fw = (fx + pitch <= w) ? pitch : w - fx;
    int fy = y0 + (int) (0.23 * band_h);
    int fh = (int) (0.70 * band_h) - (int) (0.23 * band_h);
    int gap_w = (int) (0.16 * pitch);
    int bw = (int) (0.26 * pitch);
    int bx = fx + gap_w + (int) (0.16 * pitch);
    unsigned char *base;
    int x, y;

    if (fw <= 0)
        return;

    base = malloc((size_t) fw);
    if (!base)
        return;

    fx_seed(7919u * (unsigned) (f + 1) + seed_off);

    /* A "flat" frame keeps its frame line and fiducial (so pitch detection is
     * unaffected) but leaves the picture area uniform, i.e. no high-pass
     * texture at all. Real film contains such frames, and they must never
     * contribute a signature. */
    if (flat) {
        unsigned int r;
        int mx, my, mv;
        for (y = fy; y < fy + fh && y < y0 + band_h && y < h; y++) {
            unsigned char *p = img + (size_t) y * w;
            for (x = 0; x < fw; x++)
                p[fx + x] = 140;
        }
        /* One sparse seed-dependent marker. Without it two blank frames would
         * be pixel-identical and legitimately de-duplicate as 1.0, which tells
         * us nothing about the low-texture path. */
        r = fx_rand();
        mx = 20 + (int) (r % (unsigned) (fw > 60 ? fw - 60 : 1));
        my = fy + 20 + (int) ((r >> 8) % 60u);
        mv = 20 + (int) ((r >> 16) % 200u);
        for (y = my; y < my + 24 && y < fy + fh && y < y0 + band_h && y < h; y++) {
            unsigned char *p = img + (size_t) y * w;
            for (x = mx; x < mx + 24 && fx + x < w; x++)
                p[fx + x] = (unsigned char) mv;
        }
    } else {
    /* Random vertical bands: distinctive per frame. */
    x = 0;
    while (x < fw) {
        int seg = 20 + (int) (fx_rand() % (unsigned) (fw / 4 + 1));
        int val = 30 + (int) (fx_rand() % 200u);
        while (seg-- > 0 && x < fw)
            base[x++] = (unsigned char) val;
    }

    for (y = fy; y < fy + fh && y < y0 + band_h && y < h; y++) {
        unsigned char *p = img + (size_t) y * w;
        for (x = 0; x < fw; x++) {
            int v = base[x] + (int) (fx_rand() % 25u) - 12;
            p[fx + x] = (unsigned char) fx_clamp(v);
        }
    }
    }

    /* Film-base gap at the cell boundary: bright, and taller than the picture
     * area so it reads as a clear base band from edge to edge. Real Academy
     * gaps run about a sixth of the pitch, and the gap has to be that wide or
     * Otsu cannot find it against the picture histogram. */
    for (y = fy; y < y0 + band_h && y < h; y++) {
        unsigned char *p = img + (size_t) y * w;
        for (x = 0; x < gap_w && fx + x < w; x++)
            p[fx + x] = 250;
    }

    /* Fiducial bar in the top margin. */
    for (y = y0 + (int) (0.05 * band_h); y < y0 + (int) (0.19 * band_h) && y < h;
         y++) {
        unsigned char *p = img + (size_t) y * w;
        for (x = 0; x < bw && bx + x < w; x++)
            p[bx + x] = 35;
    }

    free(base);
}

/* Builds a whole synthetic scan: dark background, mid-grey strip, `nf` frames. */
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
    /* The strip spans the whole raster, as on the real scans: the frames sit
     * inside it and a leader of clear base precedes the first frame. */
    for (y = y0; y < y0 + band_h && y < h; y++)
        for (x = 0; x < w; x++)
            img[(size_t) y * w + x] = 128;
    for (f = 0; f < nf && leader + f * pitch < w; f++)
        fx_draw_frame(img, w, h, y0, band_h, pitch, f, seed_off,
                      blank >= 0 && f == blank, leader);
    return img;
}

#endif /* FIXTURE_H */
