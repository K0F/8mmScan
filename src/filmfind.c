/* filmfind.c - film strip and frame grid detection. See filmfind.h. */
#include "filmfind.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define EDGE_ROWS     60    /* rows sampled at each end for background level */
#define MIN_BAND_H   200    /* a strip shorter than this is not a strip       */
#define GAP_CLOSE    150    /* rows of dark picture content bridged over     */
#define MIN_TOL       14    /* minimum "brighter than background" threshold   */
#define TOL_FRAC     0.05   /* ... or this fraction of the available range    */

int ff_median_u8(const unsigned char *v, int n)
{
    long hist[256];
    long half, acc = 0;
    int i, k;

    if (n <= 0)
        return 0;
    memset(hist, 0, sizeof hist);
    for (i = 0; i < n; i++)
        hist[v[i]]++;
    half = (long) ((n + 1) / 2);
    for (k = 0; k < 256; k++) {
        acc += hist[k];
        if (acc >= half)
            return k;
    }
    return 255;
}

/* Longest run of `prof` entries above `thr`, bridging gaps shorter than
 * GAP_CLOSE so that dark content inside a frame does not split the strip. */
static void longest_run(const unsigned char *prof, int n, int thr, int min_len,
                        int *out_lo, int *out_hi)
{
    unsigned char *m;
    int i, lo = -1, best = 0, blo = 0, bhi = -1;

    if (n <= 0) {
        *out_lo = 0;
        *out_hi = -1;
        return;
    }

    m = malloc((size_t) n);
    if (!m) {
        *out_lo = 0;
        *out_hi = -1;
        return;
    }
    for (i = 0; i < n; i++)
        m[i] = (unsigned char) (prof[i] > thr);

    /* Bridge short gaps so dark content inside a frame does not split it. */
    for (i = 0; i < n; i++) {
        int j;
        if (m[i])
            continue;
        j = i;
        while (j < n && !m[j])
            j++;
        if (j - i < GAP_CLOSE && i > 0 && j < n)
            memset(m + i, 1, (size_t) (j - i));
        i = j - 1;
    }

    for (i = 0; i <= n; i++) {
        if (i < n && m[i]) {
            if (lo < 0)
                lo = i;
        } else if (lo >= 0) {
            int len = i - lo;
            if (len >= min_len && len > best) {
                best = len;
                blo = lo;
                bhi = i - 1;
            }
            lo = -1;
        }
    }

    free(m);
    *out_lo = blo;
    *out_hi = bhi;
}

int ff_find_strip(const unsigned char *gray, int w, int h, ff_strip *out)
{
    unsigned char *rowmed, *colmed, *col;
    int er, y, x, bg, tol, range;
    int y0 = 0, y1 = -1, x0 = 0, x1 = -1;

    if (!gray || !out || w <= 0 || h <= 0)
        return -1;

    rowmed = malloc((size_t) h);
    colmed = malloc((size_t) w);
    col = malloc((size_t) h);
    if (!rowmed || !colmed || !col) {
        free(rowmed);
        free(colmed);
        free(col);
        return -1;
    }

    for (y = 0; y < h; y++)
        rowmed[y] = (unsigned char) ff_median_u8(gray + (size_t) y * w, w);
    for (x = 0; x < w; x++) {
        for (y = 0; y < h; y++)
            col[y] = gray[(size_t) y * w + x];
        colmed[x] = (unsigned char) ff_median_u8(col, h);
    }

    er = EDGE_ROWS;
    if (er > h / 8)
        er = h / 8;
    if (er < 1)
        er = 1;

    bg = (ff_median_u8(rowmed, er) +
          ff_median_u8(rowmed + h - er, er)) / 2;

    range = bg > 255 - bg ? bg : 255 - bg;
    tol = (int) (TOL_FRAC * range);
    if (tol < MIN_TOL)
        tol = MIN_TOL;

    longest_run(rowmed, h, bg + tol, MIN_BAND_H, &y0, &y1);
    if (y1 < y0) {
        free(rowmed);
        free(colmed);
        free(col);
        return -1;
    }

    /* The strip is a solid band vertically, so for columns use the medians of
     * the rows already known to be inside it; that avoids the rebate and the
     * background both ends of the frame from skewing the column profile. */
    longest_run(colmed, w, bg + tol, MIN_BAND_H, &x0, &x1);
    if (x1 < x0) {
        x0 = 0;
        x1 = w - 1;
    }

    out->x0 = x0;
    out->y0 = y0;
    out->x1 = x1;
    out->y1 = y1;
    out->w = x1 - x0 + 1;
    out->h = y1 - y0 + 1;

    free(rowmed);
    free(colmed);
    free(col);
    return 0;
}

int ff_fit_pitch(const unsigned char *gray, int w, int h, int y0, int y1,
                 double *out_pitch)
{
    double *g, *ac;
    int band_h = y1 - y0 + 1;
    int r0, r1, nrows, i, lag, lo, hi, best;
    double bestval, sum = 0.0, frac;

    if (!gray || w < 8 || band_h < MIN_BAND_H)
        return -1;

    /* Picture area: skip the top margin (fiducials) and the bottom rebate. */
    r0 = y0 + (int) (0.23 * band_h);
    r1 = y0 + (int) (0.70 * band_h);
    if (r0 < 0)
        r0 = 0;
    if (r1 >= h)
        r1 = h - 1;
    nrows = r1 - r0 + 1;
    if (nrows < 4 || w < 16)
        return -1;

    g = malloc(sizeof(double) * (size_t) (w - 1));
    ac = malloc(sizeof(double) * (size_t) w);
    if (!g || !ac) {
        free(g);
        free(ac);
        return -1;
    }

    for (i = 0; i < w - 1; i++) {
        double acc = 0.0;
        int y;
        for (y = r0; y <= r1; y++) {
            int a = gray[(size_t) y * w + i];
            int b = gray[(size_t) y * w + i + 1];
            acc += a > b ? a - b : b - a;
        }
        g[i] = acc / nrows;
    }
    for (i = 0; i < w - 1; i++)
        sum += g[i];
    sum /= (w - 1);
    for (i = 0; i < w - 1; i++)
        g[i] -= sum;

    /* Direct autocorrelation: only the lags we care about, so O(w * range). */
    lo = (int) (0.40 * band_h);
    hi = (int) (0.80 * band_h);
    if (lo < FF_PITCH_MIN)
        lo = FF_PITCH_MIN;
    if (hi > w - 2)
        hi = w - 2;
    if (hi > FF_PITCH_MAX)
        hi = FF_PITCH_MAX;
    if (lo >= hi) {
        free(g);
        free(ac);
        return -1;
    }

    best = lo;
    bestval = -1.0;
    for (lag = lo; lag <= hi; lag++) {
        double acc = 0.0;
        for (i = 0; i + lag < w - 1; i++)
            acc += g[i] * g[i + lag];
        ac[lag] = acc;
        if (acc > bestval) {
            bestval = acc;
            best = lag;
        }
    }

    /* Parabolic refinement around the peak. */
    frac = 0.0;
    if (best > lo && best < hi) {
        double ym = ac[best - 1], y0v = ac[best], yp = ac[best + 1];
        double den = ym - 2.0 * y0v + yp;
        if (fabs(den) > 1e-12)
            frac = 0.5 * (ym - yp) / den;
        if (frac > 0.5)
            frac = 0.5;
        if (frac < -0.5)
            frac = -0.5;
    }

    free(g);
    free(ac);

    if (bestval <= 0.0)
        return -1;
    *out_pitch = (double) best + frac;
    return 0;
}

/* Otsu threshold over raster rows [r0,r1]. Returns the intensity that
 * maximises the between-class variance of the histogram. */
static int ff_otsu(const unsigned char *gray, int w, int r0, int r1)
{
    long hist[256];
    double w0 = 0.0, w1, s0 = 0.0, s1, tot = 0.0, sum_all = 0.0;
    double best = -1.0;
    int i, thr = 128;

    if (r1 < r0 || w <= 0)
        return thr;
    memset(hist, 0, sizeof hist);
    for (i = r0; i <= r1; i++) {
        const unsigned char *row = gray + (size_t) i * w;
        int x;
        for (x = 0; x < w; x++)
            hist[row[x]]++;
    }
    for (i = 0; i < 256; i++) {
        tot += (double) hist[i];
        sum_all += (double) i * hist[i];
    }
    if (tot <= 0.0)
        return thr;

    for (i = 0; i < 256; i++) {
        double m0, m1, v;
        w0 += (double) hist[i];
        s0 += (double) i * hist[i];
        w1 = tot - w0;
        if (w1 <= 0.0)
            break;
        s1 = sum_all - s0;
        m0 = s0 / w0;
        m1 = s1 / w1;
        v = w0 * w1 * (m0 - m1) * (m0 - m1);
        if (v > best) {
            best = v;
            thr = i;
        }
    }
    return thr;
}

/* Base-band profile: for every column, the fraction of picture-area rows that
 * lie within FF_BASE_TOL of that column's own median, zeroed unless the median
 * is above the Otsu threshold.
 *
 * Keying on vertical uniformity rather than plain brightness matters. A bright
 * scene can out-shine the film base, which flattens a brightness profile and
 * leaves the frame comb nowhere to sit; but base is base whatever the picture
 * does, so a gap column stays almost perfectly uniform top to bottom while
 * picture columns vary. That separates the gaps on every scan tried. */
static void ff_base_uniformity(const unsigned char *gray, int w, int r0, int r1,
                               int thr, int tol, double *prof)
{
    int nrows = r1 - r0 + 1;
    int x, y, i;

    for (x = 0; x < w; x++) {
        int hist[256];
        int half, acc = 0, med = 0, cnt = 0;

        memset(hist, 0, sizeof hist);
        for (y = r0; y <= r1; y++)
            hist[gray[(size_t) y * w + x]]++;
        half = (nrows + 1) / 2;
        for (i = 0; i < 256; i++) {
            acc += hist[i];
            if (acc >= half) {
                med = i;
                break;
            }
        }
        if (med <= thr) {
            prof[x] = 0.0;
            continue;
        }
        for (i = med - tol; i <= med + tol; i++)
            if (i >= 0 && i < 256)
                cnt += hist[i];
        prof[x] = (double) cnt / (double) nrows;
    }
}

int ff_fit_phase(const unsigned char *gray, int w, int h, int y0, int y1,
                 double pitch, int *out_phase, double *out_score)
{
    double *prof;
    int band_h, r0, r1, nrows, step, p, x, thr;
    int best_phase = 0;
    double best_mean = -1.0;

    if (!gray || w < 16 || y1 <= y0 || pitch < 2.0)
        return -1;

    /* Same picture-area band the pitch autocorrelation uses. */
    band_h = y1 - y0 + 1;
    r0 = y0 + (int) (0.23 * band_h);
    r1 = y0 + (int) (0.70 * band_h);
    if (r0 < 0)
        r0 = 0;
    if (r1 >= h)
        r1 = h - 1;
    nrows = r1 - r0 + 1;
    if (nrows < 4)
        return -1;

    thr = ff_otsu(gray, w, r0, r1);

    prof = malloc(sizeof(double) * (size_t) w);
    if (!prof)
        return -1;
    ff_base_uniformity(gray, w, r0, r1, thr, FF_BASE_TOL, prof);

    step = (int) (pitch + 0.5);
    for (p = 0; p < step; p++) {
        double sum = 0.0, m;
        int n = 0;
        for (x = p; x < w; x += step) {
            sum += prof[x];
            n++;
        }
        /* At least three frame boundaries must fall inside the raster for the
         * comb to mean anything; a real scan of eight frames has eight. */
        if (n < 3)
            continue;
        m = sum / (double) n;
        if (m > best_mean) {
            best_mean = m;
            best_phase = p;
        }
    }

    free(prof);
    if (best_mean < 0.0)
        return -1;
    if (out_phase)
        *out_phase = best_phase;
    if (out_score)
        *out_score = best_mean;
    return 0;
}

double ff_tiled_pitch(const ff_result *out)
{
    double strip_w, pitch;
    int cells;

    if (!out || out->strip.w <= 0 || out->pitch <= 0.0)
        return 0.0;

    /* The frames tile the strip end to end, so the frame count is fixed and
     * the pitch follows from the strip width. The autocorrelation estimate is
     * only good to a couple of percent, which is not enough to decide 7 vs 8
     * frames on its own. Callers that need the pitch actually used by the grid
     * (notably ff_fit_phase) must take it from here, or the frame comb drifts
     * off the gaps by the accumulated error. */
    strip_w = (double) out->strip.w;
    cells = (int) floor(strip_w / out->pitch + 0.5);
    if (cells < 1)
        cells = 1;
    if (cells > FF_MAX_FRAMES)
        cells = FF_MAX_FRAMES;
    pitch = strip_w / cells;
    while (cells > 1 && pitch > FF_PITCH_MAX) {
        cells--;
        pitch = strip_w / cells;
    }
    while (pitch < FF_PITCH_MIN && cells < FF_MAX_FRAMES) {
        cells++;
        pitch = strip_w / cells;
    }
    return pitch;
}

int ff_build_frames(ff_result *out, int raster_w, int raster_h,
                    double pitch_override, double phase, double film_mm,
                    double top_mm, double height_mm, int include_partial)
{
    double pitch, fw, fh, s;
    int i, n = 0, grid_x0;

    if (!out || raster_w <= 0 || raster_h <= 0)
        return -1;
    if (film_mm <= 0.0)
        film_mm = FF_FILM_MM;
    if (out->strip.h <= 0 || out->strip.w <= 0)
        return -1;

    if (pitch_override > 0.0) {
        pitch = pitch_override;
    } else {
        pitch = ff_tiled_pitch(out);
    }
    if (pitch < FF_PITCH_MIN || pitch > FF_PITCH_MAX)
        return -1;

    s = (double) out->strip.h / film_mm;    /* pixels per millimetre */
    fw = pitch;                              /* frames butt together   */
    fh = height_mm * s;
    out->px_per_mm = s;

    if (fh < 1.0 || fh > (double) raster_h)
        return -1;

    out->frames[0].h = (int) (fh + 0.5);
    out->frames[0].w = (int) (fw + 0.5);
    out->frames[0].y0 = out->strip.y0 + (int) (top_mm * s + 0.5);
    grid_x0 = out->strip.x0;

    for (i = 0; ; i++) {
        int x = grid_x0 + (int) (phase + (double) i * pitch + 0.5);
        if (x >= raster_w)
            break;
        if (n >= FF_MAX_FRAMES)
            break;
        if (x < 0) {
            if (!include_partial)
                continue;
            x = 0;
        }
        /* Tolerate a couple of pixels of overhang. The frame edge and the
         * frame width are each truncated to whole pixels, so a grid that
         * should tile the raster exactly can miss by one; without this a 1px
         * phase offset silently costs a whole frame. */
        if (!include_partial &&
            (x + out->frames[0].w > raster_w + 2 ||
             out->frames[0].y0 + out->frames[0].h > raster_h))
            break;
        if (include_partial) {
            int x1 = x + out->frames[0].w;
            int y1 = out->frames[0].y0 + out->frames[0].h;
            if (x1 > raster_w)
                x1 = raster_w;
            if (y1 > raster_h)
                y1 = raster_h;
            if (x1 <= x || y1 <= out->frames[0].y0)
                continue;
            out->frames[n].x0 = x;
            out->frames[n].y0 = out->frames[0].y0;
            out->frames[n].w = x1 - x;
            out->frames[n].h = y1 - out->frames[0].y0;
        } else {
            out->frames[n].x0 = x;
            out->frames[n].y0 = out->frames[0].y0;
            out->frames[n].w = out->frames[0].w;
            out->frames[n].h = out->frames[0].h;
        }
        n++;
    }

    out->pitch = pitch;
    out->nframes = n;
    return n;
}
