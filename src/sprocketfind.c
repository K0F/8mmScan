/* sprocketfind.c - locate 8mm film sprocket holes and compute strip geometry.
 *
 * Detection is profile based.  Within the film band the perforations are the
 * periodic high-contrast feature, so each horizontal strip near a film edge is
 * reduced to a 1-D contrast profile and its pitch is read from the profile's
 * autocorrelation.  The comb phase then places the individual holes, and the
 * row of maximum contrast under each comb tooth gives the hole's height.  This
 * is polarity- and exposure-agnostic: a hole is whatever differs from its own
 * local neighbourhood, bright or dark.
 *
 * A blob (connected-component) detector is kept for the per-frame crops used
 * by stabilization, where there is no full strip to profile.
 *
 * All functions operate on 8-bit grayscale rasters (w*h bytes, row-major) and
 * contain no I/O.
 */

#include "sprocketfind.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SF_BG_TOL     8      /* grey levels away from the modal background  */
#define SF_COVER_MIN  0.60   /* fraction of a row that must be non-background */
#define SF_PITCH_MIN  550.0
#define SF_PITCH_MAX  950.0
#define SF_STRIP_FRAC 0.38   /* outer fraction of the band searched for holes */

/* -------------------------------------------------------------- thresholds */

static int sf_modal(const unsigned char *gray, int w, int h)
{
    long hist[256];
    int i, best = 0;
    long bestn = -1;

    memset(hist, 0, sizeof hist);
    for (i = 0; i < w * h; i++)
        hist[gray[i]]++;
    for (i = 0; i < 256; i++)
        if (hist[i] > bestn) {
            bestn = hist[i];
            best = i;
        }
    return best;
}

static int sf_otsu_region(const unsigned char *gray, int w, int y0, int y1,
                          int x0, int x1)
{
    long hist[256];
    double tot = 0.0, sum = 0.0, w0 = 0.0, s0 = 0.0, best = -1.0;
    int i, thr = 128, y;

    memset(hist, 0, sizeof hist);
    for (y = y0; y <= y1; y++) {
        const unsigned char *row = gray + (size_t) y * w;
        for (i = x0; i <= x1; i++)
            hist[row[i]]++;
    }
    for (i = 0; i < 256; i++) {
        tot += (double) hist[i];
        sum += (double) i * hist[i];
    }
    if (tot <= 0.0)
        return thr;
    for (i = 0; i < 256; i++) {
        double m0, m1, v;
        w0 += (double) hist[i];
        if (w0 <= 0.0)
            continue;
        s0 += (double) i * hist[i];
        if (tot - w0 <= 0.0)
            break;
        m0 = s0 / w0;
        m1 = (sum - s0) / (tot - w0);
        v = w0 * (tot - w0) * (m0 - m1) * (m0 - m1);
        if (v > best) {
            best = v;
            thr = i;
        }
    }
    return thr;
}

/* -------------------------------------------------------------- band */

static int sf_background(const unsigned char *gray, int w, int h)
{
    int margin = h / 16;
    long hist[256];
    int i, best = 0;
    long bestn = -1;

    if (margin < 4)
        margin = 4;
    memset(hist, 0, sizeof hist);
    for (i = 0; i < margin * w; i++)
        hist[gray[i]]++;
    for (i = 0; i < margin * w; i++)
        hist[gray[(size_t) (h - 1 - i / w) * w + (i % w)]]++;
    for (i = 0; i < 256; i++)
        if (hist[i] > bestn) {
            bestn = hist[i];
            best = i;
        }
    if (bestn < 0)
        return sf_modal(gray, w, h);
    return best;
}

int sf_background_level(const unsigned char *gray, int w, int h)
{
    if (!gray || w <= 0 || h <= 0)
        return 0;
    return sf_background(gray, w, h);
}

int sf_find_band(const unsigned char *gray, int w, int h,
                 int *out_y0, int *out_y1)
{
    int bg, tol = SF_BG_TOL, y;
    int best_start = -1, best_len = 0;
    int need;

    if (!gray || w <= 0 || h <= 0)
        return -1;
    bg = sf_background(gray, w, h);
    need = (int) (SF_COVER_MIN * w);

    {
        int gap = h / 40, film_rows = 0, last_film = -1;
        int best_rows = 0, cur_rows = 0, cur_start = -1;

        if (gap < 8)
            gap = 8;
        for (y = 0; y <= h; y++) {
            int film = 0;
            if (y < h) {
                const unsigned char *row = gray + (size_t) y * w;
                int x, cnt = 0;
                for (x = 0; x < w; x++)
                    if (abs((int) row[x] - bg) > tol)
                        cnt++;
                film = (cnt >= need);
            }
            if (film) {
                if (cur_start < 0) {
                    cur_start = y;
                    cur_rows = 1;
                } else if (y - last_film > gap) {
                    if (cur_rows > best_rows) {
                        best_rows = cur_rows;
                        best_start = cur_start;
                        best_len = last_film - cur_start + 1;
                    }
                    cur_start = y;
                    cur_rows = 1;
                } else {
                    cur_rows++;
                }
                film_rows++;
                last_film = y;
            }
        }
        if (cur_start >= 0 && cur_rows > best_rows) {
            best_start = cur_start;
            best_len = last_film - cur_start + 1;
        }
        (void) film_rows;
    }

    if (best_start < 0 || best_len < h / 20)
        return -1;
    *out_y0 = best_start;
    *out_y1 = best_start + best_len - 1;
    return 0;
}

/* -------------------------------------------------------------- profiles */

/* Separable box blur of the band, radius R, written to `out` (bh*w floats). */
static void band_blur(const unsigned char *gray, int w, int y0, int bh, int R,
                      float *scratch, float *out)
{
    int x, y;
    int span = 2 * R + 1;

    for (y = 0; y < bh; y++) {
        const unsigned char *row = gray + (size_t) (y0 + y) * w;
        float *d = scratch + (size_t) y * w;
        double run = 0.0;
        int lo = 0, hi = -1;
        for (x = 0; x < w; x++) {
            int want_hi = x + R < w - 1 ? x + R : w - 1;
            int want_lo = x - R > 0 ? x - R : 0;
            while (hi < want_hi)
                run += row[++hi];
            while (lo < want_lo)
                run -= row[lo++];
            d[x] = (float) (run / (hi - lo + 1));
        }
    }
    /* Column-major sliding window is brutal on the cache, so process a block
     * of columns at a time: the per-row window bounds are shared, and each
     * row visit loads a run of consecutive floats. */
    {
        enum { BLK = 64 };
        int xb;
        double run[BLK];
        for (xb = 0; xb < w; xb += BLK) {
            int nb = w - xb < BLK ? w - xb : BLK;
            int k, lo = 0, hi = -1;
            for (k = 0; k < nb; k++)
                run[k] = 0.0;
            for (y = 0; y < bh; y++) {
                int want_hi = y + R < bh - 1 ? y + R : bh - 1;
                int want_lo = y - R > 0 ? y - R : 0;
                float *drow = out + (size_t) y * w + xb;
                while (hi < want_hi) {
                    const float *srow = scratch + (size_t) (++hi) * w + xb;
                    for (k = 0; k < nb; k++)
                        run[k] += srow[k];
                }
                while (lo < want_lo) {
                    const float *srow = scratch + (size_t) lo * w + xb;
                    for (k = 0; k < nb; k++)
                        run[k] -= srow[k];
                    lo++;
                }
                {
                    float inv = 1.0f / (float) (hi - lo + 1);
                    for (k = 0; k < nb; k++)
                        drow[k] = (float) run[k] * inv;
                }
            }
        }
    }
    (void) span;
}

/* Autocorrelation peak -> pitch, and comb -> phase, for a 1-D profile.  The
 * returned score is the normalised autocorrelation at the pitch. */
static int pitch_phase(const double *prof, int w, double pmin, double pmax,
                       double *out_pitch, double *out_phase, double *out_score)
{
    double mean = 0.0, *p;
    int i, lag, lo, hi, bestlag = -1;
    double best = -1e30, step, bestc = -1e30, besto = 0.0;

    if (w < 32)
        return -1;
    p = malloc((size_t) w * sizeof(double));
    if (!p)
        return -1;
    for (i = 0; i < w; i++)
        mean += prof[i];
    mean /= w;
    for (i = 0; i < w; i++)
        p[i] = prof[i] - mean;

    lo = (int) pmin;
    hi = (int) pmax;
    if (lo < 2)
        lo = 2;
    if (hi > w - 2)
        hi = w - 2;
    for (lag = lo; lag <= hi; lag++) {
        double s = 0.0;
        int n = 0;
        for (i = 0; i + lag < w; i++) {
            s += p[i] * p[i + lag];
            n++;
        }
        if (n > 0)
            s /= n;
        if (s > best) {
            best = s;
            bestlag = lag;
        }
    }
    free(p);
    if (bestlag < 0)
        return -1;

    step = bestlag;
    /* Comb phase: the offset whose teeth land on the most contrast. */
    {
        int o, st = (int) (step + 0.5);
        if (st < 1)
            st = 1;
        for (o = 0; o < st; o++) {
            double s = 0.0;
            int k = 0;
            for (i = o; i < w; i += st) {
                s += prof[i];
                k++;
            }
            if (k > 0) {
                double a = s / k;
                if (a > bestc) {
                    bestc = a;
                    besto = (double) o;
                }
            }
        }
    }
    /* Parabolic refinement of the pitch on the autocorrelation peak. */
    *out_pitch = step;
    *out_phase = besto;
    *out_score = 0.0;
    {
        double norm = 0.0;
        for (i = 0; i < w; i++)
            norm += prof[i] * prof[i];
        if (norm > 1e-9)
            *out_score = best * (double) w / norm;
    }
    return 0;
}

int sf_find_sprockets(const unsigned char *gray, int w, int h,
                      int band_y0, int band_y1,
                      sf_sprocket *out, int *out_n)
{
    int bh, R, strip, y, x, i;
    float *scratch = NULL, *blur = NULL;
    double *prof = NULL;
    double best_score = -1e30;
    double best_pitch = 0.0, best_phase = 0.0;
    int best_row = -1, best_dir = 0;   /* 0 = top strip, 1 = bottom strip */

    *out_n = 0;
    if (!gray || w <= 0 || h <= 0)
        return -1;
    if (band_y0 < 0)
        band_y0 = 0;
    if (band_y1 >= h)
        band_y1 = h - 1;
    if (band_y1 <= band_y0)
        return -1;
    bh = band_y1 - band_y0 + 1;
    R = bh / 6;
    if (R < 8)
        R = 8;
    if (R > 400)
        R = 400;
    strip = (int) (bh * SF_STRIP_FRAC);
    if (strip < 8)
        strip = 8;

    scratch = malloc((size_t) w * bh * sizeof(float));
    blur = malloc((size_t) w * bh * sizeof(float));
    prof = malloc((size_t) w * sizeof(double));
    if (!scratch || !blur || !prof) {
        free(scratch);
        free(blur);
        free(prof);
        return -1;
    }
    band_blur(gray, w, band_y0, bh, R, scratch, blur);
    free(scratch);

    for (i = 0; i < 2; i++) {
        int r0 = (i == 0) ? 0 : bh - strip;
        int r1 = (i == 0) ? strip : bh;
        int polarity;
        for (polarity = 0; polarity < 2; polarity++) {
            double pitch, phase, score;
            int maxrow = -1;
            double maxrowv = -1e30;

            for (x = 0; x < w; x++)
                prof[x] = 0.0;
            for (y = r0; y < r1; y++) {
                const unsigned char *srow = gray + (size_t) (band_y0 + y) * w;
                const float *brow = blur + (size_t) y * w;
                double rowsum = 0.0;
                for (x = 0; x < w; x++) {
                    double d = (double) srow[x] - brow[x];
                    double v = (polarity == 0) ? -d : d; /* dark=+ or bright=+ */
                    if (v > 0.0)
                        prof[x] += v;
                    rowsum += fabs(d);
                }
                if (rowsum > maxrowv) {
                    maxrowv = rowsum;
                    maxrow = y;
                }
            }

            if (pitch_phase(prof, w, SF_PITCH_MIN, SF_PITCH_MAX,
                            &pitch, &phase, &score) != 0)
                continue;
            if (pitch < SF_PITCH_MIN || pitch > SF_PITCH_MAX)
                continue;
            if (score > best_score) {
                best_score = score;
                best_pitch = pitch;
                best_phase = phase;
                best_row = maxrow;
                best_dir = i;
            }
        }
    }

    if (best_row < 0) {
        free(blur);
        free(prof);
        return 0;
    }

    /* Build the holes: one per comb tooth, height from the local contrast. */
    {
        int step = (int) (best_pitch + 0.5);
        int strip_r0 = (best_dir == 0) ? 0 : bh - strip;
        int strip_r1 = (best_dir == 0) ? strip : bh;
        int wd = step / 8;
        int k;
        if (step < 1)
            step = 1;
        if (wd < 2)
            wd = 2;
        for (k = 0, x = (int) (best_phase + 0.5); x < w; x += step) {
            int yy, by = best_row;
            double bv = -1e30;
            int a0 = x - wd, a1 = x + wd;
            if (a0 < 0)
                a0 = 0;
            if (a1 >= w)
                a1 = w - 1;
            for (yy = strip_r0; yy < strip_r1; yy++) {
                const unsigned char *srow = gray + (size_t) (band_y0 + yy) * w;
                const float *brow = blur + (size_t) yy * w;
                double s = 0.0;
                int xx;
                for (xx = a0; xx <= a1; xx++)
                    s += fabs((double) srow[xx] - brow[xx]);
                if (s > bv) {
                    bv = s;
                    by = yy;
                }
            }
            if (k >= SF_MAX_SPROCKETS || *out_n >= SF_MAX_SPROCKETS)
                break;
            out[*out_n].x = x;
            out[*out_n].y = band_y0 + by;
            out[*out_n].r = step / 4;
            out[*out_n].score = best_score;
            (*out_n)++;
            k++;
        }
    }

    free(blur);
    free(prof);
    return *out_n;
}

/* -------------------------------------------------------------- band model */

/* Mean positive local contrast over rows [r0, r1) for one polarity: dark
 * features (polarity 0) or bright ones (polarity 1). */
static void strip_profile(const unsigned char *gray, int w, int band_y0,
                          const float *blur, int r0, int r1, int polarity,
                          double *prof)
{
    int x, y;

    for (x = 0; x < w; x++)
        prof[x] = 0.0;
    for (y = r0; y < r1; y++) {
        const unsigned char *srow = gray + (size_t) (band_y0 + y) * w;
        const float *brow = blur + (size_t) y * w;
        for (x = 0; x < w; x++) {
            double d = (double) srow[x] - brow[x];
            double v = polarity ? d : -d;
            if (v > 0.0)
                prof[x] += v;
        }
    }
    if (r1 > r0) {
        double inv = 1.0 / (r1 - r0);
        for (x = 0; x < w; x++)
            prof[x] *= inv;
    }
}

int sf_analyze_band(const unsigned char *gray, int w, int h,
                    int band_y0, int band_y1,
                    double pitch_min, double pitch_max, sf_band *out)
{
    int bh, R, strip, i;
    float *scratch = NULL, *blur = NULL;
    double *prof = NULL;
    double pt[2] = { 0.0, 0.0 }, ph[2] = { 0.0, 0.0 }, sc[2] = { -1.0, -1.0 };
    int crow[2] = { 0, 0 };

    if (!gray || w <= 0 || h <= 0 || !out)
        return -1;
    if (band_y0 < 0)
        band_y0 = 0;
    if (band_y1 >= h)
        band_y1 = h - 1;
    if (band_y1 <= band_y0)
        return -1;
    bh = band_y1 - band_y0 + 1;
    R = bh / 6;
    if (R < 8)
        R = 8;
    if (R > 400)
        R = 400;
    strip = (int) (bh * 0.20);
    if (strip < 8)
        strip = 8;

    scratch = malloc((size_t) w * bh * sizeof(float));
    blur = malloc((size_t) w * bh * sizeof(float));
    prof = malloc((size_t) w * sizeof(double));
    if (!scratch || !blur || !prof) {
        free(scratch);
        free(blur);
        free(prof);
        return -1;
    }
    band_blur(gray, w, band_y0, bh, R, scratch, blur);
    free(scratch);

    for (i = 0; i < 2; i++) {
        int c = (i == 0) ? (int) (bh * 0.15) : (int) (bh * 0.85);
        int r0 = c - strip / 2, r1 = c + strip / 2, pol;
        if (r0 < 0)
            r0 = 0;
        if (r1 > bh)
            r1 = bh;
        crow[i] = c;
        for (pol = 0; pol < 2; pol++) {
            double p, phv, s;
            strip_profile(gray, w, band_y0, blur, r0, r1, pol, prof);
            if (pitch_phase(prof, w, pitch_min, pitch_max, &p, &phv, &s) != 0)
                continue;
            if (s > sc[i]) {
                sc[i] = s;
                pt[i] = p;
                ph[i] = phv;
            }
        }
    }
    free(blur);
    free(prof);

    if (pt[0] <= 0.0 && pt[1] <= 0.0)
        return -1;

    memset(out, 0, sizeof *out);
    out->pitch_top = pt[0];
    out->pitch_bot = pt[1];
    out->band_y0 = band_y0;
    out->band_y1 = band_y1;
    out->band_h = bh;

    /* The stronger edge is the perforation row (Super 8 has only one); trust
     * its pitch and phase, not the average or the weaker edge, which often
     * locks onto picture content instead. */
    i = (sc[0] >= sc[1]) ? 0 : 1;
    out->pitch = pt[i];
    out->phase = ph[i];
    out->hole_dir = (i == 0) ? 0 : 1;
    out->score = sc[i];
    out->cy = band_y0 + crow[i];

    /* Tilt from the along-strip phase shift between the two edges. */
    if (pt[0] > 0.0 && pt[1] > 0.0 && sc[0] > 0.0 && sc[1] > 0.0) {
        double dy = (double) (crow[1] - crow[0]);
        double step = out->pitch;
        double d = ph[1] - ph[0];
        if (step > 1.0) {
            d -= step * floor(d / step + 0.5);
            if (dy > 1.0)
                out->angle = atan2(d, dy);
        }
    }
    return 0;
}

/* ---------------------------------------------------------- rectification */

static unsigned char *rectify_ch(const unsigned char *src, int w, int h,
                                 int band_y0, int band_y1, double grad,
                                 int ch, int *out_w, int *out_h)
{
    int bh, x, y, c;
    double cx;
    unsigned char *out;

    if (!src || w <= 0 || h <= 0 || band_y1 < band_y0)
        return NULL;
    if (band_y0 < 0)
        band_y0 = 0;
    if (band_y1 >= h)
        band_y1 = h - 1;
    bh = band_y1 - band_y0 + 1;
    if (bh <= 0)
        return NULL;

    out = malloc((size_t) w * bh * ch);
    if (!out)
        return NULL;
    cx = (w - 1) / 2.0;

    for (y = 0; y < bh; y++) {
        double v = (bh > 1) ? (double) y / (bh - 1) : 0.0;
        double f = 1.0 + grad * (v - 0.5);
        const unsigned char *srow = src + (size_t) (band_y0 + y) * w * ch;
        unsigned char *drow = out + (size_t) y * w * ch;
        if (f < 0.5)
            f = 0.5;
        for (x = 0; x < w; x++) {
            double sx = cx + ((double) x - cx) * f;
            int x0, x1;
            double fx;
            if (sx < 0.0)
                sx = 0.0;
            if (sx > (double) (w - 1))
                sx = (double) (w - 1);
            x0 = (int) sx;
            x1 = x0 + 1 < w ? x0 + 1 : x0;
            fx = sx - x0;
            for (c = 0; c < ch; c++) {
                double a = srow[(size_t) x0 * ch + c];
                double b = srow[(size_t) x1 * ch + c];
                drow[(size_t) x * ch + c] =
                    (unsigned char) (a * (1.0 - fx) + b * fx + 0.5);
            }
        }
    }
    *out_w = w;
    *out_h = bh;
    return out;
}

unsigned char *sf_rectify_gray(const unsigned char *gray, int w, int h,
                               int band_y0, int band_y1, double grad,
                               int *out_w, int *out_h)
{
    return rectify_ch(gray, w, h, band_y0, band_y1, grad, 1, out_w, out_h);
}

unsigned char *sf_rectify_rgb(const unsigned char *rgb, int w, int h,
                              int band_y0, int band_y1, double grad,
                              int *out_w, int *out_h)
{
    return rectify_ch(rgb, w, h, band_y0, band_y1, grad, 3, out_w, out_h);
}

/* Deskew and rectify in one resample: pull just the film band out of the scan
 * with the strip tilt removed and the along-strip pitch made uniform.  Doing
 * it in one pass avoids rotating the whole 22 MPx scan and a second band
 * search, and only touches the band. */
unsigned char *sf_extract_band(const unsigned char *src, int w, int h, int ch,
                               int band_y0, int band_y1, double angle,
                               double grad, int *out_w, int *out_h)
{
    int bh, x, y, c;
    double ca, sa, cx, cyb;
    unsigned char *out;

    if (!src || w <= 0 || h <= 0 || ch < 1 || band_y1 < band_y0)
        return NULL;
    if (band_y0 < 0)
        band_y0 = 0;
    if (band_y1 >= h)
        band_y1 = h - 1;
    bh = band_y1 - band_y0 + 1;
    if (bh <= 0)
        return NULL;

    ca = cos(angle);
    sa = sin(angle);
    cx = (w - 1) / 2.0;
    cyb = 0.5 * (band_y0 + band_y1);
    out = malloc((size_t) w * bh * ch);
    if (!out)
        return NULL;

    for (y = 0; y < bh; y++) {
        double v = (bh > 1) ? (double) y / (bh - 1) : 0.0;
        double f = 1.0 + grad * (v - 0.5);
        double dy = (double) y - (bh - 1) / 2.0;
        unsigned char *drow = out + (size_t) y * w * ch;

        if (f < 0.5)
            f = 0.5;
        for (x = 0; x < w; x++) {
            double dx = ((double) x - cx) * f;
            double sx = cx + ca * dx - sa * dy;
            double sy = cyb + sa * dx + ca * dy;
            int x0, x1, yy0, yy1;
            double fx, fy;

            if (sx < 0.0 || sy < 0.0 ||
                sx > (double) (w - 1) || sy > (double) (h - 1)) {
                for (c = 0; c < ch; c++)
                    drow[(size_t) x * ch + c] = 0;
                continue;
            }
            x0 = (int) sx;
            yy0 = (int) sy;
            x1 = x0 + 1 < w ? x0 + 1 : x0;
            yy1 = yy0 + 1 < h ? yy0 + 1 : yy0;
            fx = sx - x0;
            fy = sy - yy0;
            for (c = 0; c < ch; c++) {
                double a = src[((size_t) yy0 * w + x0) * ch + c];
                double b = src[((size_t) yy0 * w + x1) * ch + c];
                double d = src[((size_t) yy1 * w + x0) * ch + c];
                double e = src[((size_t) yy1 * w + x1) * ch + c];
                double val = (a * (1 - fx) + b * fx) * (1 - fy) +
                             (d * (1 - fx) + e * fx) * fy;
                drow[(size_t) x * ch + c] = (unsigned char) (val + 0.5);
            }
        }
    }
    *out_w = w;
    *out_h = bh;
    return out;
}

/* -------------------------------------------------------------- filter */

static int cmp_sprocket_x(const void *a, const void *b)
{
    const sf_sprocket *p = a, *q = b;
    if (p->x != q->x)
        return p->x < q->x ? -1 : 1;
    return p->y < q->y ? -1 : (p->y > q->y ? 1 : 0);
}

int sf_filter_sprockets(const sf_sprocket *in, int n_in,
                        sf_sprocket *out, int *out_n)
{
    sf_sprocket *tmp;
    int i, keep = 0;

    *out_n = 0;
    if (n_in <= 0)
        return 0;
    tmp = malloc((size_t) n_in * sizeof(sf_sprocket));
    if (!tmp)
        return -1;
    memcpy(tmp, in, (size_t) n_in * sizeof(sf_sprocket));
    qsort(tmp, (size_t) n_in, sizeof(sf_sprocket), cmp_sprocket_x);

    for (i = 0; i < n_in; i++) {
        int dup = 0, j;
        for (j = 0; j < keep; j++) {
            int dx = tmp[i].x - out[j].x;
            int dy = tmp[i].y - out[j].y;
            if (dx * dx + dy * dy < 64) {
                dup = 1;
                break;
            }
        }
        if (!dup)
            out[keep++] = tmp[i];
    }
    free(tmp);
    *out_n = keep;
    return 0;
}

/* -------------------------------------------------------------- geometry */

static double median_double(double *v, int n)
{
    int i, j;
    for (i = 0; i < n - 1; i++)
        for (j = i + 1; j < n; j++)
            if (v[i] > v[j]) {
                double t = v[i];
                v[i] = v[j];
                v[j] = t;
            }
    if (n <= 0)
        return 0.0;
    return (n & 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

int sf_fit_strip_geometry(const sf_sprocket *sprockets, int n,
                          sf_strip_geometry *out)
{
    double *vals, *t;
    double cx = 0.0, cy = 0.0;
    int i, j, nv = 0, np = 0;
    double fine, fcos, fsin;

    if (n < 2) {
        memset(out, 0, sizeof *out);
        return -1;
    }
    for (i = 0; i < n; i++) {
        cx += sprockets[i].x;
        cy += sprockets[i].y;
    }
    cx /= n;
    cy /= n;

    vals = malloc((size_t) n * (n - 1) / 2 * sizeof(double));
    t = malloc((size_t) n * sizeof(double));
    if (!vals || !t) {
        free(vals);
        free(t);
        return -1;
    }
    for (i = 0; i < n; i++)
        for (j = i + 1; j < n; j++) {
            int dx = sprockets[j].x - sprockets[i].x;
            int dy = sprockets[j].y - sprockets[i].y;
            if (dx != 0)
                vals[nv++] = atan2((double) dy, (double) dx);
        }
    fine = nv > 0 ? median_double(vals, nv) : 0.0;
    fcos = cos(fine);
    fsin = sin(fine);

    for (i = 0; i < n; i++)
        t[i] = (sprockets[i].x - cx) * fcos + (sprockets[i].y - cy) * fsin;
    for (i = 0; i < n - 1; i++)
        for (j = i + 1; j < n; j++)
            if (t[j] < t[i]) {
                double tmp = t[i];
                t[i] = t[j];
                t[j] = tmp;
            }
    for (i = 0; i < n - 1; i++)
        vals[i] = t[i + 1] - t[i];
    out->pitch = median_double(vals, n - 1);
    out->angle = fine;
    out->cx = cx;
    out->cy = cy;
    out->n_sprockets = n;
    {
        int leftmost = 0;
        for (i = 1; i < n; i++)
            if (sprockets[i].x < sprockets[leftmost].x)
                leftmost = i;
        out->phase = (double) sprockets[leftmost].x;
    }
    (void) np;
    free(vals);
    free(t);
    return 0;
}

void sf_print_geometry(const sf_strip_geometry *g)
{
    printf("strip: angle=%.4f rad (%.2f deg) pitch=%.1f phase=%.0f "
           "centre=(%.0f,%.0f) n=%d\n",
           g->angle, g->angle * 180.0 / M_PI, g->pitch, g->phase,
           g->cx, g->cy, g->n_sprockets);
}

/* -------------------------------------------------------------- frames */

int sf_build_frames(const sf_strip_geometry *geo, int raster_w, int raster_h,
                    int frame_w, int frame_h, int offset_x, int offset_y,
                    int include_partial, ff_box *out, int *out_n)
{
    int i;

    *out_n = 0;
    if (!geo || !out || raster_w <= 0 || raster_h <= 0)
        return -1;
    if (geo->pitch < 1.0 || frame_w <= 0 || frame_h <= 0)
        return -1;

    for (i = 0; i < SF_MAX_SPROCKETS * 4; i++) {
        double x = geo->phase + (double) i * geo->pitch;
        int x0, y0;

        if (x >= (double) raster_w)
            break;
        x0 = (int) floor(x + offset_x + 0.5);
        y0 = (int) floor(geo->cy + offset_y + 0.5);

        if (!include_partial) {
            if (x0 < 0 || x0 + frame_w > raster_w)
                continue;
            if (y0 < 0 || y0 + frame_h > raster_h)
                continue;
        } else {
            if (x0 + frame_w <= 0 || x0 >= raster_w)
                continue;
            if (y0 + frame_h <= 0 || y0 >= raster_h)
                continue;
        }
        out[*out_n].x0 = x0;
        out[*out_n].y0 = y0;
        out[*out_n].w = frame_w;
        out[*out_n].h = frame_h;
        (*out_n)++;
        if (*out_n >= SF_MAX_SPROCKETS)
            break;
    }
    return *out_n;
}

static double bilinear(const unsigned char *src, int w, int h, double x, double y)
{
    int x0, y0, x1, y1;
    double fx, fy, a, b, c, d;

    if (x < 0.0 || y < 0.0 || x > (double) (w - 1) || y > (double) (h - 1))
        return 0.0;
    x0 = (int) x;
    y0 = (int) y;
    x1 = x0 + 1 < w ? x0 + 1 : x0;
    y1 = y0 + 1 < h ? y0 + 1 : y0;
    fx = x - x0;
    fy = y - y0;
    a = src[(size_t) y0 * w + x0];
    b = src[(size_t) y0 * w + x1];
    c = src[(size_t) y1 * w + x0];
    d = src[(size_t) y1 * w + x1];
    return (a * (1 - fx) + b * fx) * (1 - fy) +
           (c * (1 - fx) + d * fx) * fy;
}

/* -------------------------------------------------------------- frame sprockets */

typedef struct {
    int label;
    int minx, miny, maxx, maxy;
    long area;
    double sx, sy;
} sf_cc;

static int sf_cc_region(const unsigned char *mask, int rw, int rh, int stride,
                        sf_cc **out)
{
    int *comp = NULL, *q = NULL;
    sf_cc *st = NULL;
    int cap = 64, n = 0, x, y;

    *out = NULL;
    comp = calloc((size_t) rw * rh, sizeof(int));
    q = malloc((size_t) rw * rh * sizeof(int));
    st = malloc((size_t) cap * sizeof(sf_cc));
    if (!comp || !q || !st)
        goto fail;

    for (y = 0; y < rh; y++) {
        for (x = 0; x < rw; x++) {
            int idx = y * rw + x, label, qh, qt, minx, miny, maxx, maxy;
            long area;
            double sx, sy;

            if (!mask[(size_t) y * stride + x] || comp[idx])
                continue;
            if (n == cap) {
                sf_cc *ns;
                cap *= 2;
                ns = realloc(st, (size_t) cap * sizeof(sf_cc));
                if (!ns)
                    goto fail;
                st = ns;
            }
            label = n + 1;
            n++;
            qh = qt = 0;
            q[qt++] = idx;
            comp[idx] = label;
            minx = maxx = x;
            miny = maxy = y;
            area = 0;
            sx = sy = 0.0;
            while (qh < qt) {
                int p = q[qh++];
                int py = p / rw, px = p - rw * py;
                int dx, dy;
                area++;
                sx += px;
                sy += py;
                if (px < minx) minx = px;
                if (px > maxx) maxx = px;
                if (py < miny) miny = py;
                if (py > maxy) maxy = py;
                for (dy = -1; dy <= 1; dy++)
                    for (dx = -1; dx <= 1; dx++) {
                        int nx, ny, ni;
                        if (!dx && !dy)
                            continue;
                        nx = px + dx;
                        ny = py + dy;
                        if (nx < 0 || ny < 0 || nx >= rw || ny >= rh)
                            continue;
                        ni = ny * rw + nx;
                        if (!comp[ni] && mask[(size_t) ny * stride + nx]) {
                            comp[ni] = label;
                            q[qt++] = ni;
                        }
                    }
            }
            st[label - 1].label = label;
            st[label - 1].minx = minx;
            st[label - 1].miny = miny;
            st[label - 1].maxx = maxx;
            st[label - 1].maxy = maxy;
            st[label - 1].area = area;
            st[label - 1].sx = sx;
            st[label - 1].sy = sy;
        }
    }
    free(comp);
    free(q);
    *out = st;
    return n;

fail:
    free(comp);
    free(q);
    free(st);
    return -1;
}

int sf_frame_sprockets(const unsigned char *gray, int w, int h,
                       const ff_box *box, int max_pts,
                       int *out_pts_x, int *out_pts_y)
{
    int thr, polarity, best = 0;

    if (!gray || w <= 0 || h <= 0 || max_pts <= 0)
        return 0;
    thr = sf_otsu_region(gray, w, 0, h - 1, 0, w - 1);

    for (polarity = 0; polarity < 2 && best == 0; polarity++) {
        unsigned char *mask = malloc((size_t) w * h);
        sf_cc *st = NULL;
        int n, i, got = 0;

        if (!mask)
            break;
        for (i = 0; i < w * h; i++)
            mask[i] = polarity ? (gray[i] > thr) : (gray[i] < thr);
        n = sf_cc_region(mask, w, h, w, &st);
        free(mask);
        if (n <= 0) {
            free(st);
            continue;
        }
        for (i = 0; i < n && got < max_pts; i++) {
            int cw = st[i].maxx - st[i].minx + 1;
            int ch = st[i].maxy - st[i].miny + 1;
            double aspect = (double) cw / (double) ch;
            double fill = (double) st[i].area / ((double) cw * ch);
            double cx = st[i].sx / (double) st[i].area;
            double cy = st[i].sy / (double) st[i].area;
            double area = (double) st[i].area;
            if (area < 6.0 || area > 0.02 * w * h)
                continue;
            if (aspect < 0.3 || aspect > 3.0)
                continue;
            if (fill < 0.35)
                continue;
            if (box && (cx < box->x0 - 2 || cx > box->x0 + box->w + 2 ||
                        cy < box->y0 - 2 || cy > box->y0 + box->h + 2))
                continue;
            if (out_pts_x && out_pts_y) {
                out_pts_x[got] = (int) (cx + 0.5);
                out_pts_y[got] = (int) (cy + 0.5);
            }
            got++;
        }
        free(st);
        if (got > 0)
            best = got;
    }
    return best;
}

/* -------------------------------------------------------------- affine */

unsigned char *sf_apply_affine(const unsigned char *src, int w, int h,
                               double a, double b, double c,
                               double d, double e, double f,
                               int *out_w, int *out_h)
{
    unsigned char *dst;
    int x, y;

    *out_w = w;
    *out_h = h;
    dst = calloc((size_t) w * h, sizeof(unsigned char));
    if (!dst)
        return NULL;
    for (y = 0; y < h; y++)
        for (x = 0; x < w; x++) {
            double sx = a * x + b * y + c;
            double sy = d * x + e * y + f;
            dst[(size_t) y * w + x] =
                (unsigned char) (bilinear(src, w, h, sx, sy) + 0.5);
        }
    return dst;
}

unsigned char *sf_apply_affine_rgb(const unsigned char *src, int w, int h,
                                   double a, double b, double c,
                                   double d, double e, double f,
                                   int *out_w, int *out_h)
{
    unsigned char *dst;
    int x, y, ch;

    *out_w = w;
    *out_h = h;
    dst = calloc((size_t) w * h * 3, sizeof(unsigned char));
    if (!dst)
        return NULL;
    for (y = 0; y < h; y++)
        for (x = 0; x < w; x++) {
            double sx = a * x + b * y + c;
            double sy = d * x + e * y + f;
            int x0, y0, x1, y1;
            double fx, fy;
            if (sx < 0.0 || sy < 0.0 ||
                sx > (double) (w - 1) || sy > (double) (h - 1))
                continue;
            x0 = (int) sx;
            y0 = (int) sy;
            x1 = x0 + 1 < w ? x0 + 1 : x0;
            y1 = y0 + 1 < h ? y0 + 1 : y0;
            fx = sx - x0;
            fy = sy - y0;
            for (ch = 0; ch < 3; ch++) {
                double p00 = src[((size_t) y0 * w + x0) * 3 + ch];
                double p01 = src[((size_t) y0 * w + x1) * 3 + ch];
                double p10 = src[((size_t) y1 * w + x0) * 3 + ch];
                double p11 = src[((size_t) y1 * w + x1) * 3 + ch];
                double v = (p00 * (1 - fx) + p01 * fx) * (1 - fy) +
                           (p10 * (1 - fx) + p11 * fx) * fy;
                dst[((size_t) y * w + x) * 3 + ch] =
                    (unsigned char) (v + 0.5);
            }
        }
    return dst;
}

/* -------------------------------------------------------------- end */
