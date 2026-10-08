/* filmfind.c - film strip and frame grid detection. See filmfind.h. */
#include "filmfind.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define EDGE_ROWS     60    /* rows sampled at each end for background level */
#define MIN_BAND_H   200    /* a strip shorter than this is not a strip       */
#define GAP_CLOSE    160    /* short interior dip bridged unconditionally      */
#define GAP_CLOSE_WEAK 400   /* longer dip bridged if every row is partly film */

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

/* Modal grey level of a byte range: the single most common value in it. */
static int ff_modal_range(const unsigned char *v, size_t n)
{
    long hist[256];
    size_t i;
    int k, best = 0;

    if (n == 0)
        return 0;
    memset(hist, 0, sizeof hist);
    for (i = 0; i < n; i++)
        hist[v[i]]++;
    for (k = 1; k < 256; k++)
        if (hist[k] > hist[best])
            best = k;
    return best;
}

/* Picks the longest run of rows that are not `bg` along more than FF_COVER_MIN
 * of their width, bridging interior dips. */
static void strip_run(const unsigned char *gray, int w, int h, int bg,
                      int *out_lo, int *out_hi)
{
    unsigned char *hit, *weak;
    long need = (long) (FF_COVER_MIN * (double) w);
    long weak_need = (long) (0.40 * (double) w);
    int y, lo = -1, best = 0, blo = 0, bhi = -1, seen = 0;

    *out_lo = 0;
    *out_hi = -1;
    if (h <= 0)
        return;

    hit = malloc((size_t) h);
    weak = malloc((size_t) h);
    if (!hit || !weak) {
        free(hit);
        free(weak);
        return;
    }

    for (y = 0; y < h; y++) {
        const unsigned char *row = gray + (size_t) y * w;
        long c = 0;
        for (int x = 0; x < w; x++) {
            int d = (int) row[x] - bg;
            if (d < 0)
                d = -d;
            if (d > FF_BG_TOL) {
                c++;
                if (c > need)
                    break;
            }
        }
        hit[y] = (unsigned char) (c > need);
        weak[y] = (unsigned char) (c > weak_need);
    }

    /* Bridge dips in coverage inside the strip.
     *
     * A strip is not uniformly covered at 90%: the top rebate, where the
     * perforations sit, only reached 75% coverage on IMAG0086 while the
     * background just above the film reached 64%. Left alone, that rebate
     * splits the band in two and the tool then reports the picture area alone,
     * 1123 rows instead of 1350, which throws the vertical crop off by a couple
     * of hundred rows.
     *
     * Three conditions keep this from over-reaching. The gap has to be bounded
     * by film above and below, so the outer boundary the coverage test found --
     * the actual film edge -- is never moved. A short gap is bridged outright,
     * which is what carries dark content inside a frame that is only a few grey
     * levels off the background and so leaves no coverage signal at all. A
     * longer gap is bridged only if every row in it still reads as partly film,
     * which is what separates a dim rebate from a real gap between two bands. */
    for (y = 1; y < h - 1; y++) {
        int j, k, all_weak = 1;
        if (hit[y]) {
            seen = 1;
            continue;
        }
        j = y;
        while (j < h && !hit[j])
            j++;
        for (k = y; k < j; k++)
            if (!weak[k]) {
                all_weak = 0;
                break;
            }
        if (seen && j < h &&
            ((j - y) <= GAP_CLOSE ||
             (all_weak && (j - y) <= GAP_CLOSE_WEAK)))
            memset(hit + y, 1, (size_t) (j - y));
        y = j - 1;
    }

    for (y = 0; y <= h; y++) {
        if (y < h && hit[y]) {
            if (lo < 0)
                lo = y;
        } else if (lo >= 0) {
            int len = y - lo;
            if (len >= MIN_BAND_H && len > best) {
                best = len;
                blo = lo;
                bhi = y - 1;
            }
            lo = -1;
        }
    }

    free(hit);
    free(weak);
    *out_lo = blo;
    *out_hi = bhi;
}

int ff_find_strip(const unsigned char *gray, int w, int h, ff_strip *out)
{
    unsigned char *col;
    int bg_edge, bg_all, blo = 0, bhi = -1, x0 = 0, x1 = w - 1;
    int er, alo = 0, ahi = -1, lo, hi;

    if (!gray || !out || w <= 0 || h <= 0)
        return -1;

    /* Two candidate background levels, because either one alone is wrong on
     * some scan:
     *
     *   - the mode of the whole raster, which is the flat background on a scan
     *     where the film covers less than half the frame;
     *   - the mode of the rows at the top and bottom edges, which is where the
     *     scanner background always is, and which stays right even when the
     *     film is the largest region in the raster.
     *
     * Running the search against both and keeping the taller band picks the
     * film either way, rather than whichever level happened to be modal. */
    bg_all = ff_modal_range(gray, (size_t) w * (size_t) h);
    er = EDGE_ROWS;
    if (er > h / 8)
        er = h / 8;
    if (er < 1)
        er = 1;
    {
        /* Mode of the two edge blocks, taken together. */
        long hist[256];
        int i, k, best = 0;
        memset(hist, 0, sizeof hist);
        for (i = 0; i < er; i++) {
            int x;
            for (x = 0; x < w; x++) {
                hist[gray[(size_t) i * w + x]]++;
                hist[gray[(size_t) (h - 1 - i) * w + x]]++;
            }
        }
        for (k = 1; k < 256; k++)
            if (hist[k] > hist[best])
                best = k;
        bg_edge = best;
    }

    strip_run(gray, w, h, bg_edge, &alo, &ahi);
    strip_run(gray, w, h, bg_all, &blo, &bhi);
    if (ahi - alo > bhi - blo) {
        blo = alo;
        bhi = ahi;
    }
    if (bhi < blo)
        return -1;

    /* Columns: median of the rows already known to be inside the strip, so the
     * rebate and the background at either end of the frame cannot skew it. */
    col = malloc((size_t) h);
    if (col) {
        unsigned char *colmed = malloc((size_t) w);
        if (colmed) {
            int x, clo = -1, cbest = 0, clo_b = 0, chi_b = -1, bg = bg_edge;
            for (x = 0; x < w; x++) {
                for (lo = blo; lo <= bhi; lo++)
                    col[lo - blo] = gray[(size_t) lo * w + x];
                colmed[x] = (unsigned char) ff_median_u8(col, bhi - blo + 1);
            }
            for (x = 0; x <= w; x++) {
                if (x < w && colmed[x] > bg + FF_BG_TOL) {
                    if (clo < 0)
                        clo = x;
                } else if (clo >= 0) {
                    if (x - clo > cbest) {
                        cbest = x - clo;
                        clo_b = clo;
                        chi_b = x - 1;
                    }
                    clo = -1;
                }
            }
            free(colmed);
            if (chi_b > clo_b && cbest >= MIN_BAND_H) {
                x0 = clo_b;
                x1 = chi_b;
            }
        }
        free(col);
    }

    out->x0 = x0;
    out->y0 = blo;
    out->x1 = x1;
    out->y1 = bhi;
    out->w = x1 - x0 + 1;
    out->h = bhi - blo + 1;
    (void) hi;
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

/* Otsu threshold over raster rows [r0,r1]: the intensity that maximises the
 * between-class variance of the histogram. */
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
 * picture columns vary. */
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

/* Median of n doubles. n is small (a handful of frame-line gaps), so a sort is
 * not worth the allocation. */
static double ff_median_d(const double *v, int n)
{
    double best = 0.0;
    int i, j, bestrank = 0;

    if (n < 1)
        return 0.0;
    bestrank = n / 2;
    for (i = 0; i < n; i++) {
        int rank = 0;
        for (j = 0; j < n; j++)
            if (v[j] < v[i] || (v[j] == v[i] && j < i))
                rank++;
        if (rank == bestrank) {
            best = v[i];
            break;
        }
    }
    return best;
}

/* Locates the frame lines as the leading edges of the wide runs of columns that
 * are clear film base, and fits a lattice to them.
 *
 * ff_fit_phase() samples one column per frame position, which cannot say
 * *where* inside the frame line it has landed. The frame line is 150-200px wide
 * on this roll, so the whole plateau scores almost identically and the reported
 * phase wanders by tens of pixels -- measured edge statistics on a full render
 * showed 86% of frames with picture intruding into the crop edges. Finding the
 * run and taking its leading edge pins the phase down instead of merely placing
 * it somewhere inside the band.
 *
 * `pitch_hint` seeds the search; the fitted pitch is returned in `out_pitch`
 * because individual scans differ from the roll median by a percent or two.
 * `out_score` is the fraction of the expected frame lines that were actually
 * found, so a caller can tell a clean lock from a guess. Returns 0 on success,
 * -1 if too few frame lines were found to fit anything. */
int ff_fit_frame_lines(const unsigned char *gray, int w, int h, int y0, int y1,
                       double pitch_hint, double *out_x0, double *out_pitch,
                       double *out_score)
{
    double *prof;
    int band_h, r0, r1, x, thr, min_w, i, n = 0, cap;
    int *starts;
    double pitch = pitch_hint, phase = 0.0, best_rms = 1e18;
    double score = 0.0;
    int nlines;

    if (!gray || w < 16 || y1 <= y0 || pitch_hint < 2.0)
        return -1;

    band_h = y1 - y0 + 1;
    r0 = y0 + (int) (0.23 * band_h);
    r1 = y0 + (int) (0.70 * band_h);
    if (r0 < 0)
        r0 = 0;
    if (r1 >= h)
        r1 = h - 1;
    if (r1 - r0 + 1 < 4)
        return -1;

    thr = ff_otsu(gray, w, r0, r1);
    prof = malloc(sizeof(double) * (size_t) w);
    cap = w / 2 + 2;
    starts = malloc(sizeof(int) * (size_t) cap);
    if (!prof || !starts) {
        free(prof);
        free(starts);
        return -1;
    }
    ff_base_uniformity(gray, w, r0, r1, thr, FF_BASE_TOL, prof);

    /* ff_base_uniformity() only credits a column whose median is above the
     * Otsu cut, so on a scan whose clear base sits well below the picture's
     * highlights it rejects every frame line. Union it with a plain count of
     * rows at the top of the range, which needs no threshold at all, so a frame
     * line is found whichever of the two views sees it. */
    for (x = 0; x < w; x++) {
        int y, cnt = 0;
        for (y = r0; y <= r1; y++)
            if (gray[(size_t) y * w + x] >= FF_CLEAR_LEVEL)
                cnt++;
        {
            double c = (double) cnt / (double) (r1 - r0 + 1);
            if (c > prof[x])
                prof[x] = c;
        }
    }

    /* Threshold relative to this scan's own best column: the absolute level of
     * clear base varies a lot across the roll, and a fixed cut finds frame lines
     * on some scans and nothing at all on others. */
    {
        double pmax = 0.0, cut;
        for (x = 0; x < w; x++)
            if (prof[x] > pmax)
                pmax = prof[x];
        cut = pmax * 0.85;
        if (cut < 0.55)
            cut = 0.55;
        /* Real frame lines on this roll are 130-320px wide; bright specular
         * highlights inside a picture are well under 100px. Cutting at a tenth
         * of the pitch drops those without touching a frame line. */
        min_w = (int) (0.10 * pitch_hint);
        if (min_w < 8)
            min_w = 8;
        if (getenv("FRAMESCAN_DEBUG"))
            fprintf(stderr, "dbg pmax=%.3f cut=%.3f min_w=%d\n", pmax, cut, min_w);

        for (x = 0; x < w;) {
            int s, e;
            if (prof[x] < cut) {
                x++;
                continue;
            }
            s = x;
            while (x < w && prof[x] >= cut)
                x++;
            e = x - 1;
            /* A run touching a raster edge is cut off, so its leading edge is
             * not where the frame line really starts. The lattice fit recovers
             * the position from the other lines. */
            if (s <= 2 || e >= w - 3)
                continue;
            if (e - s + 1 < min_w)
                continue;
            if (getenv("FRAMESCAN_DEBUG"))
                fprintf(stderr, "dbg run x=%d..%d w=%d\n", s, e, e - s + 1);
            if (n < cap)
                starts[n++] = s;
        }
    }
    free(prof);

    if (n < 3) {
        free(starts);
        return -1;
    }

    /* Seed the pitch from the gaps between neighbouring frame lines, keeping
     * only those already close to the hint so a spurious bright column in the
     * picture cannot drag the pitch. */
    {
        double *gaps = malloc(sizeof(double) * (size_t) n);
        int ng = 0;
        if (!gaps) {
            free(starts);
            return -1;
        }
        for (i = 1; i < n; i++) {
            double d = starts[i] - starts[i - 1];
            if (d > 0.90 * pitch_hint && d < 1.10 * pitch_hint)
                gaps[ng++] = d;
        }
        if (ng > 0)
            pitch = ff_median_d(gaps, ng);
        free(gaps);
    }

    /* Refine by least squares over the lines that agree on the pitch, then
     * take the mean residual as the phase. Two passes: the first fit can be
     * thrown off by one spurious line, the second cannot. */
    for (i = 0; i < 2; i++) {
        double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
        int m = 0;
        double ph = 0.0;
        for (x = 0; x < n; x++) {
            double k = floor((starts[x] - 0.0) / pitch + 0.5);
            ph += starts[x] - k * pitch;
        }
        ph /= n;
        for (x = 0; x < n; x++) {
            double k = floor((starts[x] - ph) / pitch + 0.5);
            double res = starts[x] - (ph + k * pitch);
            if (res < 0.0 ? -res > 30.0 : res > 30.0)
                continue;
            sx += k;
            sy += starts[x];
            sxx += k * k;
            sxy += k * starts[x];
            m++;
        }
        if (m < 3)
            break;
        {
            double den = m * sxx - sx * sx;
            double b, a;
            if (fabs(den) < 1e-9)
                break;
            b = (m * sxy - sx * sy) / den;
            a = (sy - b * sx) / m;
            if (b < 0.5 * pitch_hint || b > 2.0 * pitch_hint)
                break;
            pitch = b;
            {
                double rms = 0.0;
                int c = 0;
                for (x = 0; x < n; x++) {
                    double k = floor((starts[x] - a) / b + 0.5);
                    double res = starts[x] - (a + k * b);
                    if (fabs(res) <= 30.0) {
                        rms += res * res;
                        c++;
                    }
                }
                rms = c ? sqrt(rms / c) : 1e18;
                if (rms < best_rms) {
                    best_rms = rms;
                    phase = a;
                }
            }
        }
    }

    /* Count how many frame lines the fitted lattice actually explains. Reported
     * as a fraction of the lines the raster could hold, so a scan that locked
     * onto three coincidences cannot masquerade as a clean fit. */
    nlines = (phase < w) ? (int) ((w - phase) / pitch) + 1 : 1;
    if (nlines < 1)
        nlines = 1;
    {
        int found = 0;
        for (i = 0; i < n; i++) {
            double k = floor((starts[i] - phase) / pitch + 0.5);
            double res = starts[i] - (phase + k * pitch);
            if (fabs(res) <= 30.0)
                found++;
        }
        score = (double) found / (double) nlines;
        if (score > 1.0)
            score = 1.0;
    }
    free(starts);

    /* A lattice that explains none of the frame lines is not a measurement.
     * Report failure so the caller can fall back rather than crop at a phase
     * invented out of nothing. */
    if (score < 0.5)
        return -1;

    if (out_x0)
        *out_x0 = phase;
    if (out_pitch)
        *out_pitch = pitch;
    if (out_score)
        *out_score = score;
    return 0;
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

void ff_geometry_defaults(ff_geometry *g)
{
    if (!g)
        return;
    g->pitch = 0.0;
    g->px_per_mm = 0.0;
    g->img_top_mm = FF_TOP_MM;
    g->out_w = 0;
    g->out_h = 0;
    g->n_scans = 0;
}

/* Largest even integer <= v. yuv420p chroma subsampling needs both output
 * dimensions even, and the rotated frame width is the one that most often
 * lands odd -- the old per-scan widths ran 927, 936, 941 ... */
static int even_floor(double v)
{
    int i = (int) (v + 0.5);
    if (i < 2)
        i = 2;
    return i & ~1;
}

void ff_geometry_finish(ff_geometry *g, double height_mm)
{
    if (!g)
        return;
    if (g->pitch <= 0.0)
        return;
    if (height_mm <= 0.0)
        height_mm = FF_HEIGHT_MM;
    g->px_per_mm = g->pitch / FF_FRAME_MM;
    /* After the default 90 degree rotation the crop's width becomes the frame's
     * height in the film, so out_w/out_h are the pitch and the image-area
     * height respectively. */
    g->out_w = even_floor(g->pitch);
    g->out_h = even_floor(g->px_per_mm * height_mm);
}

/* Mean of rows [r0,r1] for every column, detrended along x by a centred moving
 * average so that uneven scanner illumination does not dominate. This is the
 * profile the frame period is read off. */
static int rebate_profile(const unsigned char *gray, int w, int r0, int r1,
                          int win, double *prof)
{
    double *sm;
    int nrows = r1 - r0 + 1, x, y;
    double mean = 0.0;

    if (nrows < 2 || w < 2 * win + 2)
        return -1;

    sm = malloc(sizeof(double) * (size_t) w);
    if (!sm)
        return -1;

    for (x = 0; x < w; x++) {
        double acc = 0.0;
        for (y = r0; y <= r1; y++)
            acc += gray[(size_t) y * w + x];
        sm[x] = acc / nrows;
        mean += sm[x];
    }
    mean /= w;

    {
        double *c = malloc(sizeof(double) * (size_t) (w + 2 * win));
        double acc = 0.0;
        int i;
        if (!c) {
            free(sm);
            return -1;
        }
        for (i = 0; i < 2 * win; i++) {
            c[i] = sm[0];
            acc += sm[0];
        }
        for (i = 2 * win; i < w + 2 * win; i++) {
            c[i] = sm[w - 1];
            acc += sm[w - 1];
        }
        for (x = 0; x < w; x++) {
            if (x > 0) {
                acc += sm[x + win];
                acc -= sm[x - win - 1];
            }
            prof[x] = sm[x] - acc / (2.0 * win + 1.0);
            prof[x] += mean;
        }
        free(c);
    }

    free(sm);

    mean = 0.0;
    for (x = 0; x < w; x++)
        mean += prof[x];
    mean /= w;
    for (x = 0; x < w; x++)
        prof[x] -= mean;
    return 0;
}

int ff_fit_rebate_pitch(const unsigned char *gray, int w, int h,
                        const ff_strip *st, double *out_pitch,
                        double *out_conf)
{
    double *prof;
    int r0, r1, lo, hi, lag, best;
    double bestval = 0.0, zero, frac = 0.0;

    if (!gray || !st || st->h < MIN_BAND_H || w < 64)
        return -1;

    /* The outer 3mm at the top of the strip: rebate, and the only band with no
     * picture content in it. */
    r0 = st->y0 + (int) (0.010 * st->h);
    r1 = st->y0 + (int) (0.090 * st->h);
    if (r1 > h - 1)
        r1 = h - 1;
    if (r1 - r0 + 1 < 4)
        return -1;

    prof = malloc(sizeof(double) * (size_t) w);
    if (!prof)
        return -1;
    if (rebate_profile(gray, w, r0, r1, 40, prof) != 0) {
        free(prof);
        return -1;
    }

    zero = 0.0;
    for (lag = 0; lag < w; lag++)
        zero += prof[lag] * prof[lag];
    if (zero <= 1e-9) {
        free(prof);
        return -1;
    }

    /* A 35mm frame is 19.05mm of a 35mm strip, so the pitch is about 0.544 of
     * the strip height. The strip height is itself a fuzzy measurement, so
     * search a wide band around that estimate rather than the whole width: a
     * search the full width of the raster lets picture structure win, and the
     * rebate, being the one band with no content in it, is the only place a
     * trustworthy estimate can come from. */
    {
        double est = (double) st->h * (FF_FRAME_MM / FF_FILM_MM);
        lo = (int) (0.75 * est);
        hi = (int) (1.25 * est);
    }
    if (lo < FF_PITCH_MIN)
        lo = FF_PITCH_MIN;
    if (hi > w - 2)
        hi = w - 2;
    if (hi > FF_PITCH_MAX)
        hi = FF_PITCH_MAX;
    if (lo >= hi) {
        free(prof);
        return -1;
    }

    best = -1;
    for (lag = lo; lag <= hi; lag++) {
        double acc = 0.0;
        int i;
        for (i = 0; i + lag < w; i++)
            acc += prof[i] * prof[i + lag];
        acc /= zero;
        if (best < 0 || acc > bestval) {
            bestval = acc;
            best = lag;
        }
    }

    if (best > lo && best < hi) {
        double a = 0.0, b = 0.0, c = 0.0;
        int i;
        for (i = 0; i + best - 1 < w; i++)
            a += prof[i] * prof[i + best - 1];
        for (i = 0; i + best < w; i++)
            b += prof[i] * prof[i + best];
        for (i = 0; i + best + 1 < w; i++)
            c += prof[i] * prof[i + best + 1];
        a /= zero;
        b /= zero;
        c /= zero;
        {
            double den = a - 2.0 * b + c;
            if (fabs(den) > 1e-12) {
                frac = 0.5 * (a - c) / den;
                if (frac > 0.5)
                    frac = 0.5;
                if (frac < -0.5)
                    frac = -0.5;
            }
        }
    }

    free(prof);

    if (best < 0 || bestval < 0.10)
        return -1;
    if (out_pitch)
        *out_pitch = (double) best + frac;
    if (out_conf)
        *out_conf = bestval;
    return 0;
}

int ff_build_frames(ff_result *out, int raster_w, int raster_h,
                    const ff_geometry *geo, int img_top_px, double phase,
                    double film_mm, int include_partial)
{
    int fw, fh, i, n = 0;

    if (!out || !geo || raster_w <= 0 || raster_h <= 0)
        return -1;
    if (out->strip.h <= 0 || out->strip.w <= 0)
        return -1;

    if (geo->pitch < FF_PITCH_MIN || geo->pitch > FF_PITCH_MAX)
        return -1;
    if (img_top_px < 0)
        return -1;

    /* One size for the whole roll. Taking it from the global geometry rather
     * than from this scan's strip is what makes every PNG the same size. */
    fw = geo->out_w;
    fh = geo->out_h;
    if (fw <= 0 || fh <= 0)
        return -1;
    if (fh > raster_h)
        return -1;

    out->frames[0].w = fw;
    out->frames[0].h = fh;
    out->frames[0].y0 = img_top_px;
    out->px_per_mm = geo->px_per_mm;

    for (i = 0; ; i++) {
        int x = (int) (phase + (double) i * geo->pitch + 0.5);
        if (x >= raster_w)
            break;
        if (n >= FF_MAX_FRAMES)
            break;
        if (x < 0) {
            if (!include_partial)
                continue;
            x = 0;
        }
        if (!include_partial &&
            (x + fw > raster_w + 2 || out->frames[0].y0 + fh > raster_h))
            break;
        if (include_partial) {
            int x1 = x + fw;
            int y1 = out->frames[0].y0 + fh;
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
            out->frames[n].w = fw;
            out->frames[n].h = fh;
        }
        n++;
    }

    out->pitch = geo->pitch;
    out->nframes = n;
    return n;
}
