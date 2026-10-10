/* stabilize.c - frame-to-frame affine stabilization using sprocket fiducials.
 *
 * See stabilize.h for the model.  The interesting cases:
 *   - One hole per crop (Super 8): the pairwise transform is a translation,
 *     and the cumulative transform moves every frame's hole back onto frame
 *     0's hole.
 *   - Two or more matched holes (Regular 8): a full least-squares affine is
 *     solved, so rotation and scale are corrected too.
 *
 * Cumulative translations are low-pass filtered (moving average, window
 * DEFAULT_SMOOTH) so that high-frequency wobble is removed while the slow
 * drift of the film through the gate is preserved.
 */

#include "stabilize.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

#define ST_MAX_PTS      16
#define ST_MATCH_DIST2  400.0   /* max squared gap for a correspondence */
#define DEFAULT_SMOOTH  3

/* -------------------------------------------------------------- matching */

/* Detects holes in a crop and keeps those nearest the crop centre, so the
 * frame's own sprocket is tracked rather than a neighbour's. */
static int detect_crop_holes(const unsigned char *gray, int w, int h,
                             double *outx, double *outy)
{
    int xs[ST_MAX_PTS], ys[ST_MAX_PTS];
    int n, i, kept = 0;
    double cx = (w - 1) / 2.0;

    n = sf_frame_sprockets(gray, w, h, NULL, ST_MAX_PTS, xs, ys);
    if (n <= 0)
        return 0;

    /* Keep the holes on the centre column; sort by |x-cx| via selection. */
    for (i = 0; i < n && kept < ST_MAX_PTS; i++) {
        int best = -1, j;
        double bestd = 0.0;
        for (j = 0; j < n; j++) {
            int seen = 0, k;
            double d = fabs(xs[j] - cx);
            for (k = 0; k < kept; k++)
                if (fabs(outx[k] - xs[j]) < 1.0 && fabs(outy[k] - ys[j]) < 1.0)
                    seen = 1;
            if (seen)
                continue;
            if (best < 0 || d < bestd) {
                best = j;
                bestd = d;
            }
        }
        if (best < 0)
            break;
        outx[kept] = xs[best];
        outy[kept] = ys[best];
        kept++;
    }
    return kept;
}

/* Greedy nearest-neighbour correspondence from src to dst.  Writes matched
 * index pairs and returns the count. */
static int match_points(const double *sx, const double *sy, int ns,
                        const double *dx, const double *dy, int nd,
                        int *mi, int *mj)
{
    int i, k = 0;

    for (i = 0; i < ns; i++) {
        int best = -1, j;
        double bestd = ST_MATCH_DIST2;
        for (j = 0; j < nd; j++) {
            double ux = sx[i] - dx[j];
            double uy = sy[i] - dy[j];
            double d = ux * ux + uy * uy;
            if (d < bestd) {
                bestd = d;
                best = j;
            }
        }
        if (best >= 0) {
            mi[k] = i;
            mj[k] = best;
            k++;
        }
    }
    return k;
}

/* -------------------------------------------------------------- linear algebra */

/* Solves the 3x3 system M x = b (M row-major).  Returns 0 on success. */
static int solve3(const double m[9], const double b[3], double x[3])
{
    double a[3][4];
    int i, j, k;

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++)
            a[i][j] = m[i * 3 + j];
        a[i][3] = b[i];
    }

    for (i = 0; i < 3; i++) {
        int piv = i;
        double mx;
        for (j = i + 1; j < 3; j++)
            if (fabs(a[j][i]) > fabs(a[piv][i]))
                piv = j;
        if (fabs(a[piv][i]) < 1e-9)
            return -1;
        if (piv != i)
            for (k = 0; k < 4; k++) {
                double t = a[i][k];
                a[i][k] = a[piv][k];
                a[piv][k] = t;
            }
        mx = a[i][i];
        for (k = i; k < 4; k++)
            a[i][k] /= mx;
        for (j = 0; j < 3; j++) {
            double f;
            if (j == i)
                continue;
            f = a[j][i];
            for (k = i; k < 4; k++)
                a[j][k] -= f * a[i][k];
        }
    }
    for (i = 0; i < 3; i++)
        x[i] = a[i][3];
    return 0;
}

st_affine *st_solve_affine(int n, const double *sx, const double *sy,
                           const double *dx, const double *dy)
{
    st_affine *t = malloc(sizeof *t);
    double m[9], bx[3], by[3], px[3], py[3];
    int i;

    if (!t)
        return NULL;
    t->a = 1.0;
    t->b = 0.0;
    t->c = 0.0;
    t->d = 0.0;
    t->e = 1.0;
    t->f = 0.0;
    if (n <= 0)
        return t;

    /* Translation-only for too few points. */
    if (n < 3) {
        double sxm = 0, sym = 0, dxm = 0, dym = 0;
        for (i = 0; i < n; i++) {
            sxm += sx[i];
            sym += sy[i];
            dxm += dx[i];
            dym += dy[i];
        }
        t->c = dxm / n - sxm / n;
        t->f = dym / n - sym / n;
        return t;
    }

    memset(m, 0, sizeof m);
    memset(bx, 0, sizeof bx);
    memset(by, 0, sizeof by);
    for (i = 0; i < n; i++) {
        double r[3] = { sx[i], sy[i], 1.0 };
        int p, q;
        for (p = 0; p < 3; p++) {
            for (q = 0; q < 3; q++)
                m[p * 3 + q] += r[p] * r[q];
            bx[p] += r[p] * dx[i];
            by[p] += r[p] * dy[i];
        }
    }
    if (solve3(m, bx, px) == 0) {
        t->a = px[0];
        t->b = px[1];
        t->c = px[2];
    }
    if (solve3(m, by, py) == 0) {
        t->d = py[0];
        t->e = py[1];
        t->f = py[2];
    }
    return t;
}

/* -------------------------------------------------------------- sequence */

static st_affine affine_identity(void)
{
    st_affine a = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
    return a;
}

/* out = A * B (apply B then A). */
static st_affine affine_mul(const st_affine *A, const st_affine *B)
{
    st_affine r;
    r.a = A->a * B->a + A->b * B->d;
    r.b = A->a * B->b + A->b * B->e;
    r.c = A->a * B->c + A->b * B->f + A->c;
    r.d = A->d * B->a + A->e * B->d;
    r.e = A->d * B->b + A->e * B->e;
    r.f = A->d * B->c + A->e * B->f + A->f;
    return r;
}

st_sequence *st_stabilize_sequence(const ff_box *frames, int n_frames,
                                   const unsigned char **gray_crops,
                                   int crop_w, int crop_h)
{
    st_sequence *seq;
    double *hx = NULL, *hy = NULL;
    int *hn = NULL;
    int i;

    (void) frames;
    if (n_frames <= 0 || crop_w <= 0 || crop_h <= 0)
        return NULL;

    seq = calloc(1, sizeof *seq);
    if (!seq)
        return NULL;
    seq->n_frames = n_frames;
    seq->pairwise = malloc((size_t) (n_frames > 1 ? n_frames - 1 : 1) *
                           sizeof(st_affine));
    seq->cum = malloc((size_t) n_frames * sizeof(st_affine));
    seq->valid = malloc((size_t) n_frames * sizeof(int));
    hx = malloc((size_t) n_frames * ST_MAX_PTS * sizeof(double));
    hy = malloc((size_t) n_frames * ST_MAX_PTS * sizeof(double));
    hn = malloc((size_t) n_frames * sizeof(int));
    if (!seq->pairwise || !seq->cum || !seq->valid || !hx || !hy || !hn) {
        free(hx);
        free(hy);
        free(hn);
        st_free_sequence(seq);
        return NULL;
    }

    /* Detect the frame's own hole(s) in every crop. */
    for (i = 0; i < n_frames; i++) {
        double *px = hx + (size_t) i * ST_MAX_PTS;
        double *py = hy + (size_t) i * ST_MAX_PTS;
        if (gray_crops && gray_crops[i])
            hn[i] = detect_crop_holes(gray_crops[i], crop_w, crop_h, px, py);
        else
            hn[i] = 0;
    }

    /* Pairwise transforms frame i -> frame i+1. */
    for (i = 0; i + 1 < n_frames; i++) {
        double *sx = hx + (size_t) i * ST_MAX_PTS;
        double *sy = hy + (size_t) i * ST_MAX_PTS;
        double *dx = hx + (size_t) (i + 1) * ST_MAX_PTS;
        double *dy = hy + (size_t) (i + 1) * ST_MAX_PTS;
        int mi[ST_MAX_PTS], mj[ST_MAX_PTS], nm;
        st_affine *t;

        if (hn[i] == 0 || hn[i + 1] == 0) {
            seq->pairwise[i] = affine_identity();
            continue;
        }
        if (hn[i] == 1 && hn[i + 1] == 1) {
            st_affine tr = affine_identity();
            tr.c = dx[0] - sx[0];
            tr.f = dy[0] - sy[0];
            seq->pairwise[i] = tr;
            continue;
        }
        nm = match_points(sx, sy, hn[i], dx, dy, hn[i + 1], mi, mj);
        if (nm <= 0) {
            seq->pairwise[i] = affine_identity();
            continue;
        } else {
            double msx[ST_MAX_PTS], msy[ST_MAX_PTS];
            double mdx[ST_MAX_PTS], mdy[ST_MAX_PTS];
            int k;
            for (k = 0; k < nm; k++) {
                msx[k] = sx[mi[k]];
                msy[k] = sy[mi[k]];
                mdx[k] = dx[mj[k]];
                mdy[k] = dy[mj[k]];
            }
            t = st_solve_affine(nm, msx, msy, mdx, mdy);
            seq->pairwise[i] = t ? *t : affine_identity();
            free(t);
        }
    }

    /* Cumulative transform reference(0) -> frame i, then smooth the path. */
    seq->cum[0] = affine_identity();
    for (i = 1; i < n_frames; i++) {
        seq->cum[i] = affine_mul(&seq->pairwise[i - 1], &seq->cum[i - 1]);
        seq->valid[i] = (hn[i] != 0);
    }
    seq->valid[0] = (hn[0] != 0);

    /* Low-pass the cumulative translation only; rotation/scale are left as
     * measured (they are zero for the one-hole case anyway). */
    if (n_frames >= 3) {
        double *buf = malloc((size_t) n_frames * sizeof(double));
        int w = DEFAULT_SMOOTH;
        int j;
        if (buf) {
            for (j = 0; j < 2; j++) {
                int lo, hi;
                for (i = 0; i < n_frames; i++)
                    buf[i] = j == 0 ? seq->cum[i].c : seq->cum[i].f;
                for (i = 0; i < n_frames; i++) {
                    double sum = 0.0;
                    int cnt = 0, k;
                    lo = i - w / 2;
                    hi = i + w / 2;
                    if (lo < 0)
                        lo = 0;
                    if (hi >= n_frames)
                        hi = n_frames - 1;
                    for (k = lo; k <= hi; k++) {
                        sum += buf[k];
                        cnt++;
                    }
                    if (j == 0)
                        seq->cum[i].c = sum / cnt;
                    else
                        seq->cum[i].f = sum / cnt;
                }
            }
            free(buf);
        }
    }

    free(hx);
    free(hy);
    free(hn);
    return seq;
}

void st_free_sequence(st_sequence *seq)
{
    if (!seq)
        return;
    free(seq->pairwise);
    free(seq->cum);
    free(seq->valid);
    free(seq);
}

int st_apply_cum_transform(const unsigned char *src, int crop_w, int crop_h,
                           int frame_idx, const st_sequence *seq,
                           unsigned char *dst)
{
    unsigned char *warped;
    int ow = 0, oh = 0;
    const st_affine *A;

    if (!src || !dst || !seq || frame_idx < 0 || frame_idx >= seq->n_frames)
        return -1;
    A = &seq->cum[frame_idx];
    warped = sf_apply_affine(src, crop_w, crop_h, A->a, A->b, A->c,
                             A->d, A->e, A->f, &ow, &oh);
    if (!warped)
        return -1;
    memcpy(dst, warped, (size_t) crop_w * crop_h);
    free(warped);
    return 0;
}
