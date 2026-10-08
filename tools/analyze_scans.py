#!/usr/bin/env python3
"""Measure the geometry of 35mm film scans: strip position, perforation lattice,
frame pitch and phase.

The film is 35mm Bell & Howell: perforations on a 4.7625mm pitch with four per
frame, so the frame pitch is 19.05mm. The perforations sit in the rebate, outside
the image area, at a fixed offset from the frame lines -- which makes them the one
reference that does not depend on the picture content.

Run:  tools/analyze_scans.py scans/IMAG0086.JPG ...
"""
import sys
import numpy as np
from PIL import Image

PERF_MM = 4.7625
PERFS_PER_FRAME = 4
FILM_MM = 35.0
TOP_MM = 7.1          # strip top edge -> image area top
HEIGHT_MM = 23.7      # image area height
COVER = 0.90          # fraction of columns that must be non-background


def modal_bg(a):
    v, c = np.unique(a, return_counts=True)
    return float(v[np.argmax(c)])


def strip_rows(a, cover=COVER, tol=8):
    """Rows where most of the width is not the flat scanner background.

    Keying on coverage rather than a brightness threshold matters: the background
    around the film is lit unevenly, so a fixed threshold either swallows a few
    hundred rows of background (h too large) or cuts into the film (h too small).
    Coverage is immune to both, and to specks of dust elsewhere in the frame.
    """
    bg = modal_bg(a)
    m = np.abs(a - bg) > tol
    rows = m.sum(axis=1)
    hit = np.where(rows > cover * a.shape[1])[0]
    if len(hit) == 0:
        return None
    return int(hit.min()), int(hit.max()), bg


def boxblur(P, r):
    P = P.astype(np.float32)
    c = np.cumsum(np.pad(P, ((r+1, r), (0, 0)), mode='edge'), axis=0)
    P = (c[2*r+1:, :] - c[:-2*r-1, :]) / (2*r+1)
    c = np.cumsum(np.pad(P, ((0, 0), (r+1, r)), mode='edge'), axis=1)
    return (c[:, 2*r+1:] - c[:, :-2*r-1]) / (2*r+1)


def comb_score(prof, pitch, phase, nharm=6):
    """Sum of profile values sampled every `pitch` px, i.e. a comb on the perfs."""
    step = int(round(pitch))
    xs = np.arange(phase % step, len(prof), step)
    if len(xs) < 8:
        return None
    s = prof[xs]
    return float(s.mean()), float(s.std())


def analyse(path, verbose=True):
    a = np.asarray(Image.open(path).convert('L')).astype(np.float32)
    H, W = a.shape
    st = strip_rows(a)
    if st is None:
        return None
    y0, y1, bg = st
    Hh = y1 - y0 + 1
    ppg0 = Hh / FILM_MM

    # Perfs are bright or dark depending on how the scan was exposed, so compare
    # each column against its own local mean instead of against an absolute level.
    S = a[y0:y1+1]
    loc = boxblur(S, 40)
    d = S - loc

    # Rebate bands: outer 3mm at each film edge, which is where perfs live.
    def band(lo_mm, hi_mm):
        return d[int(lo_mm*ppg0):max(int(hi_mm*ppg0), int(lo_mm*ppg0)+1)].mean(axis=0)

    bands = [band(0.3, 3.0), band(32.0, 34.7)]
    profs = [b - b.mean() for b in bands]
    acs = []
    for p in profs:
        v = np.correlate(p, p, 'full')[len(p)-1:]
        acs.append(v / (v[0] + 1e-9))

    # Sweep px/mm; score = mean autocorrelation at the first six perf harmonics.
    # Requiring every harmonic to line up at once is what makes the fit immune to
    # the broad illumination gradient, which only produces a single wide peak.
    best = []
    for ppmm in np.arange(30.0, 46.0, 0.01):
        tot, n, ok = 0.0, 0, True
        for A in acs:
            for k in range(1, 7):
                lag = int(round(k * PERF_MM * ppmm))
                if lag >= len(A):
                    ok = False
                    break
                tot += A[lag]
                n += 1
            if not ok:
                break
        if ok and n:
            best.append((tot/n, ppmm))
    best.sort(reverse=True)

    out = {'path': path, 'bg': bg, 'y0': y0, 'y1': y1, 'strip_h': Hh,
           'ppg0': ppg0, 'top': best[:5]}
    if best:
        sc, ppmm = best[0]
        out['ppmm'] = ppmm
        out['score'] = sc
        out['perf_pitch'] = PERF_MM * ppmm
        out['frame_pitch'] = PERFS_PER_FRAME * PERF_MM * ppmm
    if verbose:
        print('%s bg=%3.0f strip %4d..%-4d h=%4d  h/35=%5.2f' %
              (path[-12:-4], bg, y0, y1, Hh, ppg0))
        if best:
            print('   ppmm %s   best %.3f  perf %.1f  frame %.1f px' %
                  (' '.join('%.2f' % b[1] for b in best[:4]), out['score'],
                   out['perf_pitch'], out['frame_pitch']))
    return out


def main(argv):
    paths = argv[1:]
    if not paths:
        print(__doc__)
        return 2
    res = [analyse(p) for p in paths]
    res = [r for r in res if r and 'ppmm' in r]
    if res:
        v = np.array([r['ppmm'] for r in res])
        h = np.array([r['strip_h'] for r in res], float)
        print('\n%d scans: ppmm median %.2f sd %.3f | strip_h median %.0f sd %.1f cv %.4f'
              % (len(v), np.median(v), v.std(), np.median(h), h.std(), h.std()/h.mean()))
        print('   implied frame pitch %.1f px, film height %.0f px'
              % (PERFS_PER_FRAME*PERF_MM*np.median(v), FILM_MM*np.median(v)))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
