# 8mmScan

Two film scanning pipelines:

- **framescan** — 35mm film strips (Bell & Howell, 4 perfs/frame)
- **mm8scan** — 8mm film strips (Super 8 / Regular 8, sprocket-hole based)

Both write numbered PNGs ready to mux into video.

## Building

Needs a C99 compiler, libjpeg and libpng.

    make

Produces two binaries: `framescan` and `mm8scan`.

## 35mm pipeline (framescan)

The frames run along the length of the strip, so a scan shows the whole reel
sideways: one long horizontal band of film, with the frames repeating across it
and clear film base in the gaps between them. `framescan` locates that band,
works out where the film actually starts, and cuts one PNG per frame.

### Using it

    ./framescan -o out scans/

PNGs land in `out/render/frame_0000.png`, `frame_0001.png`, … and every crop is
described in `out/manifest.csv`:

    index,source,frame_in_scan,x,y,w,h,status,out_w,out_h

`x,y,w,h` are the crop box in the source scan. `out_w,out_h` are the size of the
PNG actually written, which differs because frames are turned upright. `status`
is `kept`, or `dup_of_N` when the frame repeats an earlier one.

By default the frames are rotated a quarter turn clockwise and inverted, because
that is what a scan of a negative needs: the strip lies on its side, and the
tonality comes out inverted, so frame lines would read as white on black.

To mux the frames into a video:

    make render

which runs `ffmpeg` over the numbered PNGs at 18 fps and writes `out/frames.mp4`.

### Key options

    --rotate DEG        0, 90, 180 or 270 (default 90)
    --positive          keep the scanned tonality instead of inverting it
    --sample N          scans sampled for global geometry (default 24)
    --report            print geometry found for each scan
    --dry-run           detect and report, write nothing
    --include-partial   also emit frames clipped by the image edge
    --no-dedup          keep every frame, including overlaps

`--report` is the first thing to reach for when a scan comes out wrong; it
prints the strip position, the frame pitch and a confidence score per scan.

### How it finds the frames

1. **Strip detection**: coverage test (not brightness) finds the film band
2. **Global pitch**: median of rebate autocorrelation over a sample of scans
   — this is the only pitch measurement used; per-scan estimates are discarded
3. **Per-scan phase**: `ff_fit_frame_lines` finds the actual frame lines
   (150–200px wide bands of clear film base) and fits a lattice
4. **Frame grid**: built from global pitch + per-scan phase, all frames same size
5. **Deduplication**: 32×32 high-pass signature, cosine similarity ≥ 0.97

The global geometry is the key: every scan gets the same `out_w × out_h`, so
ffmpeg never rescale-and-jitter.

## 8mm pipeline (mm8scan)

For 8mm film (Super 8 or Regular 8), the reference is the row of sprocket holes
along the film edge. `mm8scan` detects them, corrects scanner skew, extracts
frames, and stabilizes frame-to-frame using the sprocket holes as fiducials.

### Using it

    ./mm8scan -o out scans/

Output structure matches `framescan`: PNGs in `out/render/`, manifest in
`out/manifest.csv`.

### Pipeline

1. **Threshold + connected components** → sprocket candidates
2. **Filter** by size/aspect/circularity → valid sprockets
3. **Theil-Sen line fit** → strip angle (skew) + pitch (sprocket spacing)
4. **Deskew**: rotate full scan by `-angle` (bilinear)
5. **Frame boxes** from geometry (fixed offsets from sprocket centres)
6. **Frame-to-frame stabilization**:
   - Detect sprockets in each frame crop
   - Match adjacent frames by vertical proximity
   - Solve 6-DOF affine (translation + rotation + scale)
   - Accumulate transforms with optional moving-average smoothing
   - Inverse-warp each frame by its cumulative transform
7. **Deduplication** (reuses 35mm high-pass signature)
8. **Write** PNGs + manifest

### Key options

    --sprocket-thresh N   threshold for sprocket detection (default 220)
    --frame-w N           output frame width (default 400)
    --frame-h N           output frame height (default 300)
    --offset-x N          sprocket-centre → frame left edge (default 50)
    --offset-y N          sprocket-centre → frame top edge (default -150)
    --stabilize-window N  trajectory smoothing window (default 3, 0 = off)
    --max-affine-rms N    reject affine if RMS > N px (default 3.0)

## Tests

    make test

runs the unit checks and an end-to-end pass that builds synthetic strips with
known geometry, cuts them and verifies the crop positions, the rotation, the
inversion and the de-duplication.

## Licence

GPL-3.0. See [LICENSE](LICENSE).