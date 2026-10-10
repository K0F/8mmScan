# 8mmScan

Single-pipeline film scanner:

- **framescan** — 8mm film strips (Super 8). Produces frame PNGs with global,
  position-dependent geometry correction calibrated for the **RODFS50** flatbed
  scanner, and writes numbered PNGs ready to mux into video.

## Building

Needs a C99 compiler, libjpeg and libpng.

    make

Produces a single binary: `framescan`.

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
    --no-stabilize      disable frame-to-frame affine stabilization
    --fps N             not used (reserved)
    --pattern           include all images in directory recursively

`--report` prints the strip position, pitch/angle and frames per scan.

### How it finds the frames (8mm, RODFS50)

1. **Strip detection**: profile/coverage based; band position is determined
   per scan (not fixed) because it drifts between scans.
2. **Skew & geometry**: strip tilt is estimated from band edge slopes; a small
   per-scan rotation corrects skew, then the film band is extracted.
3. **Position-dependent geometry correction**: the sprocket row and the
   opposite edge run at slightly different effective pitches along the scanner
   (keystone/barrel in the optical path). Rather than warping each frame, the
   correction is applied **globally per scanner** as a linear factor `grad`
   across the band (row-wise scaling about the horizontal centre). For
   `RODFS50` the global model (`pitch`, `grad`, `band_h`, `frame_w/h`, `offset_x/y`)
   is measured from a sample of scans and reused for all; only the band
   position, strip angle and lattice phase vary per scan.
4. **Lattice**: Super 8 has one perforation per frame; the position-dependent
   rectified image is analyzed to recover pitch and phase on the rectified
   band, and frames are cut on a fixed grid (`pitch + offset_x`, centred
   vertically by `offset_y`).
5. **Stabilization**: frame-to-frame affine stabilization using sprocket
   fiducials (translation-only if only one perf is tracked; full affine
   otherwise). Optional smoothing.
6. **Deduplication**: 32×32 high-pass signature, cosine similarity ≥ 0.97.

### Calibration

This build uses geometry calibrated for scanner **RODFS50** (found in EXIF of
the provided scans). If you use a different scanner, the global `grad` and the
default framing may need re-calibration.

## Tests

    make test

runs the unit checks and an end-to-end pass that builds synthetic strips with
known geometry, cuts them and verifies the crop positions, the rotation, the
inversion and the de-duplication.

## Licence

GPL-3.0. See [LICENSE](LICENSE).