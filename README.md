# 8mmScan

Finds the individual frames on scans of a 35mm film strip and writes them out
as numbered PNGs, ready to be muxed into a video.

The frames run along the length of the strip, so a scan shows the whole reel
sideways: one long horizontal band of film, with the frames repeating across it
and clear film base in the gaps between them. `framescan` locates that band,
works out where the film actually starts, and cuts one PNG per frame.

## Building

Needs a C99 compiler, libjpeg and libpng.

    make

## Using it

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

## Options worth knowing

    --rotate DEG        0, 90, 180 or 270 (default 90)
    --positive          keep the scanned tonality instead of inverting it
    --report            print the geometry found for each scan
    --dry-run           detect and report, write nothing
    --include-partial   also emit frames clipped by the image edge
    --no-dedup          keep every frame, including overlaps

`--report` is the first thing to reach for when a scan comes out wrong; it
prints the strip position, the frame pitch and a confidence score per scan.

## How it finds the frames

The pitch is found by autocorrelating the strip along its length, which peaks
once per frame. That tells you how wide the frames are, but not where the first
one starts — and getting that wrong cuts every frame in half with a strip of
film base down the middle.

So the phase is measured separately. Clear film base is not just brighter than
the picture, it is almost perfectly *uniform* from the top of the image area to
the bottom, and that holds however bright the scene is. Keying on uniformity
rather than brightness is what keeps this working on high-key frames where the
picture is as bright as the base and a brightness profile flattens out.

The tool scans every integer phase, scores the uniformity underneath a comb of
frame boundaries, and keeps the best. The mean uniformity under that comb is
reported as `score`: a low score means that scan has no clear base gaps to lock
onto, and its frames are worth eyeballing.

## Tests

    make test

runs the unit checks and an end-to-end pass that builds synthetic strips with
known geometry, cuts them and verifies the crop positions, the rotation, the
inversion and the de-duplication.

## Licence

GPL-3.0. See [LICENSE](LICENSE).
