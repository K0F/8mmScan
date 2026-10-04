#!/bin/sh
# e2e.sh - end-to-end test: build a synthetic scan with known geometry, run
# framescan on it, and check the crops, the numbering and the de-duplication.
set -eu

BIN=${BIN:-./framescan}
MAKE_FIXTURE=${MAKE_FIXTURE:-./build/make_fixture}
PNG_PIXEL=${PNG_PIXEL:-./build/png_pixel}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

fail() { echo "    FAIL $1" >&2; exit 1; }

# Geometry: 3 frames of 400px pitch across 1200px, strip y=150..919 (h=770).
# BH/PITCH = 1.925 matches the real scans (~1375/715), so the pitch search
# window brackets the true pitch exactly as it does on real film.
W=1200; H=1100; Y0=150; BH=770; PITCH=400; NF=3
"$MAKE_FIXTURE" "$TMP/a.jpg" "$W" "$H" "$Y0" "$BH" "$PITCH" "$NF"
# A second, identical scan: every frame must be recognised as a duplicate.
cp "$TMP/a.jpg" "$TMP/b.jpg"

echo "  e2e: single scan"
rm -rf "$TMP/out"
"$BIN" --outdir "$TMP/out" "$TMP/a.jpg" >/dev/null 2>&1 || fail "framescan exited non-zero"

EXPECT_W=$PITCH
EXPECT_H=$(awk -v bh="$BH" 'BEGIN{printf "%d", 23.7*bh/35.0+0.5}')
EXPECT_Y=$(awk -v y0="$Y0" -v bh="$BH" 'BEGIN{printf "%d", y0+7.1*bh/35.0+0.5}')

N=$(ls "$TMP/out"/render/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "$NF" ] || fail "expected $NF frames, got $N"
[ -f "$TMP/out/manifest.csv" ] || fail "manifest.csv missing"
# PNGs go to <outdir>/render, the manifest stays in <outdir>.
[ "$(ls "$TMP/out"/frame_*.png 2>/dev/null | wc -l | tr -d ' ')" = "0" ] \
  || fail "PNGs must not be written directly into the outdir"

# Crop geometry, via the manifest.
for i in 0 1 2; do
  ROW=$(awk -F, -v n="$i" 'NR>1 && $1==n {print $4","$5","$6","$7","$8}' "$TMP/out/manifest.csv")
  EXP_X=$((i * PITCH))
  [ "$ROW" = "$EXP_X,$EXPECT_Y,$EXPECT_W,$EXPECT_H,kept" ] \
    || fail "frame $i geometry: got '$ROW' want '$EXP_X,$EXPECT_Y,$EXPECT_W,$EXPECT_H,kept'"
done
echo "    ok geometry + numbering"

echo "  e2e: overlap de-duplication"
rm -rf "$TMP/out2"
"$BIN" --outdir "$TMP/out2" "$TMP/a.jpg" "$TMP/b.jpg" >/dev/null 2>&1 \
  || fail "framescan exited non-zero on duplicate scans"
N=$(ls "$TMP/out2"/render/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "$NF" ] || fail "duplicate scan not removed: expected $NF frames, got $N"
DUPS=$(grep -c ',dup_of_' "$TMP/out2/manifest.csv" || true)
[ "$DUPS" = "$NF" ] || fail "expected $NF dup rows, got $DUPS"
echo "    ok duplicates dropped"

echo "  e2e: --no-dedup keeps everything"
rm -rf "$TMP/out3"
"$BIN" --outdir "$TMP/out3" --no-dedup "$TMP/a.jpg" "$TMP/b.jpg" >/dev/null 2>&1
N=$(ls "$TMP/out3"/render/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "$((NF * 2))" ] || fail "--no-dedup: expected $((NF*2)) frames, got $N"
echo "    ok"

echo "  e2e: partial frames excluded by default"
# An explicit --pitch disables strip snapping, so a 1150px raster really does
# clip the third 400px frame at the right edge.
"$MAKE_FIXTURE" "$TMP/c.jpg" 1150 "$H" "$Y0" "$BH" "$PITCH" "$NF"
rm -rf "$TMP/out4"
"$BIN" --outdir "$TMP/out4" --pitch "$PITCH" "$TMP/c.jpg" >/dev/null 2>&1
N=$(ls "$TMP/out4"/render/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "2" ] || fail "expected 2 whole frames in 1150px, got $N"
W4=$(awk -F, 'NR>1 && $1==1 {print $6}' "$TMP/out4/manifest.csv")
[ "$W4" = "$PITCH" ] || fail "whole frame width wrong: got '$W4'"
rm -rf "$TMP/out5"
"$BIN" --outdir "$TMP/out5" --pitch "$PITCH" --include-partial "$TMP/c.jpg" \
  >/dev/null 2>&1
N=$(ls "$TMP/out5"/render/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "3" ] || fail "--include-partial: expected 3 frames, got $N"
W5=$(awk -F, 'NR>1 && $1==2 {print $6}' "$TMP/out5/manifest.csv")
[ "$W5" = "$((1150 - 2 * PITCH))" ] \
  || fail "clamped trailing frame should be $((1150 - 2 * PITCH))px wide, got '$W5'"
echo "    ok"

echo "  e2e: strip snapping absorbs a width mismatch"
# Film of 393px pitch in a 1180px raster: 1180 is not a multiple of 393, so the
# grid has to resnap to 1180/3 = 393.33px to tile the strip without cutting the
# last frame short. The film's own gaps sit at 0/393/786/1179.
"$MAKE_FIXTURE" "$TMP/g.jpg" 1180 "$H" "$Y0" "$BH" 393 "$NF"
rm -rf "$TMP/out8"
"$BIN" --outdir "$TMP/out8" "$TMP/g.jpg" >/dev/null 2>&1
N=$(ls "$TMP/out8"/render/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "3" ] || fail "snapped grid: expected 3 frames, got $N"
LAST=$(awk -F, 'NR>1 && $1==2 {print $4+$6}' "$TMP/out8/manifest.csv")
# Within the 2px rounding tolerance: the frame edge and width are truncated, so
# a grid that tiles 1180 exactly can land on 1179..1181.
[ "$LAST" -ge 1179 ] && [ "$LAST" -le 1181 ] \
  || fail "snapped frames should tile the strip, last ends at $LAST"
echo "    ok"

echo "  e2e: rejects a raster with no film strip"
"$MAKE_FIXTURE" "$TMP/d.jpg" 400 400 0 40 200 1
rm -rf "$TMP/out6"
"$BIN" --outdir "$TMP/out6" "$TMP/d.jpg" >/dev/null 2>&1 \
  || fail "framescan should exit 0 when no strip is found"
N=$(ls "$TMP/out6"/render/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "0" ] || fail "expected 0 frames from a strip-less raster, got $N"
KEEP=$(grep -c ',kept' "$TMP/out6/manifest.csv" 2>/dev/null || true)
[ "$KEEP" = "0" ] || fail "manifest should be empty, got $KEEP kept rows"
echo "    ok"

echo "  e2e: fails loudly on a missing file"
rm -rf "$TMP/out7"
"$BIN" --outdir "$TMP/out7" "$TMP/nope.jpg" >/dev/null 2>&1 \
  && fail "expected non-zero exit for a missing input"
echo "    ok"

echo "  e2e: --render-dir overrides the PNG subdirectory"
rm -rf "$TMP/out9" "$TMP/pngs"
"$BIN" --outdir "$TMP/out9" --render-dir "$TMP/pngs" "$TMP/a.jpg" >/dev/null 2>&1
N=$(ls "$TMP/pngs"/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "$NF" ] || fail "--render-dir: expected $NF frames, got $N"
[ -f "$TMP/out9/manifest.csv" ] || fail "manifest must stay in the outdir"
echo "    ok"

echo "  e2e: distinct scans are never de-duplicated"
# Same geometry but a different seed offset, so the pictures differ. Every
# frame must survive: with a shared geometry the only correct answer is 2*NF.
"$MAKE_FIXTURE" "$TMP/d1.jpg" "$W" "$H" "$Y0" "$BH" "$PITCH" "$NF" 0
"$MAKE_FIXTURE" "$TMP/d2.jpg" "$W" "$H" "$Y0" "$BH" "$PITCH" "$NF" 104729
rm -rf "$TMP/out10"
"$BIN" --outdir "$TMP/out10" "$TMP/d1.jpg" "$TMP/d2.jpg" >/dev/null 2>&1
N=$(ls "$TMP/out10"/render/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "$((NF * 2))" ] || fail "distinct scans: expected $((NF*2)) frames, got $N"
DUPS=$(grep -c ',dup_of_' "$TMP/out10/manifest.csv" || true)
[ "$DUPS" = "0" ] || fail "distinct scans must not dedup, got $DUPS dup rows"
echo "    ok"

echo "  e2e: a blank frame neither poisons nor is dropped"
# Regression: a frame with no high-pass texture has no usable signature. It
# used to be stored unnormalised, which pushed cosine similarities above 1.0
# and made later frames match it, silently deleting real frames.
"$MAKE_FIXTURE" "$TMP/f1.jpg" "$W" "$H" "$Y0" "$BH" "$PITCH" "$NF" 0 1
"$MAKE_FIXTURE" "$TMP/f2.jpg" "$W" "$H" "$Y0" "$BH" "$PITCH" "$NF" 7717 1
rm -rf "$TMP/out11"
"$BIN" --outdir "$TMP/out11" "$TMP/f1.jpg" "$TMP/f2.jpg" >/dev/null 2>&1
N=$(ls "$TMP/out11"/render/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "$((NF * 2))" ] \
  || fail "blank frames must still be kept: expected $((NF*2)) frames, got $N"
DUPS=$(grep -c ',dup_of_' "$TMP/out11/manifest.csv" || true)
[ "$DUPS" = "0" ] || fail "blank frame poisoned dedup: $DUPS dup rows"
# The same scan twice is still a perfect duplicate, blank frame included.
rm -rf "$TMP/out12"
"$BIN" --outdir "$TMP/out12" "$TMP/f1.jpg" "$TMP/f1.jpg" >/dev/null 2>&1
N=$(ls "$TMP/out12"/render/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "$NF" ] || fail "blank scan copy: expected $NF frames, got $N"
DUPS=$(grep -c ',dup_of_' "$TMP/out12/manifest.csv" || true)
[ "$DUPS" = "$NF" ] || fail "blank frame broke dedup: expected $NF dups, got $DUPS"
echo "    ok"

echo "  e2e: frame phase follows the film, not the strip edge"
# 400px pitch tiling 1200px exactly (3 cells), but the film runs start 137px in,
# behind a leader of clear base. The strip edge is x=0, the first real frame
# boundary is x=137, so this is the case where assuming phase 0 cuts frames.
rm -rf "$TMP/outp" "$TMP/outnp"
LEADER=137
"$MAKE_FIXTURE" "$TMP/lead.jpg" "$W" "$H" "$Y0" "$BH" "$PITCH" 3 0 -1 "$LEADER"
"$BIN" --outdir "$TMP/outp" "$TMP/lead.jpg" >/dev/null 2>&1 \
  || fail "framescan exited non-zero on leader fixture"
# The leader pushes the last cell off the end, so 3 cells hold only 2 frames.
N=$(ls "$TMP/outp"/render/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "2" ] || fail "leader: expected 2 frames, got $N"
for i in 0 1; do
  ROW=$(awk -F, -v n="$i" 'NR>1 && $1==n {print $4","$6","$8}' "$TMP/outp/manifest.csv")
  EXP_X=$((LEADER + i * PITCH))
  [ "$ROW" = "$EXP_X,$EXPECT_W,kept" ] \
    || fail "leader frame $i: got '$ROW' want x=$EXP_X w=$EXPECT_W"
done
# --no-phase must reproduce the old, wrong behaviour: grid starting at the strip edge.
"$BIN" --outdir "$TMP/outnp" --no-phase "$TMP/lead.jpg" >/dev/null 2>&1 \
  || fail "framescan exited non-zero with --no-phase"
ROW=$(awk -F, '$1==0 {print $4","$6}' "$TMP/outnp/manifest.csv")
[ "$ROW" = "0,$EXPECT_W" ] \
  || fail "--no-phase should start at the strip edge: got '$ROW' want 0,$EXPECT_W"
echo "    ok phase aligned to film"

echo "  e2e: frames are rotated clockwise and inverted by default"
# The tool's defaults exist because a scan of a negative reads backwards: the
# frames must turn 90 deg clockwise to stand upright, and the tonality has to
# flip so the frame lines are black on white. Compare the default output
# against an explicit unrotated, untouched render of the same scan.
rm -rf "$TMP/rot_def" "$TMP/rot_flat"
"$BIN" --outdir "$TMP/rot_def" "$TMP/lead.jpg" >/dev/null 2>&1 \
  || fail "framescan exited non-zero on the default transform"
"$BIN" --outdir "$TMP/rot_flat" --rotate 0 --positive "$TMP/lead.jpg" >/dev/null 2>&1 \
  || fail "framescan exited non-zero on --rotate 0 --positive"

# A quarter turn swaps width and height. Check it in the manifest, which records
# the source box and the emitted image separately.
SRCDIMS=$(awk -F, 'NR==2 {print $6","$7}' "$TMP/rot_flat/manifest.csv")
OUTDIMS=$(awk -F, 'NR==2 {print $9","$10}' "$TMP/rot_def/manifest.csv")
[ "$OUTDIMS" = "$(echo "$SRCDIMS" | awk -F, '{print $2","$1}')" ] \
  || fail "default transform should be a quarter turn: src $SRCDIMS -> out $OUTDIMS"
[ "$(awk -F, 'NR==2 {print $8}' "$TMP/rot_flat/manifest.csv")" = "kept" ] \
  || fail "unexpected manifest status"

# Now the pixels. Rotating clockwise moves the source's top-left corner to the
# output's top-right one, and that corner inverts to 255 minus itself.
SW=$(awk -F, 'NR==2 {print $6}' "$TMP/rot_flat/manifest.csv")
SH=$(awk -F, 'NR==2 {print $7}' "$TMP/rot_flat/manifest.csv")
FLAT_W=$(awk -F, 'NR==2 {print $9}' "$TMP/rot_flat/manifest.csv")
DEF_W=$(awk -F, 'NR==2 {print $9}' "$TMP/rot_def/manifest.csv")
DEF_H=$(awk -F, 'NR==2 {print $10}' "$TMP/rot_def/manifest.csv")
"$PNG_PIXEL" "$TMP/rot_flat/render/frame_0000.png" 0 0 > "$TMP/p_flat_tl" 2>/dev/null \
  || fail "cannot read a pixel from the unrotated frame"
"$PNG_PIXEL" "$TMP/rot_def/render/frame_0000.png" $((DEF_W - 1)) 0 > "$TMP/p_def_tr" 2>/dev/null \
  || fail "cannot read a pixel from the rotated frame"
read -r R0 G0 B0 < "$TMP/p_flat_tl"
read -r R1 G1 B1 < "$TMP/p_def_tr"
[ "$R1" = "$((255 - R0))" ] && [ "$G1" = "$((255 - G0))" ] && [ "$B1" = "$((255 - B0))" ] \
  || fail "default transform is not a clockwise turn with inversion: ($R0,$G0,$B0) -> ($R1,$G1,$B1)"

# A quarter turn clockwise puts the source's bottom-left corner at the output's
# top-left. A transpose would put the top-left corner there instead, so this is
# what separates the two.
"$PNG_PIXEL" "$TMP/rot_flat/render/frame_0000.png" 0 $((SH - 1)) > "$TMP/p_flat_bl" 2>/dev/null \
  || fail "cannot read the bottom-left pixel of the unrotated frame"
"$PNG_PIXEL" "$TMP/rot_def/render/frame_0000.png" 0 0 > "$TMP/p_def_tl" 2>/dev/null \
  || fail "cannot read the top-left pixel of the rotated frame"
read -r R2 G2 B2 < "$TMP/p_flat_bl"
read -r R3 G3 B3 < "$TMP/p_def_tl"
[ "$R3" = "$((255 - R2))" ] && [ "$G3" = "$((255 - G2))" ] && [ "$B3" = "$((255 - B2))" ] \
  || fail "turn is a transpose, not clockwise: bottom-left ($R2,$G2,$B2) should be at the top-left, got ($R3,$G3,$B3)"

# --rotate 0 must leave the frame upright and untouched.
[ "$FLAT_W" = "$SW" ] || fail "--rotate 0 should keep the source width $SW, got $FLAT_W"
echo "    ok rotated clockwise and inverted"

echo "e2e: all checks passed"
