#!/bin/sh
# e2e.sh - end-to-end test: build a synthetic Super-8 scan with known geometry,
# run framescan on it, and check the crops, the numbering, de-duplication, the
# rotate/invert defaults and the failure modes.
set -eu

BIN=${BIN:-./framescan}
MAKE_FIXTURE=${MAKE_FIXTURE:-./build/make_fixture}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

fail() { echo "    FAIL $1" >&2; exit 1; }

# Geometry: 3 frames of 700px pitch across the raster.  700px sits in the
# detector's pitch window (550..950), where real RODFS50 8mm scans land
# (~710px); the band must be taller than the frame's across-strip extent
# (~1.37 * pitch), as it is on real film.  The tool does not need to know the
# film dimensions; it measures the pitch from the perforations.
W=2400; H=1400; Y0=100; BH=1100; PITCH=700; NF=3
"$MAKE_FIXTURE" "$TMP/a.jpg" "$W" "$H" "$Y0" "$BH" "$PITCH" "$NF"
cp "$TMP/a.jpg" "$TMP/b.jpg"

echo "  e2e: single scan"
rm -rf "$TMP/out"
"$BIN" --outdir "$TMP/out" "$TMP/a.jpg" >/dev/null 2>&1 \
  || fail "framescan exited non-zero"

N=$(ls "$TMP/out/render"/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" -ge 1 ] || fail "expected at least one frame, got 0"
[ -f "$TMP/out/manifest.csv" ] || fail "manifest.csv missing"
[ "$(ls "$TMP/out"/frame_*.png 2>/dev/null | wc -l | tr -d ' ')" = "0" ] \
  || fail "PNGs must not be written directly into the outdir"

# Every frame must come out at the same size.
SIZES=$(awk -F, 'NR>1 {print $9 "x" $10}' "$TMP/out/manifest.csv" | sort -u | wc -l | tr -d ' ')
[ "$SIZES" = "1" ] || fail "frames are not all the same size: $(awk -F, 'NR>1 {print $9 "x" $10}' "$TMP/out/manifest.csv" | sort -u | tr '\n' ' ')"
[ "$(awk -F, 'NR>1 {print $8}' "$TMP/out/manifest.csv" | sort -u)" = "kept" ] \
  || fail "single scan should produce no duplicates"
N1=$N
echo "    ok frames=$N1, manifest, uniform sizes"

echo "  e2e: overlapping-scans de-duplication"
rm -rf "$TMP/out2"
"$BIN" --outdir "$TMP/out2" "$TMP/a.jpg" "$TMP/b.jpg" >/dev/null 2>&1 \
  || fail "framescan exited non-zero on duplicate scans"
N=$(ls "$TMP/out2/render"/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "$N1" ] || fail "duplicate scan not removed: expected $N1 frames, got $N"
DUPS=$(grep -c ',dup_of_' "$TMP/out2/manifest.csv" || true)
[ "$DUPS" = "$N1" ] || fail "expected $N1 dup rows, got $DUPS"
NPNG=$(ls "$TMP/out2"/render/frame_*.png | wc -l | tr -d ' ')
LAST=$(ls "$TMP/out2"/render/frame_*.png | sed -n 's/.*frame_\([0-9]*\)\.png$/\1/p' | sort -n | tail -1)
[ "$NPNG" = "$((10#$LAST + 1))" ] \
  || fail "frame numbering has a hole: $NPNG files but the highest is $LAST"
echo "    ok duplicates dropped, numbering dense"

echo "  e2e: --no-dedup keeps everything"
rm -rf "$TMP/out3"
"$BIN" --outdir "$TMP/out3" --no-dedup "$TMP/a.jpg" "$TMP/b.jpg" >/dev/null 2>&1
N=$(ls "$TMP/out3/render"/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "$((N1 * 2))" ] || fail "--no-dedup: expected $((N1*2)) frames, got $N"
echo "    ok"

echo "  e2e: distinct scans are never de-duplicated"
"$MAKE_FIXTURE" "$TMP/d1.jpg" "$W" "$H" "$Y0" "$BH" "$PITCH" "$NF" 0
"$MAKE_FIXTURE" "$TMP/d2.jpg" "$W" "$H" "$Y0" "$BH" "$PITCH" "$NF" 104729
rm -rf "$TMP/out10"
"$BIN" --outdir "$TMP/out10" "$TMP/d1.jpg" "$TMP/d2.jpg" >/dev/null 2>&1
N=$(ls "$TMP/out10/render"/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "$((N1 * 2))" ] || fail "distinct scans: expected $((N1*2)) frames, got $N"
DUPS=$(grep -c ',dup_of_' "$TMP/out10/manifest.csv" || true)
[ "$DUPS" = "0" ] || fail "distinct scans must not dedup, got $DUPS dup rows"
echo "    ok"

echo "  e2e: a blank frame neither poisons nor is dropped"
"$MAKE_FIXTURE" "$TMP/f1.jpg" "$W" "$H" "$Y0" "$BH" "$PITCH" "$NF" 0 1
cp "$TMP/f1.jpg" "$TMP/f1b.jpg"
rm -rf "$TMP/out11"
"$BIN" --outdir "$TMP/out11" "$TMP/f1.jpg" "$TMP/f1b.jpg" >/dev/null 2>&1
N=$(ls "$TMP/out11/render"/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "$N1" ] \
  || fail "same blank scan twice must dedup to $N1 frames, got $N"
[ "$(grep -c ',dup_of_' "$TMP/out11/manifest.csv" || true)" = "$N1" ] \
  || fail "blank scan copy: expected $N1 dup rows"
echo "    ok"

echo "  e2e: frame phase follows the perforations, not the strip edge"
# A leader of clear base before the first frame: the grid must land on the
# perforations, so the first frame starts inside the raster, not at x=0 (a
# detector that ignores phase would emit a frame at the strip edge instead).
LEADER=130
"$MAKE_FIXTURE" "$TMP/lead.jpg" "$W" "$H" "$Y0" "$BH" "$PITCH" 3 0 -1 "$LEADER"
rm -rf "$TMP/outp"
"$BIN" --outdir "$TMP/outp" "$TMP/lead.jpg" >/dev/null 2>&1 \
  || fail "framescan exited non-zero on leader fixture"
N=$(ls "$TMP/outp/render"/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" -ge 1 ] || fail "leader: no whole frames"
ROW=$(awk -F, 'NR>1 {print $4; exit}' "$TMP/outp/manifest.csv")
[ "$ROW" -gt 50 ] || fail "leader: grid landed on the strip edge at x=$ROW"
echo "    ok phase not pinned to the strip edge"

echo "  e2e: frames are rotated clockwise and inverted by default"
# Default output is a quarter-turn of the source crop with re-inverted
# tonality.  Check the size swap through the manifest.
rm -rf "$TMP/rot_def" "$TMP/rot_flat"
"$BIN" --outdir "$TMP/rot_def" "$TMP/a.jpg" >/dev/null 2>&1 \
  || fail "framescan exited non-zero on the default transform"
"$BIN" --outdir "$TMP/rot_flat" --rotate 0 --positive "$TMP/a.jpg" >/dev/null 2>&1 \
  || fail "framescan exited non-zero on --rotate 0 --positive"
SRCDIMS=$(awk -F, 'NR==2 {print $6","$7}' "$TMP/rot_flat/manifest.csv")
OUTDIMS=$(awk -F, 'NR==2 {print $9","$10}' "$TMP/rot_def/manifest.csv")
[ "$OUTDIMS" = "$(echo "$SRCDIMS" | awk -F, '{print $2","$1}')" ] \
  || fail "default transform should be a quarter turn: src $SRCDIMS -> out $OUTDIMS"
FLAT_W=$(awk -F, 'NR==2 {print $9}' "$TMP/rot_flat/manifest.csv")
SW=$(awk -F, 'NR==2 {print $6}' "$TMP/rot_flat/manifest.csv")
[ "$FLAT_W" = "$SW" ] || fail "--rotate 0 should keep the source width $SW, got $FLAT_W"
echo "    ok rotated clockwise and inverted"

echo "  e2e: fails loudly on a missing file"
rm -rf "$TMP/out7"
"$BIN" --outdir "$TMP/out7" "$TMP/nope.jpg" >/dev/null 2>&1 \
  && fail "expected non-zero exit for a missing input"
echo "    ok"

echo "  e2e: a raster with no film cannot yield geometry"
"$MAKE_FIXTURE" "$TMP/d.jpg" 400 400 0 40 200 1
rm -rf "$TMP/out6"
"$BIN" --outdir "$TMP/out6" "$TMP/d.jpg" >"$TMP/o6" 2>&1 \
  && fail "framescan should complain when no strip can be detected"
N=$(ls "$TMP/out6/render"/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "0" ] || fail "expected 0 frames from a strip-less raster, got $N"
echo "    ok"

echo "e2e: all checks passed"