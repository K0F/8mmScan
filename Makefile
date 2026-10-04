CC      ?= cc
CFLAGS  ?= -O2 -g -std=c99 -Wall -Wextra -Wno-unused-parameter
CPPFLAGS += -Isrc -D_POSIX_C_SOURCE=200809L

JPEG_CFLAGS := $(shell pkg-config --cflags libjpeg 2>/dev/null)
JPEG_LIBS   := $(shell pkg-config --libs   libjpeg 2>/dev/null || echo -ljpeg)
PNG_CFLAGS  := $(shell pkg-config --cflags libpng 2>/dev/null)
PNG_LIBS    := $(shell pkg-config --libs   libpng 2>/dev/null || echo -lpng)
MATH_LIBS   := $(shell pkg-config --libs   zlib 2>/dev/null || echo -lz)

BIN     := framescan
BUILD   := build
OBJS    := $(BUILD)/framescan.o $(BUILD)/filmfind.o $(BUILD)/pngwrite.o

SCANS     ?= scans
OUT       ?= out
FPS       ?= 18
# Frames come out of framescan rotated 90 deg clockwise, so they are portrait:
# the width is the frame's height along the strip, the height the frame width.
RENDER_W  ?= 936
RENDER_H  ?= 716
MP4       ?= $(OUT)/frames.mp4

.PHONY: all test unit e2e clean frames render

all: $(BIN)

$(BIN): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(JPEG_LIBS) $(PNG_LIBS) $(MATH_LIBS) -lm

$(BUILD)/%.o: src/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(JPEG_CFLAGS) $(PNG_CFLAGS) -c -o $@ $<

$(BUILD)/test_filmfind: tests/test_filmfind.c tests/fixture.h src/filmfind.c | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -Itests -o $@ tests/test_filmfind.c src/filmfind.c -lm

$(BUILD)/make_fixture: tests/make_fixture.c tests/fixture.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(JPEG_CFLAGS) -Itests -o $@ tests/make_fixture.c $(JPEG_LIBS)

$(BUILD):
	mkdir -p $(BUILD)

$(OBJS): src/filmfind.h | $(BUILD)
$(BUILD)/framescan.o $(BUILD)/pngwrite.o: src/pngwrite.h

unit: $(BUILD)/test_filmfind
	@$(BUILD)/test_filmfind

$(BUILD)/png_pixel: tests/png_pixel.c | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(PNG_CFLAGS) -o $@ tests/png_pixel.c $(PNG_LIBS) $(MATH_LIBS)

e2e: all $(BUILD)/make_fixture $(BUILD)/png_pixel
	@sh tests/e2e.sh

test: unit e2e

## Extract cropped frames into $(OUT)/render
frames: all
	./$(BIN) -o $(OUT) $(SCANS)

## Extract frames and mux them into $(MP4) at $(FPS) fps
render: frames
	@command -v ffmpeg >/dev/null 2>&1 || { \
	  echo "ffmpeg not found in PATH; install it to build $(MP4)"; exit 1; }
	ffmpeg -hide_banner -loglevel error -y \
	  -framerate $(FPS) -i $(OUT)/render/frame_%04d.png \
	  -vf "scale=$(RENDER_W):$(RENDER_H):force_original_aspect_ratio=decrease,\
pad=$(RENDER_W):$(RENDER_H):(ow-iw)/2:(oh-ih)/2,setsar=1" \
	  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p -r $(FPS) \
	  -movflags +faststart $(MP4)
	@echo "rendered $(MP4) at $(FPS) fps"

clean:
	rm -rf $(BUILD) $(BIN)
