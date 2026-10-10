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
OBJS    := $(BUILD)/framescan.o $(BUILD)/sprocketfind.o \
           $(BUILD)/stabilize.o $(BUILD)/pngwrite.o

SCANS     ?= scans
OUT       ?= out
FPS       ?= 18
# Frames come out of framescan rotated 90 deg clockwise, so they are portrait:
# the width is the frame's height along the strip, the height the frame width.
# Every frame framescan emits is exactly the same size, so the render must not
# touch the size at all: scaling each frame to a fixed canvas is what made the
# film appear to jump when the crop size drifted. The scale filter below only
# exists to force even dimensions for yuv420p, and the crops already are.
RENDER_FILTER ?= setsar=1,scale=trunc(iw/2)*2:trunc(ih/2)*2
MP4       ?= $(OUT)/frames.mp4

.PHONY: all test unit e2e clean frames render

all: $(BIN)

$(BIN): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(JPEG_LIBS) $(PNG_LIBS) $(MATH_LIBS) -lm

$(BUILD)/%.o: src/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(JPEG_CFLAGS) $(PNG_CFLAGS) -c -o $@ $<

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/framescan.o: src/sprocketfind.h src/stabilize.h src/pngwrite.h
$(BUILD)/sprocketfind.o: src/sprocketfind.h
$(BUILD)/stabilize.o: src/stabilize.h src/sprocketfind.h
$(BUILD)/pngwrite.o: src/pngwrite.h

$(BUILD)/make_fixture: tests/make_fixture.c tests/fixture.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(JPEG_CFLAGS) -Itests -o $@ \
	  tests/make_fixture.c $(JPEG_LIBS)

$(BUILD)/png_pixel: tests/png_pixel.c | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(PNG_CFLAGS) -o $@ tests/png_pixel.c \
	  $(PNG_LIBS) $(MATH_LIBS)

unit:
	@echo "no unit tests for the 8mm pipeline (covered by e2e)"

e2e: all $(BUILD)/make_fixture $(BUILD)/png_pixel
	@sh tests/e2e.sh

test: unit e2e

## Extract cropped, stabilized frames into $(OUT)/render
frames: all
	./$(BIN) -o $(OUT) $(SCANS)

## Extract frames and mux them into $(MP4) at $(FPS) fps
render: frames
	@command -v ffmpeg >/dev/null 2>&1 || { \
	  echo "ffmpeg not found in PATH; install it to build $(MP4); exit 1; }
	ffmpeg -hide_banner -loglevel error -y \
	  -framerate $(FPS) -i $(OUT)/render/frame_%04d.png \
	  -vf "$(RENDER_FILTER)" \
	  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p -r $(FPS) \
	  -movflags +faststart $(MP4)
	@echo "rendered $(MP4) at $(FPS) fps"

clean:
	rm -rf $(BUILD) $(BIN)
