/* make_fixture.c - write a synthetic 35mm scan with known geometry, for the
 * end-to-end test. Not part of the shipped tool.
 *
 * usage: make_fixture <out.jpg> <w> <h> <band_y0> <band_h> <pitch> <nframes>
 *                     [seed_off] [blank_frame] [leader]
 *
 * leader shifts the frame run sideways, so the strip edge and the first frame
 * boundary do not coincide.
 *
 * seed_off changes the per-frame pattern so two fixtures with the same
 * geometry hold genuinely different pictures. blank_frame leaves one frame
 * with no texture, as an unexposed or blank frame would be.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <jpeglib.h>   /* must follow <stdio.h> */

#include "fixture.h"

int main(int argc, char **argv)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE *fp;
    unsigned char *img;
    int w, h, y0, bh, pitch, nf;
    unsigned int seed_off = 0;
    int blank = -1;
    int leader = 0;
    JSAMPROW rowptr[1];

    if (argc < 8 || argc > 11) {
        fprintf(stderr,
                "usage: %s out.jpg w h band_y0 band_h pitch nframes"
                " [seed_off] [blank_frame]\n",
                argv[0]);
        return 2;
    }
    w = atoi(argv[2]);
    h = atoi(argv[3]);
    y0 = atoi(argv[4]);
    bh = atoi(argv[5]);
    pitch = atoi(argv[6]);
    nf = atoi(argv[7]);
    if (w <= 0 || h <= 0 || y0 < 0 || bh <= 0 || y0 + bh > h || pitch <= 0 ||
        nf <= 0) {
        fprintf(stderr, "make_fixture: bad geometry\n");
        return 2;
    }

    if (argc > 8)
        seed_off = (unsigned int) strtoul(argv[8], NULL, 10);
    if (argc > 9)
        blank = atoi(argv[9]);
    if (argc > 10)
        leader = atoi(argv[10]);

    img = fx_raster(w, h, y0, bh, pitch, nf, seed_off, blank, leader);
    if (!img) {
        fprintf(stderr, "make_fixture: out of memory\n");
        return 1;
    }

    fp = fopen(argv[1], "wb");
    if (!fp) {
        fprintf(stderr, "make_fixture: cannot write %s\n", argv[1]);
        free(img);
        return 1;
    }
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, fp);
    cinfo.image_width = (JDIMENSION) w;
    cinfo.image_height = (JDIMENSION) h;
    cinfo.input_components = 1;
    cinfo.in_color_space = JCS_GRAYSCALE;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, 92, TRUE);
    jpeg_start_compress(&cinfo, TRUE);
    while (cinfo.next_scanline < cinfo.image_height) {
        rowptr[0] = img + (size_t) cinfo.next_scanline * w;
        jpeg_write_scanlines(&cinfo, rowptr, 1);
    }
    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    fclose(fp);
    free(img);
    return 0;
}
