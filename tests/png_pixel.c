/* png_pixel.c - print one pixel of a PNG. Used by the end-to-end test to check
 * the rotation and inversion actually applied to the frames. Not part of the
 * shipped tool.
 *
 * usage: png_pixel <file.png> <x> <y>   ->   "r g b"
 */
#include <stdio.h>
#include <stdlib.h>

#include <png.h>

int main(int argc, char **argv)
{
    FILE *fp;
    png_structp png;
    png_infop info;
    png_bytep row;
    png_uint_32 w, h;
    int bitdepth, colortype;
    long x, y;

    if (argc != 4) {
        fprintf(stderr, "usage: %s <file.png> <x> <y>\n", argv[0]);
        return 2;
    }
    x = strtol(argv[2], NULL, 10);
    y = strtol(argv[3], NULL, 10);

    fp = fopen(argv[1], "rb");
    if (!fp) {
        fprintf(stderr, "png_pixel: cannot open %s\n", argv[1]);
        return 1;
    }
    png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png) {
        fclose(fp);
        return 1;
    }
    info = png_create_info_struct(png);
    if (!info || setjmp(png_jmpbuf(png))) {
        png_destroy_read_struct(&png, info ? &info : NULL, NULL);
        fclose(fp);
        fprintf(stderr, "png_pixel: cannot read %s\n", argv[1]);
        return 1;
    }
    png_init_io(png, fp);
    png_read_info(png, info);
    w = png_get_image_width(png, info);
    h = png_get_image_height(png, info);
    bitdepth = png_get_bit_depth(png, info);
    colortype = png_get_color_type(png, info);

    if (bitdepth == 16)
        png_set_strip_16(png);
    if (colortype == PNG_COLOR_TYPE_PALETTE)
        png_set_palette_to_rgb(png);
    if (colortype == PNG_COLOR_TYPE_GRAY && bitdepth < 8)
        png_set_expand_gray_1_2_4_to_8(png);
    if (png_get_valid(png, info, PNG_INFO_tRNS))
        png_set_tRNS_to_alpha(png);
    png_read_update_info(png, info);

    if (x < 0 || y < 0 || (png_uint_32) x >= w || (png_uint_32) y >= h) {
        fprintf(stderr, "png_pixel: %s is %ux%u, (%ld,%ld) is outside\n",
                argv[1], (unsigned) w, (unsigned) h, x, y);
        return 1;
    }

    /* Read the whole image: png_read_row() only ever hands back the next row
     * in sequence, so walking to y would mean looping; these frames are small
     * and a single png_read_image() keeps the indexing honest. */
    row = (png_bytep) malloc(png_get_rowbytes(png, info) * h);
    if (!row) {
        png_destroy_read_struct(&png, &info, NULL);
        fclose(fp);
        return 1;
    }
    {
        png_bytep *rows = malloc(sizeof(png_bytep) * h);
        png_uint_32 i;
        if (!rows) {
            free(row);
            png_destroy_read_struct(&png, &info, NULL);
            fclose(fp);
            return 1;
        }
        for (i = 0; i < h; i++)
            rows[i] = row + i * png_get_rowbytes(png, info);
        png_read_image(png, rows);
        free(rows);
    }
    printf("%d %d %d\n", row[y * png_get_rowbytes(png, info) + x * 3],
           row[y * png_get_rowbytes(png, info) + x * 3 + 1],
           row[y * png_get_rowbytes(png, info) + x * 3 + 2]);
    free(row);

    png_destroy_read_struct(&png, &info, NULL);
    fclose(fp);
    return 0;
}
