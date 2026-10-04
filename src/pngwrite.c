/* pngwrite.c - minimal 8-bit RGB PNG writer built on libpng. */
#include "pngwrite.h"

#include <png.h>
#include <stdio.h>
#include <stdlib.h>

int png_write_rgb(const char *path, const unsigned char *rgb, int w, int h)
{
    FILE *fp;
    png_structp png = NULL;
    png_infop info = NULL;
    /* volatile: these are read after longjmp() returns from libpng. */
    png_bytep * volatile rows = NULL;
    volatile int rc = -1;
    int y;

    if (!path || !rgb || w <= 0 || h <= 0)
        return -1;

    fp = fopen(path, "wb");
    if (!fp)
        return -1;

    png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png)
        goto done;
    info = png_create_info_struct(png);
    if (!info)
        goto done;
    if (setjmp(png_jmpbuf(png)))
        goto done;

    png_init_io(png, fp);
    png_set_IHDR(png, info, (png_uint_32) w, (png_uint_32) h, 8,
                 PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    rows = malloc(sizeof(png_bytep) * (size_t) h);
    if (!rows)
        goto done;
    for (y = 0; y < h; y++)
        rows[y] = (png_bytep) (rgb + (size_t) y * w * 3);

    png_write_image(png, rows);
    png_write_end(png, NULL);
    rc = 0;

done:
    free(rows);
    if (png)
        png_destroy_write_struct(&png, info ? &info : NULL);
    if (fclose(fp) != 0)
        rc = -1;
    return rc;
}
