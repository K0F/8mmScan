#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <png.h>

int png_write_rgb(const char *path, const unsigned char *rgb, int w, int h)
{
    FILE *fp = fopen(path, "wb");
    if (!fp) return -1;
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png) { fclose(fp); return -1; }
    png_infop info = png_create_info_struct(png);
    if (!info) {
        png_destroy_write_struct(&png, NULL);
        fclose(fp);
        return -1;
    }
    if (setjmp(png_jmpbuf(png))) {
        png_destroy_write_struct(&png, &info);
        fclose(fp);
        return -1;
    }
    png_init_io(png, fp);
    png_set_IHDR(png, info, w, h, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);
    for (int y = 0; y < h; y++) {
        png_write_row(png, (png_bytep)(rgb + (size_t)y * w * 3));
    }
    png_write_end(png, info);
    png_destroy_write_struct(&png, &info);
    fclose(fp);
    return 0;
}
