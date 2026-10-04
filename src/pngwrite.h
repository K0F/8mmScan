/* pngwrite.h - minimal 8-bit RGB PNG writer. */
#ifndef PNGWRITE_H
#define PNGWRITE_H

/* Writes w*h*3 packed RGB bytes to `path`. Returns 0 on success, -1 on error. */
int png_write_rgb(const char *path, const unsigned char *rgb, int w, int h);

#endif /* PNGWRITE_H */
