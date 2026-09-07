#ifndef IMAGE_H
#define IMAGE_H

#include <stddef.h>
#include <stdint.h>

/*
 * ============================================================================
 * GREYSCALE IMAGE LOADING (PGM and PNG)
 * ============================================================================
 *
 * A track map is a picture: dark pixels are walls, light pixels are drivable
 * road. This file's only job is to turn a file on disk into a flat array of
 * brightness values, one byte per pixel, 0 = black and 255 = white. Everything
 * above it (grid.c) works purely in terms of that array and never needs to
 * know which format the file was in.
 *
 * Two formats are supported, for a practical reason: the ROS tooling that
 * F1TENTH grew out of writes maps as PGM, but the published f1tenth_racetracks
 * repository ships PNG. You need both to load real data.
 *
 *   PGM ("portable grey map") is about as simple as a file format gets: a
 *   two-character magic number, three integers, then the pixels. You could
 *   write one by hand in a text editor.
 *
 *   PNG is a real format: chunked, checksummed, compressed, and with a
 *   per-row "filter" step layered on top of the compression. It is decoded
 *   here from scratch (see inflate.c for the compression half).
 *
 * The format is detected from the file's leading bytes, not from its
 * extension, because the extension is just a hint and can be wrong.
 *
 * ---------------------------------------------------------------------------
 * Memory ownership
 * ---------------------------------------------------------------------------
 * image_load() allocates `pixels` with malloc. The caller owns it and must
 * call image_free(). On failure nothing is allocated and `pixels` is left
 * NULL, so calling image_free() on a failed load is harmless.
 */
typedef struct GrayImage {
    int      width;
    int      height;
    uint8_t *pixels;   /* width*height bytes, row-major, row 0 is the TOP row */
} GrayImage;

/*
 * Load `path` into `out`. Returns 1 on success, 0 on failure.
 *
 * On failure a human-readable explanation is written into `err` (which may be
 * NULL if you do not want one). Errors are reported this way rather than as a
 * numeric code because there are a dozen distinct ways an image file can be
 * wrong and "could not load map" alone is useless when you are debugging.
 */
int  image_load(const char *path, GrayImage *out, char *err, size_t err_len);

void image_free(GrayImage *img);

#endif /* IMAGE_H */
