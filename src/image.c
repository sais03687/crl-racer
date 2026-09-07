#include "image.h"

#include "inflate.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Small helper so every failure path is one line and always leaves the caller
 * with a message. It returns 0 so callers can write `return fail(...)`. */
static int fail(char *err, size_t err_len, const char *fmt, ...)
{
    if (err && err_len) {
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(err, err_len, fmt, ap);
        va_end(ap);
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* Reading the whole file into memory                                         */
/* ------------------------------------------------------------------------- */

/*
 * Slurp the file. Maps are a few tens of kilobytes to a few megabytes, so
 * reading the whole thing up front is simpler and faster than streaming, and
 * lets the parsers below just walk a byte array.
 */
static uint8_t *read_whole_file(const char *path, size_t *out_len)
{
    /* "rb" matters: on Windows the default text mode silently rewrites CRLF
     * byte pairs, which would corrupt any binary image. */
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;

    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
    long size = ftell(f);
    if (size < 0) { fclose(f); return NULL; }
    rewind(f);

    uint8_t *buf = (uint8_t *)malloc((size_t)size + 1);
    if (!buf) { fclose(f); return NULL; }

    size_t got = fread(buf, 1, (size_t)size, f);
    fclose(f);

    buf[got] = 0;   /* lets the PGM text parser treat the buffer as a string */
    *out_len = got;
    return buf;
}

/* ========================================================================= */
/* PGM                                                                        */
/* ========================================================================= */

/*
 * A PGM header is whitespace-separated tokens, with '#' comments running to
 * end of line. This little reader handles both.
 */
static int pgm_next_int(const uint8_t *d, size_t len, size_t *pos, int *out)
{
    size_t i = *pos;

    for (;;) {
        while (i < len && (d[i] == ' ' || d[i] == '\t' || d[i] == '\r' || d[i] == '\n'))
            i++;
        if (i < len && d[i] == '#') {          /* skip a comment line entirely */
            while (i < len && d[i] != '\n') i++;
            continue;
        }
        break;
    }

    if (i >= len || d[i] < '0' || d[i] > '9') return 0;

    int value = 0;
    while (i < len && d[i] >= '0' && d[i] <= '9') {
        value = value * 10 + (d[i] - '0');
        i++;
    }

    *pos = i;
    *out = value;
    return 1;
}

static int load_pgm(const uint8_t *d, size_t len, GrayImage *out,
                    char *err, size_t err_len)
{
    /* P2 stores pixels as decimal text, P5 as raw bytes. Same header. */
    int binary = (d[1] == '5');
    size_t pos = 2;

    int w, h, maxval;
    if (!pgm_next_int(d, len, &pos, &w) ||
        !pgm_next_int(d, len, &pos, &h) ||
        !pgm_next_int(d, len, &pos, &maxval))
        return fail(err, err_len, "PGM header is malformed");

    if (w <= 0 || h <= 0)
        return fail(err, err_len, "PGM has non-positive dimensions %dx%d", w, h);
    if (maxval <= 0 || maxval > 255)
        return fail(err, err_len,
                    "PGM maxval is %d; only 8-bit maps (maxval <= 255) are supported",
                    maxval);

    size_t count = (size_t)w * (size_t)h;
    uint8_t *px = (uint8_t *)malloc(count);
    if (!px) return fail(err, err_len, "out of memory for %dx%d image", w, h);

    if (binary) {
        /* Exactly one whitespace byte separates the header from the data. */
        pos++;
        if (pos + count > len) {
            free(px);
            return fail(err, err_len, "PGM pixel data is truncated");
        }
        memcpy(px, d + pos, count);
    } else {
        for (size_t i = 0; i < count; i++) {
            int v;
            if (!pgm_next_int(d, len, &pos, &v)) {
                free(px);
                return fail(err, err_len, "PGM ran out of pixels at index %zu", i);
            }
            px[i] = (uint8_t)v;
        }
    }

    /* Rescale if the file used a smaller range than 0..255, so that callers can
     * always assume 255 means "as bright as this image gets". */
    if (maxval != 255)
        for (size_t i = 0; i < count; i++)
            px[i] = (uint8_t)((px[i] * 255) / maxval);

    out->width  = w;
    out->height = h;
    out->pixels = px;
    return 1;
}

/* ========================================================================= */
/* PNG                                                                        */
/* ========================================================================= */

/*
 * PNG stores multi-byte integers big-endian (most significant byte first),
 * which is the opposite of x86's native order, so they must be assembled
 * explicitly rather than memcpy'd into a uint32_t.
 */
static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}

/*
 * Undo PNG's row filtering.
 *
 * Before compressing, PNG transforms each row by subtracting a prediction of
 * each byte from its actual value. Predictions come from the byte to the left,
 * the byte above, or a combination. Good predictions leave mostly zeroes,
 * which DEFLATE then squashes to almost nothing -- this is why PNG beats
 * plain zlib-over-pixels so decisively on photographs and maps alike.
 *
 * Each row carries a leading byte naming which of the five predictors was
 * used, and rows may use different ones. Decoding means adding the prediction
 * back, row by row, in order -- you cannot decode row 10 without row 9.
 *
 * `bpp` is the distance in bytes to the pixel on the left. For an RGB image
 * that is 3, so "the byte to the left" means the same colour channel of the
 * previous pixel, not the neighbouring channel.
 *
 * All the arithmetic is deliberately done modulo 256 (hence the & 255), which
 * is what the format specifies and what makes the transform exactly
 * reversible.
 */
static void png_unfilter_row(uint8_t *row, const uint8_t *prev,
                             size_t row_bytes, int bpp, int filter)
{
    switch (filter) {
    case 0:   /* None */
        break;

    case 1:   /* Sub: predict from the pixel to the left */
        for (size_t i = (size_t)bpp; i < row_bytes; i++)
            row[i] = (uint8_t)((row[i] + row[i - bpp]) & 255);
        break;

    case 2:   /* Up: predict from the pixel above */
        for (size_t i = 0; i < row_bytes; i++)
            row[i] = (uint8_t)((row[i] + prev[i]) & 255);
        break;

    case 3:   /* Average: predict from the mean of left and above */
        for (size_t i = 0; i < row_bytes; i++) {
            int left = (i >= (size_t)bpp) ? row[i - bpp] : 0;
            row[i] = (uint8_t)((row[i] + ((left + prev[i]) >> 1)) & 255);
        }
        break;

    case 4: {  /* Paeth: pick whichever of left/above/above-left is closest to
                * their linear estimate. Handles both horizontal and vertical
                * edges better than either simple predictor alone. */
        for (size_t i = 0; i < row_bytes; i++) {
            int a = (i >= (size_t)bpp) ? row[i - bpp] : 0;    /* left */
            int b = prev[i];                                  /* above */
            int c = (i >= (size_t)bpp) ? prev[i - bpp] : 0;   /* above-left */

            int estimate = a + b - c;
            int da = abs(estimate - a);
            int db = abs(estimate - b);
            int dc = abs(estimate - c);

            int predicted = (da <= db && da <= dc) ? a : (db <= dc ? b : c);
            row[i] = (uint8_t)((row[i] + predicted) & 255);
        }
        break;
    }

    default:
        break;   /* validated by the caller before we get here */
    }
}

static int load_png(const uint8_t *d, size_t len, GrayImage *out,
                    char *err, size_t err_len)
{
    /* --- Walk the chunks ------------------------------------------------ */
    /* A PNG is a signature followed by chunks, each: 4-byte length, 4-byte
     * type, payload, 4-byte CRC. Unknown chunk types are skippable by design,
     * which is why we can ignore gAMA, tEXt and friends without complaint. */
    size_t pos = 8;

    int width = 0, height = 0, bit_depth = 0, color_type = 0, interlace = 0;
    int have_header = 0;

    uint8_t  palette[256 * 3];
    int      palette_entries = 0;

    /* IDAT payloads may be split across several chunks and must be
     * concatenated before decompression -- the split points are arbitrary and
     * fall in the middle of the compressed bitstream. */
    uint8_t *idat     = NULL;
    size_t   idat_len = 0;

    while (pos + 8 <= len) {
        uint32_t chunk_len = be32(d + pos);
        const uint8_t *type = d + pos + 4;
        const uint8_t *data = d + pos + 8;

        if (pos + 12 + (size_t)chunk_len > len) {
            free(idat);
            return fail(err, err_len, "PNG chunk extends past end of file");
        }

        if (memcmp(type, "IHDR", 4) == 0) {
            if (chunk_len < 13) { free(idat); return fail(err, err_len, "PNG IHDR too short"); }
            width      = (int)be32(data);
            height     = (int)be32(data + 4);
            bit_depth  = data[8];
            color_type = data[9];
            interlace  = data[12];
            have_header = 1;
        } else if (memcmp(type, "PLTE", 4) == 0) {
            if (chunk_len > sizeof palette) { free(idat); return fail(err, err_len, "PNG palette too large"); }
            memcpy(palette, data, chunk_len);
            palette_entries = (int)(chunk_len / 3);
        } else if (memcmp(type, "IDAT", 4) == 0) {
            uint8_t *grown = (uint8_t *)realloc(idat, idat_len + chunk_len);
            if (!grown) { free(idat); return fail(err, err_len, "out of memory reading PNG data"); }
            idat = grown;
            memcpy(idat + idat_len, data, chunk_len);
            idat_len += chunk_len;
        } else if (memcmp(type, "IEND", 4) == 0) {
            break;
        }

        pos += 12 + (size_t)chunk_len;
    }

    /* --- Check we can actually handle this variant ---------------------- */
    if (!have_header)    { free(idat); return fail(err, err_len, "PNG has no IHDR chunk"); }
    if (!idat || !idat_len) { free(idat); return fail(err, err_len, "PNG has no image data"); }
    if (width <= 0 || height <= 0) { free(idat); return fail(err, err_len, "PNG has non-positive dimensions"); }

    /* PNG allows 1, 2, 4, 8 and 16 bits per channel and an interlaced layout.
     * Supporting all of that would double this file for no benefit: every map
     * in f1tenth_racetracks is 8-bit non-interlaced. Rather than half-support
     * the rest and produce a subtly wrong map, refuse clearly and say what to
     * do about it. */
    if (bit_depth != 8) {
        free(idat);
        return fail(err, err_len,
                    "PNG bit depth %d is unsupported (only 8 is); re-save the map as "
                    "8-bit, or convert it to PGM", bit_depth);
    }
    if (interlace != 0) {
        free(idat);
        return fail(err, err_len, "interlaced PNG is unsupported; re-save without interlacing");
    }

    int channels;
    switch (color_type) {
    case 0: channels = 1; break;   /* greyscale */
    case 2: channels = 3; break;   /* RGB */
    case 3: channels = 1; break;   /* palette index */
    case 4: channels = 2; break;   /* greyscale + alpha */
    case 6: channels = 4; break;   /* RGBA */
    default:
        free(idat);
        return fail(err, err_len, "PNG colour type %d is unsupported", color_type);
    }
    if (color_type == 3 && palette_entries == 0) {
        free(idat);
        return fail(err, err_len, "palette PNG has no PLTE chunk");
    }

    /* --- Decompress ------------------------------------------------------ */
    /* Every row is one filter byte plus width*channels pixel bytes, so the
     * decompressed size is known exactly and we can allocate it up front. */
    size_t row_bytes = (size_t)width * (size_t)channels;
    size_t raw_size  = ((size_t)height) * (row_bytes + 1);

    uint8_t *raw = (uint8_t *)malloc(raw_size);
    if (!raw) { free(idat); return fail(err, err_len, "out of memory for PNG pixels"); }

    long produced = zlib_inflate(idat, idat_len, raw, raw_size);
    free(idat);

    if (produced != (long)raw_size) {
        free(raw);
        return fail(err, err_len,
                    "PNG decompression produced %ld bytes, expected %zu",
                    produced, raw_size);
    }

    /* --- Unfilter, then flatten to greyscale ---------------------------- */
    uint8_t *gray = (uint8_t *)malloc((size_t)width * (size_t)height);
    if (!gray) { free(raw); return fail(err, err_len, "out of memory for greyscale image"); }

    /* Row 0 has no row above it. The spec says to treat that missing row as
     * all zeroes, so we point at a zero-filled scratch row for the first
     * iteration rather than special-casing every predictor. */
    uint8_t *zero_row = (uint8_t *)calloc(row_bytes, 1);
    if (!zero_row) { free(raw); free(gray); return fail(err, err_len, "out of memory"); }

    const uint8_t *prev = zero_row;
    for (int y = 0; y < height; y++) {
        uint8_t *row    = raw + (size_t)y * (row_bytes + 1);
        int      filter = row[0];
        row++;   /* step past the filter byte to the pixel bytes */

        if (filter > 4) {
            free(raw); free(gray); free(zero_row);
            return fail(err, err_len, "PNG row %d uses unknown filter %d", y, filter);
        }
        png_unfilter_row(row, prev, row_bytes, channels, filter);

        /* Collapse whatever colour format this was down to one brightness per
         * pixel. Occupancy only cares about light-versus-dark. */
        for (int x = 0; x < width; x++) {
            const uint8_t *p = row + (size_t)x * (size_t)channels;
            uint8_t value;

            if (color_type == 3) {
                int index = p[0];
                if (index >= palette_entries) index = 0;
                const uint8_t *rgb = palette + index * 3;
                /* Rec. 601 luma weights: the eye is far more sensitive to
                 * green than to blue, so a plain (r+g+b)/3 average would make
                 * blue walls look lighter than they are. */
                value = (uint8_t)((rgb[0] * 77 + rgb[1] * 150 + rgb[2] * 29) >> 8);
            } else if (color_type == 2 || color_type == 6) {
                value = (uint8_t)((p[0] * 77 + p[1] * 150 + p[2] * 29) >> 8);
            } else {
                value = p[0];   /* already grey; any alpha channel is ignored */
            }

            gray[(size_t)y * (size_t)width + (size_t)x] = value;
        }

        prev = row;   /* this row becomes the prediction source for the next */
    }

    free(zero_row);
    free(raw);

    out->width  = width;
    out->height = height;
    out->pixels = gray;
    return 1;
}

/* ========================================================================= */
/* Dispatch                                                                   */
/* ========================================================================= */

int image_load(const char *path, GrayImage *out, char *err, size_t err_len)
{
    out->width  = 0;
    out->height = 0;
    out->pixels = NULL;

    size_t len = 0;
    uint8_t *data = read_whole_file(path, &len);
    if (!data)
        return fail(err, err_len, "cannot open '%s'", path);

    if (len < 8) {
        free(data);
        return fail(err, err_len, "'%s' is too small to be an image", path);
    }

    int ok;
    static const uint8_t PNG_MAGIC[8] = { 0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n' };

    /* Sniff the leading bytes rather than trusting the extension: a map named
     * .pgm that is really a PNG is a common result of an image editor's
     * "save as", and failing on it would be a confusing dead end. */
    if (memcmp(data, PNG_MAGIC, 8) == 0) {
        ok = load_png(data, len, out, err, err_len);
    } else if (data[0] == 'P' && (data[1] == '2' || data[1] == '5')) {
        ok = load_pgm(data, len, out, err, err_len);
    } else {
        ok = fail(err, err_len,
                  "'%s' is neither a PNG nor a P2/P5 PGM (starts with 0x%02X 0x%02X)",
                  path, data[0], data[1]);
    }

    free(data);
    return ok;
}

void image_free(GrayImage *img)
{
    free(img->pixels);
    img->pixels = NULL;
    img->width  = 0;
    img->height = 0;
}
