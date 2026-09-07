#ifndef INFLATE_H
#define INFLATE_H

#include <stddef.h>
#include <stdint.h>

/*
 * ============================================================================
 * DEFLATE / ZLIB DECOMPRESSION
 * ============================================================================
 *
 * Why is a decompressor in a racing simulator?
 *
 * Because the real F1TENTH track maps are distributed as PNG images, and every
 * PNG stores its pixels compressed with an algorithm called DEFLATE, wrapped
 * in a small container called zlib. Normally you would link against libz and
 * forget about it, but this project is allowed nothing beyond the C standard
 * library, so we decompress the data ourselves. It is about 250 lines.
 *
 * ---------------------------------------------------------------------------
 * How DEFLATE works, in one page
 * ---------------------------------------------------------------------------
 *
 * DEFLATE combines two classic ideas.
 *
 * 1. BACK-REFERENCES (LZ77). Text repeats itself. Instead of storing the
 *    letters of a repeat, you store a pair meaning "go back D bytes in what
 *    you have already produced, and copy L bytes from there". A map image with
 *    long runs of identical white pixels compresses enormously this way.
 *
 * 2. HUFFMAN CODING. Not all symbols are equally common. Rather than spending
 *    8 bits on every byte, give common symbols short bit-patterns and rare
 *    ones long patterns. The patterns are chosen so that no code is a prefix
 *    of another, which means you can read a stream of them with no separators
 *    and never be ambiguous about where one ends.
 *
 * The compressed stream is a sequence of BLOCKS. Each block starts with a bit
 * saying "am I the last one" and two bits saying which of three kinds it is:
 *
 *    stored  -- not compressed at all, just raw bytes (used when compressing
 *               would make things bigger, e.g. already-random data)
 *    fixed   -- Huffman-coded using a table hardcoded into the standard, so
 *               the table costs zero bits to transmit
 *    dynamic -- Huffman-coded using a table built for this block specifically
 *               and written into the block ahead of the data
 *
 * Inside a compressed block you decode symbols one at a time. Symbols 0..255
 * are literal bytes: emit them. Symbol 256 means "end of block". Symbols
 * 257..285 mean "this is a back-reference" and encode the length; a second
 * Huffman code then gives the distance. Because lengths go up to 258 and
 * distances up to 32768, and you cannot afford a distinct symbol for each,
 * some symbols are followed by a few literal "extra bits" that fine-tune the
 * value within a range. Hence the tables of bases and extra-bit counts below.
 *
 * ---------------------------------------------------------------------------
 * One subtlety that trips people up
 * ---------------------------------------------------------------------------
 *
 * A back-reference may overlap the output it is producing: "go back 1 byte and
 * copy 5" is legal and means "repeat the previous byte 5 times". So the copy
 * MUST be a byte-at-a-time loop reading from the buffer as it grows. A
 * memcpy() would read stale bytes and silently corrupt the image.
 *
 * This implementation is written for clarity, not speed. It walks the Huffman
 * tree one bit at a time instead of using the multi-bit lookup tables a real
 * library would. On a 4 MB map that costs a fraction of a second, once, at
 * startup -- and it makes the code readable, which is the point.
 */

/*
 * Decompress a zlib stream (a 2-byte header, a DEFLATE payload, and a 4-byte
 * Adler-32 checksum) from `in` into `out`.
 *
 * Returns the number of bytes written, or -1 if the stream is malformed, the
 * checksum does not match, or the output would not fit in `out_cap`.
 *
 * The caller must know the uncompressed size in advance. That is fine here:
 * a PNG's pixel data has a size fixed by its width, height and colour format,
 * so we can allocate exactly the right buffer and treat any overflow as
 * corruption rather than growing a buffer at runtime.
 */
long zlib_inflate(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap);

#endif /* INFLATE_H */
