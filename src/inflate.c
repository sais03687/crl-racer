#include "inflate.h"

#include <string.h>

/* DEFLATE never uses a Huffman code longer than 15 bits, and the largest
 * alphabet (literals + lengths) has 288 symbols. Both limits come from the
 * format specification (RFC 1951), so fixed-size arrays are safe here. */
#define MAX_BITS    15
#define MAX_SYMBOLS 288

/*
 * A canonical Huffman code. "Canonical" means the code patterns are not stored
 * at all -- only how MANY symbols use each bit-length, plus the symbols listed
 * in the standard's canonical order. The actual bit patterns can then be
 * reconstructed by counting, which is why a DEFLATE block can describe an
 * entire table using just a list of lengths.
 */
typedef struct {
    short counts[MAX_BITS + 1];   /* counts[n] = how many codes are n bits long */
    short symbols[MAX_SYMBOLS];   /* symbols, ordered by their code value */
} Huffman;

/* Everything the decoder needs to know about where it is. Passed by pointer to
 * every helper so there is no global state and the whole thing is reentrant. */
typedef struct {
    const uint8_t *in;
    size_t         in_len;
    size_t         in_pos;

    /* Bits are consumed least-significant-first, and Huffman codes can straddle
     * byte boundaries, so we keep a small reservoir of bits pulled from the
     * input and hand them out a few at a time. */
    uint32_t bitbuf;
    int      bitcnt;

    uint8_t *out;
    size_t   out_cap;
    size_t   out_pos;

    /* Sticky error flag. Once set, every further operation is a no-op and
     * returns a harmless value. This keeps error handling out of the hot path:
     * we check once at the end instead of after every single bit read. */
    int err;
} Puff;

/* ------------------------------------------------------------------------- */
/* Bit input                                                                  */
/* ------------------------------------------------------------------------- */

static uint32_t puff_bits(Puff *s, int need)
{
    while (s->bitcnt < need) {
        if (s->in_pos >= s->in_len) { s->err = 1; return 0; }
        /* New bits go above the ones already buffered, because the stream is
         * read least-significant-bit first within each byte. */
        s->bitbuf |= (uint32_t)s->in[s->in_pos++] << s->bitcnt;
        s->bitcnt += 8;
    }
    uint32_t val = s->bitbuf & ((1u << need) - 1u);
    s->bitbuf >>= need;
    s->bitcnt  -= need;
    return val;
}

/* ------------------------------------------------------------------------- */
/* Huffman decoding                                                           */
/* ------------------------------------------------------------------------- */

/*
 * Build the count/symbol tables from a plain list of code lengths, where
 * lengths[sym] is how many bits symbol `sym` uses (0 meaning "unused").
 */
static void huffman_build(Huffman *h, const uint8_t *lengths, int n)
{
    memset(h->counts, 0, sizeof h->counts);
    for (int sym = 0; sym < n; sym++)
        h->counts[lengths[sym]]++;

    /* Length 0 means "this symbol never appears", so it is not a real code. */
    h->counts[0] = 0;

    /* Lay the symbols out in canonical order: all the 1-bit symbols first (in
     * increasing symbol order), then all the 2-bit symbols, and so on. Offsets
     * tells us where each length's run begins. */
    short offsets[MAX_BITS + 1];
    offsets[1] = 0;
    for (int len = 1; len < MAX_BITS; len++)
        offsets[len + 1] = (short)(offsets[len] + h->counts[len]);

    for (int sym = 0; sym < n; sym++)
        if (lengths[sym])
            h->symbols[offsets[lengths[sym]]++] = (short)sym;
}

/*
 * Read one symbol.
 *
 * The trick that makes canonical Huffman decodable without a tree: at each
 * bit-length, the codes of that length occupy a contiguous numeric range, and
 * the range for the next length starts where this one ended, shifted left by
 * one. So we accumulate bits into `code`, and at each length ask "is `code`
 * inside this length's range?". If yes, its offset within the range indexes
 * straight into the symbol list. If no, skip past this length's symbols and
 * take another bit.
 */
static int huffman_decode(Puff *s, const Huffman *h)
{
    int code  = 0;   /* the bits read so far, as a number */
    int first = 0;   /* smallest code of the current length */
    int index = 0;   /* where this length's symbols start in h->symbols */

    for (int len = 1; len <= MAX_BITS; len++) {
        code |= (int)puff_bits(s, 1);
        if (s->err) return -1;

        int count = h->counts[len];
        if (code - first < count)
            return h->symbols[index + (code - first)];

        index += count;
        first  = (first + count) << 1;
        code <<= 1;
    }

    s->err = 1;   /* ran past 15 bits: the stream is corrupt */
    return -1;
}

/* ------------------------------------------------------------------------- */
/* Length and distance tables (RFC 1951, section 3.2.5)                       */
/* ------------------------------------------------------------------------- */

/*
 * Symbols 257..285 encode a copy length. Each symbol has a base length and a
 * number of extra bits read straight from the stream and added to the base.
 * This is how 29 symbols cover the 256 possible lengths 3..258 cheaply: short
 * lengths, which are common, get their own symbol; long ones share.
 */
static const short LENGTH_BASE[29] = {
    3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27,
    31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258
};
static const uint8_t LENGTH_EXTRA[29] = {
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2,
    2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0
};

/* Same idea for how far back to copy from: 30 symbols covering 1..32768. */
static const short DIST_BASE[30] = {
    1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129,
    193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097,
    6145, 8193, 12289, 16385, 24577
};
static const uint8_t DIST_EXTRA[30] = {
    0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6,
    6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13
};

/* ------------------------------------------------------------------------- */
/* Block decoding                                                             */
/* ------------------------------------------------------------------------- */

/* Decode one compressed block given its two Huffman tables. */
static void inflate_block(Puff *s, const Huffman *lencode, const Huffman *distcode)
{
    for (;;) {
        int sym = huffman_decode(s, lencode);
        if (s->err) return;

        if (sym < 256) {
            /* A literal byte. */
            if (s->out_pos >= s->out_cap) { s->err = 1; return; }
            s->out[s->out_pos++] = (uint8_t)sym;
            continue;
        }

        if (sym == 256) return;   /* end-of-block marker */

        /* Otherwise it is a back-reference. */
        sym -= 257;
        if (sym >= 29) { s->err = 1; return; }
        int len = LENGTH_BASE[sym] + (int)puff_bits(s, LENGTH_EXTRA[sym]);

        int dsym = huffman_decode(s, distcode);
        if (s->err || dsym < 0 || dsym >= 30) { s->err = 1; return; }
        size_t dist = (size_t)DIST_BASE[dsym] + puff_bits(s, DIST_EXTRA[dsym]);

        if (dist > s->out_pos) { s->err = 1; return; }   /* points before the start */
        if (s->out_pos + (size_t)len > s->out_cap) { s->err = 1; return; }

        /* Byte-at-a-time on purpose. The source range is allowed to overlap the
         * destination -- "distance 1, length 5" means "repeat the last byte
         * five times", and only a forward byte-by-byte copy produces that. A
         * memcpy would read bytes that have not been written yet. */
        size_t from = s->out_pos - dist;
        for (int i = 0; i < len; i++)
            s->out[s->out_pos++] = s->out[from++];
    }
}

/* The "fixed" block type uses a table every DEFLATE implementation agrees on,
 * defined by the code lengths below. Building it on demand rather than storing
 * a precomputed table keeps the two code paths identical. */
static void build_fixed_tables(Huffman *lencode, Huffman *distcode)
{
    uint8_t lengths[MAX_SYMBOLS];
    int i = 0;
    while (i < 144) lengths[i++] = 8;
    while (i < 256) lengths[i++] = 9;
    while (i < 280) lengths[i++] = 7;
    while (i < 288) lengths[i++] = 8;
    huffman_build(lencode, lengths, 288);

    /* All 30 distance codes are 5 bits in the fixed table. */
    for (i = 0; i < 30; i++) lengths[i] = 5;
    huffman_build(distcode, lengths, 30);
}

/*
 * A "dynamic" block carries its own Huffman tables. Those tables are just
 * lists of code lengths -- but the lists themselves are Huffman-coded, using a
 * third small table that is written out as plain 3-bit numbers. Three levels
 * of encoding is genuinely how the format works; it is not a mistake.
 */
static void read_dynamic_tables(Puff *s, Huffman *lencode, Huffman *distcode)
{
    int nlen  = (int)puff_bits(s, 5) + 257;   /* literal/length codes present */
    int ndist = (int)puff_bits(s, 5) + 1;     /* distance codes present */
    int ncode = (int)puff_bits(s, 4) + 4;     /* code-length codes present */
    if (s->err || nlen > 286 || ndist > 30) { s->err = 1; return; }

    /* The code-length alphabet's own lengths arrive in this shuffled order, so
     * that the entries most likely to be zero come last and can be omitted. */
    static const uint8_t ORDER[19] = {
        16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15
    };

    uint8_t cl_lengths[19];
    memset(cl_lengths, 0, sizeof cl_lengths);
    for (int i = 0; i < ncode; i++)
        cl_lengths[ORDER[i]] = (uint8_t)puff_bits(s, 3);
    if (s->err) return;

    Huffman clcode;
    huffman_build(&clcode, cl_lengths, 19);

    /* Now decode nlen + ndist code lengths using that table. Symbols 0..15 are
     * literal lengths; 16, 17 and 18 are run-length escapes, because long runs
     * of the same length (especially runs of zero, for unused symbols) are
     * extremely common. */
    uint8_t lengths[MAX_SYMBOLS + 30];
    int n = 0;
    while (n < nlen + ndist) {
        int sym = huffman_decode(s, &clcode);
        if (s->err) return;

        if (sym < 16) {
            lengths[n++] = (uint8_t)sym;
        } else {
            uint8_t value = 0;
            int repeat;
            if (sym == 16) {
                if (n == 0) { s->err = 1; return; }   /* nothing to repeat */
                value  = lengths[n - 1];
                repeat = 3 + (int)puff_bits(s, 2);
            } else if (sym == 17) {
                repeat = 3 + (int)puff_bits(s, 3);
            } else {
                repeat = 11 + (int)puff_bits(s, 7);
            }
            if (n + repeat > nlen + ndist) { s->err = 1; return; }
            while (repeat--) lengths[n++] = value;
        }
    }

    if (lengths[256] == 0) { s->err = 1; return; }   /* no end-of-block code */

    huffman_build(lencode, lengths, nlen);
    huffman_build(distcode, lengths + nlen, ndist);
}

/* ------------------------------------------------------------------------- */
/* Checksum                                                                   */
/* ------------------------------------------------------------------------- */

/*
 * Adler-32: two running sums, one of the bytes and one of that first sum, both
 * modulo 65521 (the largest prime below 2^16). Weaker than a CRC but much
 * cheaper, and quite good enough to catch a truncated or garbled file. zlib
 * appends it so the decompressor can prove it produced the original bytes.
 */
static uint32_t adler32(const uint8_t *data, size_t len)
{
    uint32_t a = 1, b = 0;
    for (size_t i = 0; i < len; i++) {
        a = (a + data[i]) % 65521u;
        b = (b + a) % 65521u;
    }
    return (b << 16) | a;
}

/* ------------------------------------------------------------------------- */
/* Entry point                                                                */
/* ------------------------------------------------------------------------- */

long zlib_inflate(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap)
{
    /* zlib header: 2 bytes, then the DEFLATE stream, then a 4-byte checksum. */
    if (in_len < 6) return -1;

    uint8_t cmf = in[0];
    uint8_t flg = in[1];
    if ((cmf & 0x0F) != 8) return -1;                 /* method must be DEFLATE */
    if (((cmf << 8) | flg) % 31u != 0) return -1;     /* header's own check value */
    if (flg & 0x20) return -1;                        /* preset dictionary: unused by PNG */

    Puff s;
    s.in      = in + 2;
    s.in_len  = in_len - 2;
    s.in_pos  = 0;
    s.bitbuf  = 0;
    s.bitcnt  = 0;
    s.out     = out;
    s.out_cap = out_cap;
    s.out_pos = 0;
    s.err     = 0;

    int last = 0;
    while (!last && !s.err) {
        last     = (int)puff_bits(&s, 1);
        int type = (int)puff_bits(&s, 2);

        if (type == 0) {
            /* Stored block: discard the rest of the current byte, then read a
             * 16-bit length followed by its bitwise complement as a check. */
            s.bitbuf = 0;
            s.bitcnt = 0;
            if (s.in_pos + 4 > s.in_len) { s.err = 1; break; }
            unsigned len  = (unsigned)s.in[s.in_pos] | ((unsigned)s.in[s.in_pos + 1] << 8);
            unsigned nlen = (unsigned)s.in[s.in_pos + 2] | ((unsigned)s.in[s.in_pos + 3] << 8);
            s.in_pos += 4;
            if ((len ^ 0xFFFFu) != nlen) { s.err = 1; break; }
            if (s.in_pos + len > s.in_len || s.out_pos + len > s.out_cap) { s.err = 1; break; }
            memcpy(s.out + s.out_pos, s.in + s.in_pos, len);
            s.in_pos  += len;
            s.out_pos += len;
        } else if (type == 1) {
            Huffman lencode, distcode;
            build_fixed_tables(&lencode, &distcode);
            inflate_block(&s, &lencode, &distcode);
        } else if (type == 2) {
            Huffman lencode, distcode;
            read_dynamic_tables(&s, &lencode, &distcode);
            if (!s.err) inflate_block(&s, &lencode, &distcode);
        } else {
            s.err = 1;   /* type 3 is reserved and never valid */
        }
    }

    if (s.err) return -1;

    /* The trailing checksum is byte-aligned and big-endian, and sits after
     * however many input bytes the bit reader actually consumed. Any bits
     * still sitting in the reservoir belong to padding, so count whole bytes
     * that were pulled out of the buffer but not yet used. */
    size_t consumed = s.in_pos - (size_t)(s.bitcnt / 8);
    if (consumed + 4 > s.in_len) return -1;

    const uint8_t *tail = s.in + consumed;
    uint32_t expected = ((uint32_t)tail[0] << 24) | ((uint32_t)tail[1] << 16) |
                        ((uint32_t)tail[2] << 8)  |  (uint32_t)tail[3];
    if (adler32(out, s.out_pos) != expected) return -1;

    return (long)s.out_pos;
}
