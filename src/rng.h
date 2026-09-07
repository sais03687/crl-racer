#ifndef RNG_H
#define RNG_H

#include <stdint.h>

/*
 * ============================================================================
 * SEEDED PSEUDO-RANDOM NUMBER GENERATOR
 * ============================================================================
 *
 * Why not just use rand() from <stdlib.h>?
 *
 * Because rand() is global mutable state shared by the whole program, and its
 * algorithm is not specified by the C standard -- two compilers can give you
 * two different sequences from the same seed. For a simulator that is fatal.
 * The whole point of a simulator is that you can run it again and get exactly
 * the same thing, so that when the car crashes you can replay the crash, add
 * a printf, and see the identical crash a second time.
 *
 * So we carry our own generator, as an explicit struct the caller owns. Two
 * consequences worth understanding:
 *
 *   - The sequence depends only on the seed, and the algorithm is written out
 *     below, so the same seed gives the same run on any machine.
 *   - Because the state is passed in rather than global, two independent
 *     streams (say sensor noise and starting-position noise) can be kept from
 *     interfering with each other. If they shared one stream, turning sensor
 *     noise on would silently change the starting position too.
 *
 * The algorithm is xorshift64*: take a 64-bit number, XOR it with shifted
 * copies of itself a few times to scramble the bits, then multiply by a large
 * odd constant to scramble them again. It is a handful of instructions, has no
 * hidden tables, and its period (how long before the sequence repeats) is
 * 2^64 - 1, which is far longer than any run here will need. It is not
 * cryptographically secure -- and does not need to be.
 */
typedef struct Rng {
    uint64_t state;   /* never allowed to be 0; see rng_init */
} Rng;

void rng_init(Rng *r, uint64_t seed);

/* Raw 64 bits. Everything else is built on top of this. */
uint64_t rng_next_u64(Rng *r);

/* Uniform in [0, 1). */
float rng_uniform(Rng *r);

/* Uniform in [lo, hi). */
float rng_range(Rng *r, float lo, float hi);

/* Normal (Gaussian / bell curve) with the given mean and standard deviation.
 * Used for sensor noise, where errors cluster around the true value rather
 * than being spread evenly. */
float rng_normal(Rng *r, float mean, float stddev);

#endif /* RNG_H */
