#include "rng.h"

#include "mathf.h"

#include <math.h>

void rng_init(Rng *r, uint64_t seed)
{
    /* xorshift has one degenerate state: zero XOR-shifted is still zero, so a
     * zero seed would make the generator emit zeros forever. Rather than
     * rejecting seed 0 and surprising the caller, we mix the seed through a
     * multiply-and-shift (this is SplitMix64's finaliser) which both avoids
     * zero and spreads a "boring" seed like 1 or 42 across all 64 bits. Seeds
     * that differ in one bit then produce completely unrelated streams. */
    uint64_t z = seed + 0x9E3779B97F4A7C15ull;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z = z ^ (z >> 31);
    r->state = z ? z : 0x9E3779B97F4A7C15ull;
}

uint64_t rng_next_u64(Rng *r)
{
    uint64_t x = r->state;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    r->state = x;
    return x * 0x2545F4914F6CDD1Dull;
}

float rng_uniform(Rng *r)
{
    /* A float has 24 bits of precision in its significand, so asking for more
     * than 24 random bits would be wasted work -- the extra bits could not
     * survive the conversion. Take the top 24 bits (the top bits of xorshift
     * are the best mixed) and divide by 2^24 to land in [0, 1). */
    uint32_t bits = (uint32_t)(rng_next_u64(r) >> 40);   /* 64 - 24 = 40 */
    return (float)bits / 16777216.0f;                    /* 2^24 */
}

float rng_range(Rng *r, float lo, float hi)
{
    return lo + (hi - lo) * rng_uniform(r);
}

float rng_normal(Rng *r, float mean, float stddev)
{
    /* Box-Muller transform: two independent uniform numbers can be turned into
     * a normally distributed one by treating them as a random radius and a
     * random angle in the plane. The radius comes from sqrt(-2 ln u), the
     * angle from 2*pi*v.
     *
     * The transform actually produces two independent normals (the cos and the
     * sin). We throw the second away. Caching it would save a call, but it
     * would also mean the generator returns different values depending on how
     * many times it was called before -- and that kind of hidden state is
     * exactly what makes a "reproducible" run stop being reproducible when you
     * add one extra call somewhere. Simplicity wins. */
    float u = rng_uniform(r);
    float v = rng_uniform(r);

    /* log(0) is -infinity, and rng_uniform can legitimately return exactly 0,
     * so nudge it onto the smallest value we are willing to take the log of. */
    if (u < 1e-7f) u = 1e-7f;

    return mean + stddev * sqrtf(-2.0f * logf(u)) * cosf(TWO_PI_F * v);
}
