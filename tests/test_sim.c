/*
 * ============================================================================
 * UNIT TESTS
 * ============================================================================
 *
 * Every test here checks something that can be verified against an answer
 * worked out independently -- by geometry, by arithmetic, or against a file
 * produced by a different implementation. A test that just records whatever
 * the code currently does would pass forever and tell you nothing.
 *
 * There is no test framework. Two macros and a counter are enough, and they
 * keep the whole suite readable in one sitting.
 *
 * Run with `make test`.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "dynamics.h"
#include "env.h"
#include "ftg.h"
#include "grid.h"
#include "image.h"
#include "inflate.h"
#include "lidar.h"
#include "mathf.h"
#include "rng.h"
#include "waypoints.h"

static int tests_run    = 0;
static int tests_failed = 0;

#define CHECK(cond, ...)                                                     \
    do {                                                                     \
        tests_run++;                                                         \
        if (!(cond)) {                                                       \
            tests_failed++;                                                  \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);                    \
            printf(__VA_ARGS__);                                             \
            printf("\n");                                                    \
        }                                                                    \
    } while (0)

/* Floats are compared with a tolerance, never with ==. Two calculations that
 * are algebraically identical routinely differ in the last bit or two, and a
 * test that demands exact equality fails for reasons that have nothing to do
 * with the code being wrong. The tolerance is stated per-check so each test
 * says how much slack it thinks is acceptable. */
#define CHECK_NEAR(actual, expected, tol, ...)                               \
    do {                                                                     \
        tests_run++;                                                         \
        float a_ = (actual), e_ = (expected);                                \
        if (!(fabsf(a_ - e_) <= (tol))) {                                    \
            tests_failed++;                                                  \
            printf("  FAIL %s:%d: got %.6f, expected %.6f (tol %.6f): ",     \
                   __FILE__, __LINE__, (double)a_, (double)e_, (double)(tol)); \
            printf(__VA_ARGS__);                                            \
            printf("\n");                                                    \
        }                                                                    \
    } while (0)

static void section(const char *name) { printf("%s\n", name); }

/* ========================================================================= */
/* A synthetic map                                                            */
/* ========================================================================= */

/*
 * Build a rectangular room in memory: solid one-cell border, everything inside
 * free. Tests against a hand-made map like this can state the right answer
 * exactly -- "a ray fired east from the middle must travel precisely 4.5 m" --
 * in a way that is impossible with a real racetrack.
 */
static void make_room(Grid *g, int w, int h, float res, float ox, float oy, float theta)
{
    g->cells        = (uint8_t *)malloc((size_t)w * (size_t)h);
    g->width        = w;
    g->height       = h;
    g->resolution   = res;
    g->origin_x     = ox;
    g->origin_y     = oy;
    g->origin_theta = theta;

    for (int row = 0; row < h; row++)
        for (int col = 0; col < w; col++)
            g->cells[row * w + col] =
                (row == 0 || col == 0 || row == h - 1 || col == w - 1)
                    ? CELL_OCCUPIED : CELL_FREE;
}

/* ========================================================================= */
/* Coordinate transforms -- the round trip the spec asks for                  */
/* ========================================================================= */

static void test_transform_roundtrip(void)
{
    section("coordinate transforms");

    /* Three maps with awkward numbers: a non-square grid, a resolution that is
     * not a round fraction, a negative origin, and (third case) a rotation.
     * Nice numbers hide sign errors, because 0 and 1 look the same forwards
     * and backwards. */
    struct { int w, h; float res, ox, oy, theta; } cases[] = {
        {  40,  25, 0.05000f,   0.0f,    0.0f,  0.0f },
        { 137,  91, 0.06367f, -39.038f, -49.237f, 0.0f },
        {  60,  33, 0.08000f,  12.500f,  -3.250f, 0.7f },
    };

    for (size_t k = 0; k < sizeof cases / sizeof cases[0]; k++) {
        Grid g;
        make_room(&g, cases[k].w, cases[k].h, cases[k].res,
                  cases[k].ox, cases[k].oy, cases[k].theta);

        /* --- grid -> world -> grid must be EXACT ---
         * Starting from a cell index and coming back is the strong version of
         * the round trip: grid_grid_to_world returns the cell's centre, which
         * is as far from every boundary as a point can be, so any rounding
         * slop shows up as a wrong cell rather than being absorbed. If this
         * passes for every cell, the two functions are genuine inverses and
         * the row-flip is right way up. */
        int mismatches = 0;
        for (int row = 0; row < g.height; row++) {
            for (int col = 0; col < g.width; col++) {
                float wx, wy;
                grid_grid_to_world(&g, col, row, &wx, &wy);

                int back_col, back_row;
                grid_world_to_grid(&g, wx, wy, &back_col, &back_row);

                if (back_col != col || back_row != row) mismatches++;
            }
        }
        CHECK(mismatches == 0,
              "case %zu: %d of %d cells did not survive grid->world->grid",
              k, mismatches, g.width * g.height);

        /* --- world -> grid -> world must land within half a cell ---
         * This direction cannot be exact and should not be: converting to a
         * cell index throws away where inside the cell the point was. What it
         * must not do is move the point into a DIFFERENT cell, so the error
         * has to stay under half a cell diagonal. */
        float worst = 0.0f;
        for (int i = 0; i < 400; i++) {
            /* Sweep across the map rather than sampling randomly, so a failure
             * is reproducible and reports the same point every run. */
            float fx = (float)(i % 20) / 19.0f;
            float fy = (float)(i / 20) / 19.0f;

            float wx, wy;
            grid_grid_to_world(&g,
                               (int)(fx * (float)(g.width  - 1)),
                               (int)(fy * (float)(g.height - 1)), &wx, &wy);

            /* Nudge off the exact centre, along the map's own axes, so this
             * tests interior points rather than repeating the centres the
             * previous loop already covered. */
            float c = cosf(g.origin_theta), s = sinf(g.origin_theta);
            wx += ( c * 0.31f - s * -0.24f) * g.resolution;
            wy += ( s * 0.31f + c * -0.24f) * g.resolution;

            int col, row;
            grid_world_to_grid(&g, wx, wy, &col, &row);

            float back_x, back_y;
            grid_grid_to_world(&g, col, row, &back_x, &back_y);

            float d = distf(wx, wy, back_x, back_y);
            if (d > worst) worst = d;
        }

        float half_diagonal = 0.70711f * g.resolution;
        CHECK(worst <= half_diagonal + 1e-5f,
              "case %zu: world->grid->world moved a point by %.6f m,"
              " more than half a cell diagonal (%.6f m)",
              k, (double)worst, (double)half_diagonal);

        grid_free(&g);
    }
}

/* ========================================================================= */
/* Grid queries                                                               */
/* ========================================================================= */

static void test_grid_queries(void)
{
    section("grid queries");

    Config cfg = config_default();
    Grid g;
    make_room(&g, 20, 10, 0.1f, -1.0f, -0.5f, 0.0f);

    CHECK(grid_in_bounds(&g, 0, 0), "cell (0,0) should be in bounds");
    CHECK(!grid_in_bounds(&g, 20, 0), "column 20 is past the right edge");
    CHECK(!grid_in_bounds(&g, 0, -1), "row -1 is above the map");

    /* Off the map must read as occupied, or a ray aimed off the edge would run
     * forever and a car could drive out of the world. */
    CHECK(grid_cell(&g, -5, -5) == CELL_OCCUPIED, "off-map should read occupied");
    CHECK(grid_cell(&g, 5, 5) == CELL_FREE, "the middle of the room is free");
    CHECK(grid_cell(&g, 0, 5) == CELL_OCCUPIED, "the left wall is occupied");

    /* The unknown-cell policy is a config switch, so check both settings. */
    g.cells[5 * 20 + 5] = CELL_UNKNOWN;
    cfg.treat_unknown_as_occupied = 1;
    CHECK(grid_is_blocked(&g, &cfg, 5, 5), "unknown must block when the policy says so");
    cfg.treat_unknown_as_occupied = 0;
    CHECK(!grid_is_blocked(&g, &cfg, 5, 5), "unknown must not block when the policy says so");

    grid_free(&g);
}

/* ========================================================================= */
/* Decompression                                                              */
/* ========================================================================= */

static void test_inflate(void)
{
    section("zlib inflate");

    /* All three fixtures were produced by the reference zlib implementation,
     * which is the point: they check this decoder against a different one
     * rather than against itself.
     *
     * zlib_inflate verifies the stream's own Adler-32 checksum internally, so
     * a successful return already means every output byte matched what the
     * compressor saw. The explicit content checks below are belt and braces. */

    /* --- a stored (uncompressed) block --- */
    static const uint8_t STORED[] = {
        0x78, 0x01, 0x01, 0x1A, 0x00, 0xE5, 0xFF, 0x73, 0x74, 0x6F, 0x72, 0x65,
        0x64, 0x20, 0x62, 0x6C, 0x6F, 0x63, 0x6B, 0x2C, 0x20, 0x75, 0x6E, 0x63,
        0x6F, 0x6D, 0x70, 0x72, 0x65, 0x73, 0x73, 0x65, 0x64, 0x87, 0x49, 0x0A,
        0x21
    };
    uint8_t out[512];
    long n = zlib_inflate(STORED, sizeof STORED, out, sizeof out);
    CHECK(n == 26, "stored block: got %ld bytes, expected 26", n);
    if (n == 26) CHECK(memcmp(out, "stored block, uncompressed", 26) == 0,
                       "stored block content mismatch");

    /* --- a fixed-Huffman block, with back-references ---
     * The source is one sentence repeated six times, so almost all of it is
     * encoded as "copy what you already emitted" -- which is exactly the code
     * path where an overlapping copy done with memcpy would go wrong. */
    static const uint8_t FIXED[] = {
        0x78, 0xDA, 0x2B, 0xC9, 0x48, 0x55, 0x28, 0x2C, 0xCD, 0x4C, 0xCE, 0x56,
        0x48, 0x2A, 0xCA, 0x2F, 0xCF, 0x53, 0x48, 0xCB, 0xAF, 0x50, 0xC8, 0x2A,
        0xCD, 0x2D, 0x28, 0x56, 0xC8, 0x2F, 0x4B, 0x2D, 0x52, 0x28, 0x01, 0x4A,
        0xE7, 0x24, 0x56, 0x55, 0x2A, 0xA4, 0xE4, 0xA7, 0xEB, 0x81, 0x79, 0xC3,
        0x5A, 0x31, 0x00, 0xF7, 0xE0, 0x61, 0xAB
    };
    const char *SENTENCE = "the quick brown fox jumps over the lazy dog. ";
    n = zlib_inflate(FIXED, sizeof FIXED, out, sizeof out);
    CHECK(n == 270, "fixed-Huffman block: got %ld bytes, expected 270", n);
    if (n == 270) {
        int ok = 1;
        for (int rep = 0; rep < 6; rep++)
            if (memcmp(out + rep * 45, SENTENCE, 45) != 0) ok = 0;
        CHECK(ok, "fixed-Huffman block content mismatch");
    }

    /* --- a dynamic-Huffman block ---
     * 300 bytes of high-entropy data, which forces the compressor to build and
     * transmit a table rather than use the standard one. This is the path the
     * real map PNGs take, and the only one that exercises the code-length
     * alphabet and its run-length escapes. */
    static const uint8_t DYNAMIC[] = {
        0x78, 0xDA, 0x0D, 0x8E, 0x31, 0x12, 0xC0, 0x20, 0x08, 0xC0, 0xE4, 0xA9,
        0x0A, 0x08, 0x2A, 0x28, 0xFF, 0x1F, 0xA0, 0xCD, 0x98, 0x21, 0x97, 0xC3,
        0x3E, 0x10, 0xC8, 0x6A, 0x34, 0x19, 0x78, 0x2F, 0x2A, 0xC2, 0x1D, 0x45,
        0x5A, 0xA3, 0xCA, 0x87, 0x0E, 0xE0, 0x7D, 0x19, 0xA8, 0x36, 0x4C, 0xAA,
        0x12, 0x23, 0xC0, 0x1A, 0x29, 0x01, 0xF7, 0xBC, 0x7A, 0xB6, 0x75, 0x2A,
        0xD6, 0x6E, 0x71, 0xDE, 0x4E, 0xA4, 0x76, 0xE7, 0xE1, 0xB8, 0x7F, 0xB2,
        0xCE, 0xB1, 0x8C, 0x7A, 0x88, 0x2B, 0x70, 0xEC, 0x7A, 0xDB, 0xAD, 0x3F,
        0x9B, 0x49, 0x31, 0x64, 0xB3, 0xBA, 0x07, 0xCE, 0xE7, 0xB0, 0xF8, 0xC2,
        0xBA, 0xE6, 0xCA, 0x38, 0x59, 0xB5, 0x47, 0xCD, 0xB5, 0xFB, 0x2F, 0x2D,
        0xEB, 0x70, 0xCB, 0xF1, 0xC0, 0x7F, 0x28, 0x7C, 0x08, 0xCA, 0x9B, 0x74,
        0x72, 0x50, 0xAF, 0x7F, 0xC9, 0xB2, 0xA3, 0xA4, 0xF3, 0xB2, 0xB4, 0x20,
        0x8A, 0x17, 0xB1, 0x91, 0xE9, 0xAC, 0x98, 0xAD, 0x4B, 0x33, 0x86, 0xDE,
        0x36, 0xAE, 0x66, 0xD3, 0x14, 0xA0, 0x1D, 0x4D, 0x51, 0x57, 0x69, 0x61,
        0xBD, 0xAF, 0x58, 0x92, 0xF6, 0xCC, 0x50, 0x49, 0x43, 0x8E, 0x44, 0x66,
        0x0F, 0x43, 0x72, 0x89, 0x79, 0x0F, 0xFA, 0x73, 0x9C, 0x93, 0x3B, 0xD7,
        0xE3, 0xCC, 0xF8, 0x5B, 0xC0, 0xFD, 0x03, 0x1D, 0xC5, 0x83, 0x0C
    };
    n = zlib_inflate(DYNAMIC, sizeof DYNAMIC, out, sizeof out);
    CHECK(n == 300, "dynamic-Huffman block: got %ld bytes, expected 300", n);
    if (n == 300) {
        static const uint8_t HEAD[8] = { 0x6B, 0x65, 0x6D, 0x62, 0x63, 0x01, 0x64, 0x6C };
        CHECK(memcmp(out, HEAD, 8) == 0, "dynamic-Huffman block content mismatch");
    }

    /* --- corruption must be detected, not ignored ---
     * A decompressor that returns plausible-looking garbage for a damaged file
     * is worse than one that fails, because the failure then surfaces as an
     * inexplicably wrong map. */
    uint8_t damaged[sizeof FIXED];
    memcpy(damaged, FIXED, sizeof FIXED);
    damaged[20] ^= 0xFF;
    n = zlib_inflate(damaged, sizeof damaged, out, sizeof out);
    CHECK(n == -1, "a corrupted stream must fail, got %ld bytes", n);

    /* An output buffer that is too small must fail rather than overrun. */
    n = zlib_inflate(FIXED, sizeof FIXED, out, 16);
    CHECK(n == -1, "an undersized output buffer must fail, got %ld", n);
}

/* ========================================================================= */
/* Image loading                                                              */
/* ========================================================================= */

static void test_pgm(void)
{
    section("PGM loading");

    /* A four-pixel image written both ways. Round-tripping through a real file
     * is the only way to test the header parsing, the comment skipping and the
     * binary/text split for real. */
    const char *ascii_path  = "test_ascii.pgm";
    const char *binary_path = "test_binary.pgm";

    FILE *f = fopen(ascii_path, "wb");
    CHECK(f != NULL, "could not create %s", ascii_path);
    if (f) {
        /* Comments and awkward whitespace on purpose: real map files have
         * them, and a parser that only handles the tidy case will fail on the
         * first file it meets in the wild. */
        fprintf(f, "P2\n# written by the test\n2  2\n255\n0 64\n192 255\n");
        fclose(f);
    }

    static const uint8_t BINARY_BODY[4] = { 0, 64, 192, 255 };
    f = fopen(binary_path, "wb");
    CHECK(f != NULL, "could not create %s", binary_path);
    if (f) {
        fprintf(f, "P5\n2 2\n255\n");
        fwrite(BINARY_BODY, 1, 4, f);
        fclose(f);
    }

    char err[256];
    GrayImage a, b;

    CHECK(image_load(ascii_path, &a, err, sizeof err), "P2 load failed: %s", err);
    CHECK(a.width == 2 && a.height == 2, "P2 dimensions wrong: %dx%d", a.width, a.height);
    if (a.pixels)
        CHECK(memcmp(a.pixels, BINARY_BODY, 4) == 0, "P2 pixel values wrong");

    CHECK(image_load(binary_path, &b, err, sizeof err), "P5 load failed: %s", err);
    CHECK(b.width == 2 && b.height == 2, "P5 dimensions wrong: %dx%d", b.width, b.height);
    if (b.pixels)
        CHECK(memcmp(b.pixels, BINARY_BODY, 4) == 0, "P5 pixel values wrong");

    image_free(&a);
    image_free(&b);

    /* A file that is not an image at all must be reported, not guessed at. */
    const char *junk_path = "test_junk.bin";
    f = fopen(junk_path, "wb");
    if (f) { fprintf(f, "this is not an image at all"); fclose(f); }
    GrayImage junk;
    CHECK(!image_load(junk_path, &junk, err, sizeof err),
          "a text file must not load as an image");

    remove(ascii_path);
    remove(binary_path);
    remove(junk_path);
}

/* ========================================================================= */
/* PRNG                                                                       */
/* ========================================================================= */

static void test_rng(void)
{
    section("seeded PRNG");

    /* Reproducibility is the entire reason this generator exists, so it is the
     * first thing to check: same seed, same sequence. */
    Rng a, b;
    rng_init(&a, 12345);
    rng_init(&b, 12345);

    int identical = 1;
    for (int i = 0; i < 1000; i++)
        if (rng_next_u64(&a) != rng_next_u64(&b)) identical = 0;
    CHECK(identical, "the same seed must give the same sequence");

    /* Different seeds must not. A generator whose seed does nothing would pass
     * the test above perfectly. */
    Rng c;
    rng_init(&a, 1);
    rng_init(&c, 2);
    CHECK(rng_next_u64(&a) != rng_next_u64(&c), "different seeds must diverge");

    /* Seed 0 is the state xorshift cannot escape from; rng_init has to mix it
     * away. Without that guard this test emits zeros forever. */
    Rng z;
    rng_init(&z, 0);
    int all_zero = 1;
    for (int i = 0; i < 10; i++)
        if (rng_next_u64(&z) != 0) all_zero = 0;
    CHECK(!all_zero, "seed 0 must not produce a stuck generator");

    /* Range and rough shape. Not a statistical test -- just enough to catch a
     * generator that is off by a factor or stuck in a corner. */
    Rng u;
    rng_init(&u, 99);
    float lo = 2.0f, hi = -1.0f, sum = 0.0f;
    for (int i = 0; i < 20000; i++) {
        float v = rng_uniform(&u);
        if (v < lo) lo = v;
        if (v > hi) hi = v;
        sum += v;
    }
    CHECK(lo >= 0.0f && hi < 1.0f, "uniform out of [0,1): [%.6f, %.6f]", (double)lo, (double)hi);
    CHECK_NEAR(sum / 20000.0f, 0.5f, 0.02f, "uniform mean is off");

    /* The Gaussian's mean and spread, to two decimal places over 20k samples. */
    float gsum = 0.0f, gsq = 0.0f;
    for (int i = 0; i < 20000; i++) {
        float v = rng_normal(&u, 3.0f, 2.0f);
        gsum += v;
        gsq  += (v - 3.0f) * (v - 3.0f);
    }
    CHECK_NEAR(gsum / 20000.0f, 3.0f, 0.06f, "normal mean is off");
    CHECK_NEAR(sqrtf(gsq / 20000.0f), 2.0f, 0.06f, "normal stddev is off");
}

/* ========================================================================= */
/* Vehicle model                                                              */
/* ========================================================================= */

static void test_dynamics(void)
{
    section("kinematic bicycle model");

    Config cfg = config_default();
    cfg.max_accel_mps2 = 1000.0f;   /* reach the target speed immediately */

    /* --- Straight ahead: distance must equal speed x time --- */
    CarState car;
    car_reset(&car, 0.0f, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 200; i++)
        car_step(&car, &cfg, 0.0f, 3.0f, 0.01f);

    /* 200 steps of 0.01 s at 3 m/s. The first step is where the speed jumps
     * from 0 to 3, and semi-implicit Euler applies the new speed straight
     * away, so the full 2.0 s of travel happens: 6.0 m. */
    CHECK_NEAR(car.x, 6.0f, 0.01f, "straight-line distance");
    CHECK_NEAR(car.y, 0.0f, 1e-4f, "a car with no steering must not drift sideways");
    CHECK_NEAR(car.heading, 0.0f, 1e-5f, "heading must not change with no steering");

    /* --- Turning radius must match the geometry ---
     * The model claims the car traces a circle of radius L / tan(delta). Drive
     * a quarter turn and check the car really is that far from where it
     * started, in the right direction. This is the test that catches a missing
     * wheelbase division or a tan/sin mix-up. */
    float delta    = 0.3f;
    float expected = cfg.wheelbase_m / tanf(delta);

    car_reset(&car, 0.0f, 0.0f, 0.0f, 0.0f);
    float dt = 0.0005f;   /* small, so Euler's own error stays well under the tolerance */
    while (car.heading < 1.5707963f)
        car_step(&car, &cfg, delta, 2.0f, dt);

    /* After exactly 90 degrees of a left turn starting at the origin heading
     * east, the centre of the circle is at (0, R), so the car is at (R, R). */
    CHECK_NEAR(car.x, expected, 0.02f, "quarter-turn x should be the turn radius");
    CHECK_NEAR(car.y, expected, 0.02f, "quarter-turn y should be the turn radius");

    /* --- Acceleration limit --- */
    cfg.max_accel_mps2 = 2.0f;
    car_reset(&car, 0.0f, 0.0f, 0.0f, 0.0f);
    car_step(&car, &cfg, 0.0f, 10.0f, 0.1f);
    CHECK_NEAR(car.speed, 0.2f, 1e-5f, "speed must rise by at most accel*dt");

    /* Braking is limited by the same number in the other direction. */
    car_reset(&car, 0.0f, 0.0f, 0.0f, 5.0f);
    car_step(&car, &cfg, 0.0f, 0.0f, 0.1f);
    CHECK_NEAR(car.speed, 4.8f, 1e-5f, "speed must fall by at most accel*dt");

    /* --- Commands out of range must be clamped, not obeyed --- */
    cfg.max_accel_mps2 = 1000.0f;
    car_reset(&car, 0.0f, 0.0f, 0.0f, 0.0f);
    car_step(&car, &cfg, 99.0f, 999.0f, 0.01f);
    CHECK(car.speed <= cfg.max_speed_mps + 1e-4f,
          "speed %.3f exceeded the cap %.3f", (double)car.speed, (double)cfg.max_speed_mps);

    /* An absurd steering command must produce no more yaw than full lock. */
    CarState sane;
    car_reset(&sane, 0.0f, 0.0f, 0.0f, 0.0f);
    car_step(&sane, &cfg, cfg.max_steer_rad, 999.0f, 0.01f);
    CHECK_NEAR(car.heading, sane.heading, 1e-5f,
               "over-range steering must behave exactly like full lock");

    /* --- The footprint must be the right size and follow the heading --- */
    float fx[4], fy[4];
    car_reset(&car, 0.0f, 0.0f, 0.0f, 0.0f);
    car_footprint(&car, &cfg, fx, fy);
    CHECK_NEAR(distf(fx[0], fy[0], fx[1], fy[1]), cfg.car_width_m, 1e-4f,
               "footprint width");
    CHECK_NEAR(distf(fx[1], fy[1], fx[2], fy[2]), cfg.car_length_m, 1e-4f,
               "footprint length");

    car_reset(&car, 0.0f, 0.0f, 1.5707963f, 0.0f);   /* pointing north */
    car_footprint(&car, &cfg, fx, fy);
    CHECK(fy[0] > 0.0f && fy[1] > 0.0f,
          "with the car facing north its front corners must be north of the axle");
}

/* ========================================================================= */
/* LiDAR                                                                      */
/* ========================================================================= */

static void test_lidar(void)
{
    section("LiDAR ray casting");

    Config cfg = config_default();
    cfg.lidar_max_range_m = 30.0f;

    /* A 10 x 10 m room with 0.1 m cells and its origin at (0,0). The walls are
     * the outermost ring of cells, so from the centre at (5,5) the inner
     * surface of each wall is 4.9 m away. Every expected number below comes
     * from that geometry, not from running the code. */
    Grid g;
    make_room(&g, 100, 100, 0.1f, 0.0f, 0.0f, 0.0f);

    CHECK_NEAR(lidar_cast_ray(&g, &cfg, 5.0f, 5.0f, 0.0f), 4.9f, 0.001f, "east wall");
    CHECK_NEAR(lidar_cast_ray(&g, &cfg, 5.0f, 5.0f, 1.5707963f), 4.9f, 0.001f, "north wall");
    CHECK_NEAR(lidar_cast_ray(&g, &cfg, 5.0f, 5.0f, 3.1415927f), 4.9f, 0.001f, "west wall");
    CHECK_NEAR(lidar_cast_ray(&g, &cfg, 5.0f, 5.0f, -1.5707963f), 4.9f, 0.001f, "south wall");

    /* A 45 degree ray from the centre reaches the corner, which is sqrt(2)
     * times further. Diagonal rays are where a DDA that steps the wrong axis
     * goes wrong, so this is the most informative single check here. */
    CHECK_NEAR(lidar_cast_ray(&g, &cfg, 5.0f, 5.0f, 0.7853982f),
               4.9f * 1.41421356f, 0.02f, "diagonal to the corner");

    /* Off centre, so the two walls are at different distances -- this catches
     * a caster that ignores where inside the starting cell the ray began. */
    CHECK_NEAR(lidar_cast_ray(&g, &cfg, 2.0f, 5.0f, 0.0f), 7.9f, 0.001f, "east from x=2");
    CHECK_NEAR(lidar_cast_ray(&g, &cfg, 2.0f, 5.0f, 3.1415927f), 1.9f, 0.001f, "west from x=2");

    /* Beyond the sensor's reach, a real wall must not be reported. */
    cfg.lidar_max_range_m = 2.0f;
    CHECK_NEAR(lidar_cast_ray(&g, &cfg, 5.0f, 5.0f, 0.0f), 2.0f, 0.001f,
               "a wall past max range must read as max range");
    cfg.lidar_max_range_m = 30.0f;

    /* Starting inside a wall reads 0, not max range. */
    CHECK_NEAR(lidar_cast_ray(&g, &cfg, 0.05f, 5.0f, 0.0f), 0.0f, 1e-6f,
               "a ray starting in a wall reads zero");

    /* A full scan: correct count, every reading inside the sensor's range, and
     * the middle ray agreeing with a single cast in the same direction. */
    float *scan = (float *)malloc((size_t)cfg.ray_count * sizeof(float));
    lidar_scan(&g, &cfg, NULL, 5.0f, 5.0f, 0.0f, scan);

    int in_range = 1;
    for (int i = 0; i < cfg.ray_count; i++)
        if (!(scan[i] >= 0.0f && scan[i] <= cfg.lidar_max_range_m)) in_range = 0;
    CHECK(in_range, "every reading must lie in [0, max_range]");

    CHECK_NEAR(scan[cfg.ray_count / 2],
               lidar_cast_ray(&g, &cfg, 5.0f, 5.0f, 0.0f), 0.02f,
               "the middle ray of a scan must look straight ahead");

    /* The scan must be symmetric about its middle in a symmetric room. This
     * catches an off-by-one in the ray bearings that would quietly bias every
     * steering decision to one side. */
    CHECK_NEAR(scan[0], scan[cfg.ray_count - 1], 0.02f,
               "first and last rays should match in a symmetric room");

    /* An empty map: with nothing to hit, every reading saturates. */
    Grid empty;
    make_room(&empty, 100, 100, 0.1f, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 100 * 100; i++) empty.cells[i] = CELL_FREE;
    cfg.lidar_max_range_m = 3.0f;
    CHECK_NEAR(lidar_cast_ray(&empty, &cfg, 5.0f, 5.0f, 0.3f), 3.0f, 1e-4f,
               "nothing to hit means max range");
    grid_free(&empty);

    free(scan);
    grid_free(&g);
}

/* ========================================================================= */
/* Waypoints                                                                  */
/* ========================================================================= */

static void test_waypoints(void)
{
    section("centerline waypoints");

    /* A 40-point circle of radius 5. Its circumference is a number we know
     * exactly (2*pi*5 = 31.4159), so the arc-length bookkeeping can be checked
     * against real geometry rather than against itself. The polygon through 40
     * points is very slightly shorter than the true circle, which is why the
     * tolerance below is 0.1 m and not 0.001. */
    const char *path = "test_circle.csv";
    FILE *f = fopen(path, "wb");
    CHECK(f != NULL, "could not create %s", path);
    if (f) {
        fprintf(f, "# x_m, y_m, w_tr_right_m, w_tr_left_m\n");
        for (int i = 0; i < 40; i++) {
            float a = 6.2831853f * (float)i / 40.0f;
            fprintf(f, "%.6f, %.6f, 1.1, 1.1\n",
                    (double)(5.0f * cosf(a)), (double)(5.0f * sinf(a)));
        }
        fclose(f);
    }

    char err[256];
    Waypoints w;
    CHECK(waypoints_load(path, &w, err, sizeof err), "circle load failed: %s", err);
    CHECK(w.count == 40, "expected 40 waypoints, got %d", w.count);
    CHECK_NEAR(w.total_length, 31.4159f, 0.1f, "circumference of a radius-5 circle");
    CHECK_NEAR(w.s[0], 0.0f, 1e-6f, "arc length must start at zero");

    /* Arc length must increase monotonically along the list. */
    int monotonic = 1;
    for (int i = 1; i < w.count; i++)
        if (w.s[i] <= w.s[i - 1]) monotonic = 0;
    CHECK(monotonic, "cumulative arc length must strictly increase");

    /* Nearest point: (5,0) is waypoint 0 by construction. */
    CHECK(waypoints_nearest(&w, 5.0f, 0.0f, -1, 0) == 0, "nearest to (5,0) is point 0");

    /* A quarter of the way round is at (0,5), which should be a quarter of the
     * circumference along. */
    int q = waypoints_nearest(&w, 0.0f, 5.0f, -1, 0);
    CHECK_NEAR(waypoints_arclength(&w, q, 0.0f, 5.0f), 31.4159f / 4.0f, 0.1f,
               "arc length a quarter of the way round");

    /* The projection must refine BELOW the waypoint spacing -- that is the
     * whole reason it exists. Halfway between points 0 and 1 should report
     * about half a segment, not 0 and not a whole segment. */
    float mid_x = 0.5f * (w.x[0] + w.x[1]);
    float mid_y = 0.5f * (w.y[0] + w.y[1]);
    int   mid_i = waypoints_nearest(&w, mid_x, mid_y, -1, 0);
    float mid_s = waypoints_arclength(&w, mid_i, mid_x, mid_y);
    CHECK(mid_s > 0.2f * w.s[1] && mid_s < 0.8f * w.s[1],
          "a point between waypoints 0 and 1 gave arc length %.4f, expected near %.4f",
          (double)mid_s, (double)(0.5f * w.s[1]));

    /* --- The wrap-around, which is the subtle part ---
     * Going from just before the line to just after it is a small step
     * forward, not a lap backwards. */
    CHECK_NEAR(waypoints_forward_delta(&w, w.total_length - 0.1f, 0.1f), 0.2f, 1e-3f,
               "crossing the start line must read as small forward progress");
    CHECK_NEAR(waypoints_forward_delta(&w, 0.1f, w.total_length - 0.1f), -0.2f, 1e-3f,
               "crossing it backwards must read as small negative progress");
    CHECK_NEAR(waypoints_forward_delta(&w, 1.0f, 3.0f), 2.0f, 1e-4f,
               "an ordinary step in the middle of the lap");

    /* Heading at (5,0) on a counter-clockwise circle points north. */
    CHECK_NEAR(waypoints_heading(&w, 0), 1.5707963f, 0.1f,
               "heading at the start of a counter-clockwise circle");

    /* The windowed search must not jump to the far side of the track. Ask for
     * the point nearest (5,0) -- genuinely waypoint 0 -- but from a previous
     * index halfway round, with a small window. The right answer is "somewhere
     * near where I was", because that is what the window means. */
    int windowed = waypoints_nearest(&w, 5.0f, 0.0f, 20, 3);
    CHECK(windowed >= 17 && windowed <= 23,
          "a windowed search returned %d, outside the window around 20", windowed);

    waypoints_free(&w);
    remove(path);
}

/* ========================================================================= */
/* Follow-The-Gap                                                             */
/* ========================================================================= */

static void test_ftg(void)
{
    section("Follow-The-Gap");

    /* A small scan is easier to reason about than 1080 beams, and the
     * algorithm does not care how many there are. */
    Config cfg = config_default();
    cfg.ray_count           = 101;
    cfg.lidar_fov_rad       = 3.1415927f;   /* 180 degrees, so beam 50 is straight ahead */
    cfg.ftg_smooth_window   = 0;            /* off, so the input is exactly what we set */
    cfg.ftg_bubble_radius_m = 0.0f;         /* off, so the gap is exactly what we set */
    cfg.ftg_gap_threshold_m = 2.0f;
    cfg.ftg_range_clip_m    = 10.0f;

    FollowTheGap d;
    CHECK(ftg_init(&d, &cfg), "driver init failed");

    float scan[101];
    Observation obs;
    obs.scan       = scan;
    obs.scan_count = 101;
    obs.speed_mps  = 0.0f;

    /* --- Everything open: aim straight ahead, go flat out --- */
    for (int i = 0; i < 101; i++) scan[i] = 8.0f;
    Action a = ftg_plan(&d, &cfg, &obs);
    CHECK(d.target_ray == 50, "with a uniform view the target should be the middle ray, got %d",
          d.target_ray);
    CHECK_NEAR(a.steering_rad, 0.0f, 1e-5f, "uniform view should steer straight");
    CHECK_NEAR(a.target_speed_mps, cfg.max_speed_mps, 1e-4f,
               "driving straight should request full speed");

    /* --- One clear gap on the left: aim into its middle ---
     * Beams 70..90 are open, everything else is wall. The midpoint is beam 80,
     * which on this 180 degree sweep is 0.9425 rad to the left. */
    for (int i = 0; i < 101; i++) scan[i] = 0.5f;
    for (int i = 70; i <= 90; i++) scan[i] = 9.0f;
    a = ftg_plan(&d, &cfg, &obs);
    CHECK(d.target_ray == 80, "expected the midpoint of beams 70..90, got %d", d.target_ray);
    CHECK(a.steering_rad > 0.0f, "a gap on the left must produce a left turn");
    CHECK(a.target_speed_mps < cfg.max_speed_mps,
          "turning should reduce the requested speed");

    /* Mirrored: the same gap on the right must give the mirrored steering. */
    for (int i = 0; i < 101; i++) scan[i] = 0.5f;
    for (int i = 10; i <= 30; i++) scan[i] = 9.0f;
    Action mirrored = ftg_plan(&d, &cfg, &obs);
    CHECK(d.target_ray == 20, "expected the midpoint of beams 10..30, got %d", d.target_ray);
    CHECK_NEAR(mirrored.steering_rad, -a.steering_rad, 1e-5f,
               "a mirrored scan must give mirrored steering");

    /* --- Two gaps: the LONGER one wins, even if the shorter one is nearer
     * straight ahead. This is the property the algorithm is named for. --- */
    for (int i = 0; i < 101; i++) scan[i] = 0.5f;
    for (int i = 45; i <= 50; i++) scan[i] = 9.0f;   /* narrow, dead ahead */
    for (int i = 75; i <= 95; i++) scan[i] = 9.0f;   /* wide, off to the left */
    ftg_plan(&d, &cfg, &obs);
    CHECK(d.gap_start == 75 && d.gap_end == 95,
          "the wider gap should win, got beams %d..%d", d.gap_start, d.gap_end);

    /* --- A gap running to the very edge of the scan must still be found.
     * This is the off-by-one that a run-detecting loop classically misses. --- */
    for (int i = 0; i < 101; i++) scan[i] = 0.5f;
    for (int i = 80; i <= 100; i++) scan[i] = 9.0f;
    ftg_plan(&d, &cfg, &obs);
    CHECK(d.gap_end == 100, "a gap reaching the last beam must be detected, got end %d",
          d.gap_end);

    /* --- Boxed in: must still produce a legal action rather than give up --- */
    for (int i = 0; i < 101; i++) scan[i] = 0.3f;
    scan[65] = 1.2f;   /* the least bad direction, still under the threshold */
    a = ftg_plan(&d, &cfg, &obs);
    CHECK(d.target_ray == 65, "with no gap, aim at the most open beam, got %d", d.target_ray);
    CHECK(a.target_speed_mps >= cfg.min_speed_mps,
          "the car must keep moving; a stopped car cannot steer");

    /* --- The safety bubble must actually erase beams --- */
    cfg.ftg_bubble_radius_m = 0.5f;
    for (int i = 0; i < 101; i++) scan[i] = 9.0f;
    scan[30] = 0.4f;                  /* something very close on the right */
    ftg_plan(&d, &cfg, &obs);
    CHECK(d.closest_ray == 30, "closest beam should be 30, got %d", d.closest_ray);
    CHECK(d.target_ray > 45,
          "the target should be pushed away from the near obstacle, got %d", d.target_ray);

    /* Every action must be inside the config's limits, whatever the scan. */
    CHECK(fabsf(a.steering_rad) <= cfg.max_steer_rad + 1e-5f, "steering out of range");
    CHECK(a.target_speed_mps <= cfg.max_speed_mps + 1e-5f, "speed out of range");

    ftg_free(&d);
}

/* ========================================================================= */
/* Config                                                                     */
/* ========================================================================= */

static void test_config(void)
{
    section("config");

    Config cfg = config_default();

    CHECK(config_steps_per_control(&cfg) ==
          (int)(1.0f / cfg.control_hz / cfg.physics_dt_s + 0.5f),
          "substep count should be the control period over the physics step");
    CHECK(config_steps_per_control(&cfg) >= 1, "there must be at least one substep");

    /* The rays must span exactly the field of view, ends included. */
    CHECK_NEAR(config_ray_bearing(&cfg, 0), -cfg.lidar_fov_rad * 0.5f, 1e-5f,
               "the first ray sits at one edge of the field of view");
    CHECK_NEAR(config_ray_bearing(&cfg, cfg.ray_count - 1), cfg.lidar_fov_rad * 0.5f, 1e-5f,
               "the last ray sits at the other edge");
    CHECK_NEAR(config_ray_bearing(&cfg, cfg.ray_count / 2), 0.0f, 0.01f,
               "the middle ray looks straight ahead");

    /* The physics step has to be small enough that the car cannot cross a
     * whole map cell in one step -- the tunnelling argument from config.h.
     * 0.06 m is the resolution of the published maps. */
    float travel = cfg.max_speed_mps * cfg.physics_dt_s;
    CHECK(travel < 0.06f,
          "at %.1f m/s the car moves %.4f m per physics step, which is more than "
          "one map cell; it could pass through a thin wall",
          (double)cfg.max_speed_mps, (double)travel);
}

/* ========================================================================= */

int main(void)
{
    printf("running unit tests\n\n");

    test_config();
    test_transform_roundtrip();
    test_grid_queries();
    test_inflate();
    test_pgm();
    test_rng();
    test_dynamics();
    test_lidar();
    test_waypoints();
    test_ftg();

    printf("\n%d checks, %d failed\n", tests_run, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
