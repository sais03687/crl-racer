/* grid_transforms.c
 *
 * Coordinate transforms between world space (metres, y up) and grid space
 * (cell indices, rows counting down from the top of the image).
 *
 * Build and run:
 *     gcc -std=c11 -Wall -Wextra -g -fsanitize=address,undefined \
 *         grid_transforms.c -lm -o grid_transforms && ./grid_transforms
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* ------------------------------------------------------------------------ */
/* The grid                                                                  */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint8_t *cells;             /* row-major, cells[row * width + col]       */
    int      width, height;     /* in cells                                  */
    float    resolution;        /* metres per cell                           */
    float    origin_x, origin_y;/* world coords of the grid's BOTTOM-LEFT    */
} Grid;

/* ------------------------------------------------------------------------ */
/* World -> grid                                                             */
/* ------------------------------------------------------------------------ */

/* The fractional version. The raycaster needs to know where INSIDE a cell a
 * ray begins, so the un-floored value is the primitive and the integer
 * version is derived from it -- not the other way around. Deriving it the
 * other way would mean writing the same arithmetic twice, and the y-flip is
 * exactly the kind of thing you only want to get right once. */
void grid_world_to_cellf(const Grid *g, float wx, float wy,
                         float *out_col, float *out_row)
{
    /* x is the straightforward direction: distance from the origin, converted
     * from metres into a count of cells. */
    *out_col = (wx - g->origin_x) / g->resolution;

    /* y is not symmetric with x, and this is the whole difficulty.
     *
     * The expression below computes cells-up-from-the-bottom, exactly as the
     * x line computes cells-right-of-the-left-edge. But the image stores its
     * TOP row at index 0, so an index counting downward is what we need.
     * Subtracting from (height - 1) performs that flip.
     *
     * Get this wrong and nothing crashes: the track still loads and still
     * looks like a track, just mirrored vertically, and the car drives into
     * walls that appear to be open space. */
    float cells_up = (wy - g->origin_y) / g->resolution;
    *out_row = (float)g->height - cells_up;
}

/* A note on that constant, because it is an easy off-by-one and I got it
 * wrong on the first pass -- the test below is what caught it.
 *
 * For INTEGER indices the flip is (height - 1) - k: the bottom cell, k = 0,
 * becomes the last row, height - 1. That form is correct and it is tempting
 * to reuse it here.
 *
 * But this function works in CONTINUOUS space, and there the flip is
 * height - cells_up. Check it: a point half a cell up from the bottom has
 * cells_up = 0.5, and height - 0.5 = 9.5 in a 10-row grid, which floors to
 * row 9 -- the bottom row. Using (height - 1) instead gives 8.5, which
 * floors to 8, and every row in the map is off by one.
 *
 * The general rule: flip first in continuous space, floor afterwards. */

/* The integer version: the same transform, discretised. A point 3.7 cells
 * from the edge lies inside cell 3, which is what floor gives. Note that
 * floorf is not the same as a cast to int for negative values -- a cast
 * truncates toward zero, so -0.5 would become cell 0 instead of cell -1, and
 * points just outside the map's left edge would silently alias to points
 * inside it. */
void grid_world_to_cell(const Grid *g, float wx, float wy,
                        int *out_col, int *out_row)
{
    float fcol, frow;
    grid_world_to_cellf(g, wx, wy, &fcol, &frow);

    *out_col = (int)floorf(fcol);
    *out_row = (int)floorf(frow);
}

/* ------------------------------------------------------------------------ */
/* Grid -> world                                                             */
/* ------------------------------------------------------------------------ */

/* Going backwards, a cell index names a square REGION, not a point, so we
 * have to choose which point in that region to return. We return the centre
 * (hence the + 0.5f) rather than the corner.
 *
 * The corner would be the more literal inverse, but it biases every result
 * half a cell down and left -- small enough to look correct in a render and
 * large enough to confuse you later when a waypoint appears to sit slightly
 * off the track. The centre is the honest answer to "where is this cell". */
void grid_cell_to_world(const Grid *g, int col, int row,
                        float *out_wx, float *out_wy)
{
    *out_wx = g->origin_x + ((float)col + 0.5f) * g->resolution;

    /* The same flip as above, run in reverse: convert a row index counting
     * down into a count of cells up from the bottom, then scale to metres. */
    float cells_up = (float)(g->height - 1 - row);
    *out_wy = g->origin_y + (cells_up + 0.5f) * g->resolution;
}

/* ------------------------------------------------------------------------ */
/* Bounds                                                                    */
/* ------------------------------------------------------------------------ */

/* Out-of-bounds reads as occupied. That is a deliberate choice, not just
 * defensive coding: it makes the edge of the map behave like a wall, which
 * gives the raycaster a natural stopping condition and means a ray fired off
 * the side of the map terminates instead of marching forever. */
int grid_is_blocked(const Grid *g, int col, int row)
{
    if (col < 0 || col >= g->width || row < 0 || row >= g->height)
        return 1;

    return g->cells[(size_t)row * (size_t)g->width + (size_t)col] != 0;
}

/* ------------------------------------------------------------------------ */
/* Tests                                                                     */
/* ------------------------------------------------------------------------ */

static int checks = 0, failures = 0;

static void check(int condition, const char *what)
{
    checks++;
    if (!condition) {
        failures++;
        printf("  FAIL: %s\n", what);
    }
}

static void check_near(float got, float want, float tol, const char *what)
{
    checks++;
    if (fabsf(got - want) > tol) {
        failures++;
        printf("  FAIL: %s (got %.4f, want %.4f)\n", what, got, want);
    }
}

int main(void)
{
    /* A deliberately trivial grid: resolution 1.0 and origin (0,0) mean the
     * arithmetic is checkable in your head, which is what you want when the
     * thing under test is the arithmetic itself. */
    Grid g = { .cells = NULL, .width = 10, .height = 10,
               .resolution = 1.0f, .origin_x = 0.0f, .origin_y = 0.0f };

    int col, row;

    printf("simple grid: 10x10, res 1.0, origin (0,0)\n");

    /* Bottom-left of the world is the BOTTOM-left of the image, so the last
     * row, not the first. */
    grid_world_to_cell(&g, 0.5f, 0.5f, &col, &row);
    check(col == 0, "world (0.5, 0.5) -> col 0");
    check(row == 9, "world (0.5, 0.5) -> row 9 (bottom of image)");

    /* Top-left: same column, row 0. */
    grid_world_to_cell(&g, 0.5f, 9.5f, &col, &row);
    check(col == 0, "world (0.5, 9.5) -> col 0");
    check(row == 0, "world (0.5, 9.5) -> row 0 (top of image)");

    /* Raising y must LOWER the row index. If this one passes and the two
     * above fail, your flip is inverted. */
    int row_low, row_high;
    grid_world_to_cell(&g, 5.0f, 2.0f, &col, &row_low);
    grid_world_to_cell(&g, 5.0f, 8.0f, &col, &row_high);
    check(row_high < row_low, "higher world y gives a lower row index");

    /* The fractional version must keep the sub-cell position: 3.7 metres in
     * a 1-metre grid is 3.7 cells, not 3. */
    float fcol, frow;
    grid_world_to_cellf(&g, 3.7f, 0.5f, &fcol, &frow);
    check_near(fcol, 3.7f, 1e-4f, "fractional col keeps sub-cell position");

    /* Negative coordinates: floor, not truncation. A point at x = -0.5 is
     * outside the left edge and belongs to cell -1. */
    grid_world_to_cell(&g, -0.5f, 0.5f, &col, &row);
    check(col == -1, "world x = -0.5 -> col -1 (floor, not truncate)");

    printf("\nround trip\n");

    /* The round trip cannot be exact -- flooring discards the position within
     * the cell -- so the assertion is "within half a cell", not equality.
     * Asserting equality here is the classic way to write a test that fails
     * for the wrong reason. */
    for (float wy = 0.5f; wy < 10.0f; wy += 2.0f) {
        for (float wx = 0.5f; wx < 10.0f; wx += 2.0f) {
            float back_x, back_y;
            grid_world_to_cell(&g, wx, wy, &col, &row);
            grid_cell_to_world(&g, col, row, &back_x, &back_y);

            check_near(back_x, wx, 0.5f * g.resolution, "round trip x");
            check_near(back_y, wy, 0.5f * g.resolution, "round trip y");
        }
    }

    printf("\nrealistic grid: 2000x2000, res 0.06367, origin (-39.038, -49.237)\n");

    /* The numbers from the real IMS map, so the test exercises the awkward
     * values rather than only the tidy ones. */
    Grid r = { .cells = NULL, .width = 2000, .height = 2000,
               .resolution = 0.06367f,
               .origin_x = -39.038f, .origin_y = -49.237f };

    /* The worked example: a car near the bottom edge of the map. */
    grid_world_to_cell(&r, -39.0f, -49.0f, &col, &row);
    check(row == 1996, "world y = -49.0 -> row 1996 (near bottom of image)");

    /* And near the top edge. */
    float top_y = r.origin_y + (float)r.height * r.resolution;
    grid_world_to_cell(&r, -39.0f, top_y - 0.01f, &col, &row);
    check(row == 0, "world y at the top edge -> row 0");

    /* Bounds policy: off the map reads as occupied. */
    check(grid_is_blocked(&r, -1, 100), "col -1 is blocked");
    check(grid_is_blocked(&r, 100, 2000), "row 2000 is blocked");

    printf("\n%d checks, %d failed\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
