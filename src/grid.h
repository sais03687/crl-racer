#ifndef GRID_H
#define GRID_H

#include <stddef.h>
#include <stdint.h>

#include "config.h"

/*
 * ============================================================================
 * OCCUPANCY GRID
 * ============================================================================
 *
 * An occupancy grid is the simplest useful map a robot can have: chop the
 * world into a square lattice of cells and record, for each one, whether you
 * can drive through it. That is all. No polygons, no walls-as-line-segments,
 * no geometry library. The pay-off is that "is this point blocked?" becomes
 * two divisions and an array lookup, and ray casting becomes a walk along a
 * row of cells.
 *
 * The map arrives as two files, which is the convention ROS established and
 * F1TENTH inherited:
 *
 *   Foo_map.png   the picture -- dark pixels are wall, light pixels are road
 *   Foo_map.yaml  the metadata that gives the picture a physical size and
 *                 tells you where it sits in the world
 *
 * The image alone is meaningless: it is just pixels. The YAML supplies the two
 * facts that turn pixels into metres:
 *
 *   resolution -- how many metres across one pixel is (about 0.06 here, so
 *                 roughly 6 cm per cell for a 1/10-scale car)
 *   origin     -- the world coordinate of the image's BOTTOM-LEFT corner
 *
 * ---------------------------------------------------------------------------
 * The row-flip, which is where everyone's first bug lives
 * ---------------------------------------------------------------------------
 *
 * Images and maps disagree about which way is up.
 *
 *   In an image, row 0 is the TOP row, and row numbers increase downward.
 *   That is how every image format stores its scanlines.
 *
 *   In the world, y increases UPWARD (north), and the origin is stated for
 *   the bottom-left corner.
 *
 * So converting between them requires flipping the row index:
 *
 *     row = height - 1 - (how many cells up from the bottom)
 *
 * We keep the array in image order -- row 0 on top -- because that is how it
 * comes off disk and how the ASCII renderer wants to print it, and we put the
 * flip inside the two transform functions. Anywhere else in the codebase you
 * can forget this exists. If you ever find yourself writing `height - 1 - row`
 * outside grid.c, something has leaked.
 *
 * ---------------------------------------------------------------------------
 * From grey to blocked
 * ---------------------------------------------------------------------------
 *
 * The image is greyscale, not black and white, so we need a rule for which
 * greys are wall. ROS's rule, which we follow so that published maps behave
 * exactly as their authors intended:
 *
 *     p = (255 - pixel) / 255           (or pixel/255 if `negate` is set)
 *
 * p is read as "probability this cell is occupied": black gives p = 1, white
 * gives p = 0. Then p above `occupied_thresh` is wall, p below `free_thresh`
 * is road, and anything in between is UNKNOWN -- genuinely undetermined, which
 * is what a real mapping run produces for corners the laser never saw.
 *
 * We treat unknown as blocked (configurable). For a car that is the safe
 * reading: driving into territory the map cannot vouch for is how you hit
 * something that was never recorded.
 */

typedef enum CellState {
    CELL_FREE     = 0,
    CELL_OCCUPIED = 1,
    CELL_UNKNOWN  = 2
} CellState;

typedef struct Grid {
    /* Flat array, row-major, `width * height` entries, holding CellState
     * values. Flat rather than a uint8_t** of row pointers: one allocation,
     * contiguous memory, and index arithmetic you can read. */
    uint8_t *cells;

    int   width;
    int   height;
    float resolution;     /* metres per cell */

    /* World coordinate of the bottom-left corner of the map, plus the map's
     * rotation about that corner. Every published F1TENTH map has
     * origin_theta == 0, but the field is part of the format and honouring it
     * costs six lines, so it is handled properly rather than assumed away. */
    float origin_x;
    float origin_y;
    float origin_theta;
} Grid;

/*
 * Load a map from its YAML sidecar. The image path inside the YAML is resolved
 * relative to the YAML file's own directory, which is what makes a downloaded
 * track folder work unmodified.
 *
 * Returns 1 on success, 0 on failure with an explanation in `err`.
 */
int  grid_load(const char *yaml_path, const Config *cfg, Grid *g,
               char *err, size_t err_len);

void grid_free(Grid *g);

/* --- Coordinate transforms ----------------------------------------------- */

/*
 * World (metres) -> grid (cell indices).
 *
 * The result is NOT clamped and may legitimately be outside the map: a LiDAR
 * ray aimed off the edge has to be able to say so. Callers check bounds with
 * grid_in_bounds(), or use the *_blocked_* helpers which treat off-map as
 * blocked.
 */
void grid_world_to_grid(const Grid *g, float wx, float wy, int *col, int *row);

/*
 * Grid -> world, returning the CENTRE of the cell.
 *
 * The centre, not the corner, because a cell is an area and its centre is the
 * point that best represents it. Round-tripping through the corner would sit
 * exactly on a boundary where floating-point rounding can fall either way.
 */
void grid_grid_to_world(const Grid *g, int col, int row, float *wx, float *wy);

/*
 * The same two transforms, but keeping the fractional part.
 *
 * The ray caster needs to know not just which cell it starts in but WHERE
 * inside that cell, so it can work out how far it is to the next cell
 * boundary. These give continuous (column, row) coordinates on exactly the
 * axes the integer versions use: column grows right, row grows DOWN, and
 * floor()ing the results reproduces grid_world_to_grid() exactly.
 *
 * They exist so that the row-flip and the map rotation stay confined to
 * grid.c. lidar.c can then march through cells without ever knowing that the
 * row axis points the opposite way to world y.
 */
void grid_world_to_cellf(const Grid *g, float wx, float wy,
                         float *fcol, float *frow);

/*
 * A direction rather than a position: rotated like a vector, and with no
 * origin offset applied. A unit vector in stays a unit vector out (the row
 * component simply changes sign relative to world y), so distances measured
 * along it are still in cells and convert to metres with one multiply.
 */
void grid_world_dir_to_cell_dir(const Grid *g, float dx, float dy,
                                float *dcol, float *drow);

/* --- Queries -------------------------------------------------------------- */

int     grid_in_bounds(const Grid *g, int col, int row);

/* CELL_OCCUPIED for anything off the map, so callers cannot forget the check. */
uint8_t grid_cell(const Grid *g, int col, int row);

/* Applies the config's unknown-cell policy on top of grid_cell(). */
int grid_is_blocked(const Grid *g, const Config *cfg, int col, int row);
int grid_is_blocked_world(const Grid *g, const Config *cfg, float wx, float wy);

/* Count of each cell state, for the startup summary and for sanity-checking a
 * newly downloaded map. */
void grid_count_states(const Grid *g, long *free_cells, long *occupied_cells,
                       long *unknown_cells);

#endif /* GRID_H */
