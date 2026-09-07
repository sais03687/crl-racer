#ifndef LIDAR_H
#define LIDAR_H

#include "config.h"
#include "grid.h"
#include "rng.h"

/*
 * ============================================================================
 * SIMULATED 2D LiDAR
 * ============================================================================
 *
 * A LiDAR spins a laser around and, many times per revolution, measures how
 * far away the first thing in that direction is. The output is an array of
 * distances -- one per beam, in a known angular order. That array is the ONLY
 * thing the driver in this project gets to see. It does not know where it is
 * on the track, which way the track goes, or that a track exists.
 *
 * Simulating it means answering, for each beam: starting at the car and
 * heading in this direction, how far do I travel before I hit an occupied
 * cell? That is ray casting.
 *
 * ---------------------------------------------------------------------------
 * Why DDA and not just stepping along the ray
 * ---------------------------------------------------------------------------
 *
 * The obvious approach is to walk the ray in small increments -- add 1 cm to
 * the position, check the cell, repeat. It works, and it is wrong in two ways
 * at once. Too large a step and the ray jumps clean over a thin wall (the
 * walls on these maps are about one cell thick, so this is not hypothetical).
 * Too small a step and you check the same cell dozens of times over.
 *
 * DDA -- a digital differential analyser, in the sense used by Amanatides and
 * Woo's 1987 grid-traversal paper -- fixes both. Instead of stepping by
 * distance, it steps from cell boundary to cell boundary. At each point it
 * asks: is the next vertical grid line closer, or the next horizontal one? It
 * crosses whichever comes first and moves into that neighbouring cell.
 *
 * So it visits every cell the ray passes through, in order, exactly once. No
 * cell can be skipped, so no wall can be missed however thin it is, and no
 * work is repeated. The number of iterations is proportional to the number of
 * cells crossed, which is the minimum possible.
 *
 * The bookkeeping is two running numbers per axis:
 *
 *   t_max_col   distance along the ray to the next vertical boundary
 *   t_delta_col distance along the ray between two consecutive vertical
 *               boundaries -- constant, because the grid is uniform
 *
 * and the same pair for rows. Each iteration takes the smaller t_max, steps
 * that axis' cell index by +/-1, and adds t_delta to that t_max to point at
 * the following boundary. That is the whole algorithm.
 */

/*
 * Cast a single ray from (ox, oy) along `angle` (world frame, radians).
 *
 * Returns the distance in metres to the first blocked cell, or
 * cfg->lidar_max_range_m if nothing is hit within that distance. Leaving the
 * map counts as a hit, because grid_cell() reports off-map as occupied.
 *
 * Returns 0 if the ray starts inside a wall. That is a real state -- it is
 * what a crashed car sees -- and reporting it as 0 rather than max range keeps
 * the driver from cheerfully accelerating into the scenery.
 */
float lidar_cast_ray(const Grid *g, const Config *cfg,
                     float ox, float oy, float angle);

/*
 * Fill `out_ranges` with cfg->ray_count readings taken from (ox, oy) with the
 * car facing `heading`.
 *
 * Ray 0 is at heading - fov/2 (the car's right) and the last ray is at
 * heading + fov/2 (its left), matching how a real scanner reports its sweep
 * and how config_ray_bearing() indexes it.
 *
 * `rng` may be NULL if cfg->lidar_noise_stddev_m is zero. When noise is on,
 * each reading is perturbed independently -- real LiDAR error is per-beam, not
 * a single offset applied to the whole sweep.
 */
void lidar_scan(const Grid *g, const Config *cfg, Rng *rng,
                float ox, float oy, float heading, float *out_ranges);

#endif /* LIDAR_H */
