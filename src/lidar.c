#include "lidar.h"

#include "mathf.h"

#include <math.h>

float lidar_cast_ray(const Grid *g, const Config *cfg,
                     float ox, float oy, float angle)
{
    /* Work entirely in CELL units rather than metres. The grid boundaries are
     * then at the integers, which makes every "distance to the next boundary"
     * a subtraction against a whole number instead of a multiply-and-compare
     * against the resolution. One multiply at the very end converts back. */
    float start_col, start_row;
    grid_world_to_cellf(g, ox, oy, &start_col, &start_row);

    float dir_col, dir_row;
    grid_world_dir_to_cell_dir(g, cosf(angle), sinf(angle), &dir_col, &dir_row);

    int cell_col = (int)floorf(start_col);
    int cell_row = (int)floorf(start_row);

    /* If the beam starts inside something solid there is nothing to march
     * through. See the header for why this is 0 and not max range. */
    if (grid_is_blocked(g, cfg, cell_col, cell_row))
        return 0.0f;

    /* Which way each index moves as the ray advances. */
    int step_col = (dir_col > 0.0f) ? 1 : -1;
    int step_row = (dir_row > 0.0f) ? 1 : -1;

    /* t is measured in cells along the ray.
     *
     * t_delta_col: how far along the ray between successive vertical grid
     * lines. If the ray is 45 degrees, crossing one column costs sqrt(2)
     * cells of travel, and 1/|dir_col| = 1/0.707 gives exactly that.
     *
     * A ray exactly parallel to an axis never crosses that axis' boundaries at
     * all. Rather than special-casing it everywhere, set the distance to
     * infinity: the "which boundary is nearer" comparison then always picks
     * the other axis, which is the correct behaviour and needs no branch.
     * (Comparisons against infinity are well-defined for IEEE floats; it is
     * only arithmetic that produces NaN, and we never do arithmetic on it
     * because the branch is never taken.) */
    float t_delta_col = (dir_col != 0.0f) ? fabsf(1.0f / dir_col) : INFINITY;
    float t_delta_row = (dir_row != 0.0f) ? fabsf(1.0f / dir_row) : INFINITY;

    /* t_max: distance to the FIRST boundary on each axis. This depends on
     * where inside the starting cell we are, which is why the fractional part
     * of the start position was needed. Going right, the next vertical line is
     * at cell_col + 1; going left, it is at cell_col itself. */
    float t_max_col;
    if (dir_col > 0.0f)      t_max_col = ((float)cell_col + 1.0f - start_col) / dir_col;
    else if (dir_col < 0.0f) t_max_col = ((float)cell_col - start_col) / dir_col;
    else                     t_max_col = INFINITY;

    float t_max_row;
    if (dir_row > 0.0f)      t_max_row = ((float)cell_row + 1.0f - start_row) / dir_row;
    else if (dir_row < 0.0f) t_max_row = ((float)cell_row - start_row) / dir_row;
    else                     t_max_row = INFINITY;

    float limit_cells = cfg->lidar_max_range_m / g->resolution;

    /* March. The loop always terminates: t strictly increases by at least
     * min(t_delta_col, t_delta_row) each iteration, and both the range limit
     * and the map edge (which reads as blocked) stop it. */
    for (;;) {
        float t;

        if (t_max_col < t_max_row) {
            t = t_max_col;
            cell_col  += step_col;
            t_max_col += t_delta_col;
        } else {
            t = t_max_row;
            cell_row  += step_row;
            t_max_row += t_delta_row;
        }

        /* Range check before the cell test: a wall beyond the sensor's reach
         * must not be reported, or the driver would see through the horizon. */
        if (t >= limit_cells)
            return cfg->lidar_max_range_m;

        if (grid_is_blocked(g, cfg, cell_col, cell_row))
            return t * g->resolution;   /* cells back to metres */
    }
}

void lidar_scan(const Grid *g, const Config *cfg, Rng *rng,
                float ox, float oy, float heading, float *out_ranges)
{
    for (int i = 0; i < cfg->ray_count; i++) {
        float angle = heading + config_ray_bearing(cfg, i);
        float range = lidar_cast_ray(g, cfg, ox, oy, angle);

        if (cfg->lidar_noise_stddev_m > 0.0f && rng) {
            range += rng_normal(rng, 0.0f, cfg->lidar_noise_stddev_m);
            /* Noise must not invent a negative distance or a reading beyond
             * what the hardware could report -- a driver written against the
             * real sensor's contract would be entitled to trust both. */
            range = clampf(range, 0.0f, cfg->lidar_max_range_m);
        }

        out_ranges[i] = range;
    }
}
