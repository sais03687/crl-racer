#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <stddef.h>

/*
 * ============================================================================
 * CENTERLINE WAYPOINTS
 * ============================================================================
 *
 * The centerline is a list of points running down the middle of the track,
 * closed into a loop (the last point joins back to the first). The
 * f1tenth_racetracks repository ships one per circuit as a CSV:
 *
 *     # x_m, y_m, w_tr_right_m, w_tr_left_m
 *     0.0, 0.0, 1.1, 1.1
 *     0.00737, -0.36408, 1.1, 1.1
 *     ...
 *
 * x and y are in the SAME world frame as the map's origin, so a point can be
 * fed straight to grid_world_to_grid(). The two width columns say how far the
 * track edge is to each side at that point.
 *
 * The simulator uses the centerline for two things, and neither of them is
 * driving -- the Follow-The-Gap driver never looks at it:
 *
 *   1. Where to put the car on reset.
 *   2. Measuring PROGRESS. "How far round the track am I?" is not a question
 *      you can answer from x and y alone, because the track is a loop and the
 *      same position occurs on every lap. What you need is arc length: the
 *      distance travelled ALONG the centerline from the start. That gives a
 *      single number that increases monotonically as you drive, and it is
 *      what both the reward and the lap counter are built on.
 *
 * ---------------------------------------------------------------------------
 * Layout
 * ---------------------------------------------------------------------------
 * Parallel flat arrays rather than an array of point structs. Same memory,
 * same indexing, and it keeps the "flat arrays, explicit indexing" rule of
 * this project literal: wp->x[i] and wp->y[i] are the i'th point.
 */
typedef struct Waypoints {
    int count;

    float *x;         /* world metres */
    float *y;
    float *w_right;   /* track half-width to the right at this point */
    float *w_left;

    /* Cumulative arc length: s[i] is the distance from point 0 to point i
     * measured along the polyline. s[0] is 0 by definition. Precomputed at
     * load time because it never changes and is needed every single step. */
    float *s;

    /* Length of the full closed loop, i.e. s[count-1] plus the closing segment
     * from the last point back to the first. Anything that wraps around needs
     * this, not s[count-1]. */
    float total_length;
} Waypoints;

/*
 * Load a centerline CSV. Lines beginning with '#' are comments; fields may be
 * separated by commas, semicolons or whitespace. Rows with fewer than two
 * numbers are skipped. Missing width columns default to 0.
 *
 * Returns 1 on success, 0 on failure with a message in `err`.
 */
int  waypoints_load(const char *csv_path, Waypoints *w, char *err, size_t err_len);

void waypoints_free(Waypoints *w);

/*
 * Index of the centerline point closest to (x, y).
 *
 * If `prev` is >= 0 and `window` > 0, only the points within `window` indices
 * of `prev` are examined, wrapping around the ends. This is a correctness
 * measure, not an optimisation: circuits pass close to themselves at hairpins
 * and crossings, and a global search will happily decide the car has jumped to
 * the far lobe of the track, which makes progress and lap counting nonsense.
 * Pass prev = -1 to search everything (used once, at reset).
 */
int waypoints_nearest(const Waypoints *w, float x, float y, int prev, int window);

/*
 * How far around the lap the car is, in metres, refined below the spacing of
 * the waypoints themselves.
 *
 * Taking s[nearest] directly would quantise progress to the ~0.4 m gap between
 * points, so the reward would arrive in lumps -- a few steps of zero, then a
 * jump. Instead the car's position is projected perpendicularly onto the
 * centerline segments either side of the nearest point, which gives a smooth,
 * continuously increasing value.
 */
float waypoints_arclength(const Waypoints *w, int nearest, float x, float y);

/*
 * Signed forward progress from `from` to `to`, both arc lengths.
 *
 * Needed because arc length wraps: crossing the start/finish line takes you
 * from 549.9 m back to 0.1 m, and a naive subtraction reports that as -549.8 m
 * of progress rather than +0.2 m. This resolves the ambiguity by assuming the
 * car moved by less than half a lap in one step, which at any sane step size
 * it did.
 */
float waypoints_forward_delta(const Waypoints *w, float from, float to);

/*
 * Heading (radians) of the centerline at point i, i.e. the direction from
 * point i to point i+1. Used to face the car down the track on reset.
 */
float waypoints_heading(const Waypoints *w, int i);

#endif /* WAYPOINTS_H */
