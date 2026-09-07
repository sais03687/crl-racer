#include "ftg.h"

#include "mathf.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

int ftg_init(FollowTheGap *d, const Config *cfg)
{
    memset(d, 0, sizeof *d);

    d->processed = (float *)malloc((size_t)cfg->ray_count * sizeof(float));
    if (!d->processed) return 0;

    d->count = cfg->ray_count;
    return 1;
}

void ftg_free(FollowTheGap *d)
{
    free(d->processed);
    d->processed = NULL;
    d->count = 0;
}

/* --- Stage 1: clip and smooth -------------------------------------------- */

static void preprocess(FollowTheGap *d, const Config *cfg, const float *scan, int n)
{
    int w = cfg->ftg_smooth_window;

    for (int i = 0; i < n; i++) {
        if (w <= 0) {
            d->processed[i] = fminf(scan[i], cfg->ftg_range_clip_m);
            continue;
        }

        /* Mean over the beams within w either side. The window is truncated at
         * the ends of the scan rather than wrapped: beam 0 and beam n-1 point
         * in completely different directions (they are the two ends of a 270
         * degree sweep, not neighbours), so wrapping would average the view
         * out of the car's right side with the view out of its left. */
        int lo = i - w < 0 ? 0 : i - w;
        int hi = i + w >= n ? n - 1 : i + w;

        float sum = 0.0f;
        for (int k = lo; k <= hi; k++)
            sum += fminf(scan[k], cfg->ftg_range_clip_m);

        d->processed[i] = sum / (float)(hi - lo + 1);
    }
}

/* --- Stage 2 and 3: closest point, then the safety bubble ---------------- */

static void apply_safety_bubble(FollowTheGap *d, const Config *cfg, int n)
{
    int closest = 0;
    for (int i = 1; i < n; i++)
        if (d->processed[i] < d->processed[closest])
            closest = i;

    d->closest_ray = closest;

    float distance = d->processed[closest];

    /* How wide is the bubble, in radians, as seen from the car?
     *
     * Picture a circle of radius `bubble` sitting at distance `distance`. The
     * half-angle it subtends is atan(bubble / distance). Far away, that is a
     * sliver; up close it approaches 90 degrees and wipes out half the scan --
     * which is the intended behaviour, because something almost touching the
     * car should rule out almost every direction on that side.
     *
     * atan2f rather than atanf so that distance == 0 (the car is already
     * against a wall) gives pi/2 instead of a division by zero. */
    float half_angle = atan2f(cfg->ftg_bubble_radius_m, fmaxf(distance, 1e-4f));

    float increment = config_ray_angle_increment(cfg);
    int half_rays = (increment > 0.0f) ? (int)(half_angle / increment) : n;
    if (half_rays > n) half_rays = n;

    int lo = closest - half_rays < 0 ? 0 : closest - half_rays;
    int hi = closest + half_rays >= n ? n - 1 : closest + half_rays;

    for (int i = lo; i <= hi; i++)
        d->processed[i] = 0.0f;
}

/* --- Stage 4: the longest run of open beams ------------------------------ */

static void find_largest_gap(FollowTheGap *d, const Config *cfg, int n)
{
    int best_start = -1, best_len = 0;
    int run_start  = -1;

    for (int i = 0; i < n; i++) {
        int open = d->processed[i] > cfg->ftg_gap_threshold_m;

        if (open && run_start < 0) {
            run_start = i;
        } else if (!open && run_start >= 0) {
            int len = i - run_start;
            if (len > best_len) { best_len = len; best_start = run_start; }
            run_start = -1;
        }
    }

    /* A run that reaches the last beam never hits the closing `else` above, so
     * it has to be considered separately. Forgetting this is the classic bug
     * in this kind of loop, and here it would mean the car ignored a gap that
     * happened to sit at the edge of its vision. */
    if (run_start >= 0) {
        int len = n - run_start;
        if (len > best_len) { best_len = len; best_start = run_start; }
    }

    if (best_len > 0) {
        d->gap_start = best_start;
        d->gap_end   = best_start + best_len - 1;

        /* Aim at the middle of the gap rather than at its deepest point. The
         * deepest point is tempting -- it is the most open direction -- but it
         * jumps around between neighbouring beams as the car moves, and the
         * steering chatters. The midpoint moves smoothly and keeps the most
         * clearance on both sides. */
        d->target_ray = (d->gap_start + d->gap_end) / 2;
    } else {
        /* Nothing at all reads past the threshold: the car is boxed in, or
         * pointing at a wall a metre away. Fall back to the most open
         * direction there is and let the speed rule below crawl. Giving up
         * (steering straight) would drive into the wall it can already see. */
        int best = 0;
        for (int i = 1; i < n; i++)
            if (d->processed[i] > d->processed[best]) best = i;

        d->gap_start = d->gap_end = d->target_ray = best;
    }
}

/* --- Stage 5: turn the target beam into an action ------------------------ */

Action ftg_plan(FollowTheGap *d, const Config *cfg, const Observation *obs)
{
    int n = obs->scan_count < d->count ? obs->scan_count : d->count;

    preprocess(d, cfg, obs->scan, n);
    apply_safety_bubble(d, cfg, n);
    find_largest_gap(d, cfg, n);

    /* config_ray_bearing gives the target beam's angle relative to the car's
     * heading -- which is precisely the direction we want to be pointing, so
     * it can be used as the steering angle directly. The gain is there to tune
     * how aggressively that is chased; 1.0 means "turn the wheel to the full
     * bearing", below 1.0 damps the response. */
    float steer = config_ray_bearing(cfg, d->target_ray) * cfg->ftg_steer_gain;
    steer = clampf(steer, -cfg->max_steer_rad, cfg->max_steer_rad);

    /* Speed falls off linearly with how hard we are steering: full speed
     * pointing straight ahead, min_speed at full lock.
     *
     * The reason this simple rule is enough: the only thing that makes this
     * driver steer is an obstacle, so "steering hard" and "in a tight spot"
     * are the same condition. It gets you a car that lifts for corners without
     * anything that knows what a corner is.
     *
     * The floor at min_speed matters more than it looks. Without it the car
     * would stop dead facing a wall, and a stopped car cannot turn -- the
     * bicycle model's yaw rate is proportional to speed -- so it would sit
     * there forever. */
    float lock = (cfg->max_steer_rad > 0.0f) ? fabsf(steer) / cfg->max_steer_rad : 0.0f;
    float speed = cfg->max_speed_mps - (cfg->max_speed_mps - cfg->min_speed_mps) * lock;

    Action action;
    action.steering_rad     = steer;
    action.target_speed_mps = clampf(speed, cfg->min_speed_mps, cfg->max_speed_mps);
    return action;
}
