#include "env.h"

#include "mathf.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char *done_reason_name(DoneReason reason)
{
    switch (reason) {
    case DONE_RUNNING: return "running";
    case DONE_CRASH:   return "wall contact";
    case DONE_LAP:     return "lap complete";
    case DONE_TIMEOUT: return "time limit";
    default:           return "unknown";
    }
}

int env_init(Env *env, const Config *cfg,
             const char *map_yaml, const char *centerline_csv,
             char *err, size_t err_len)
{
    memset(env, 0, sizeof *env);
    env->cfg = cfg;

    if (!grid_load(map_yaml, cfg, &env->grid, err, err_len))
        return 0;

    if (!waypoints_load(centerline_csv, &env->wp, err, err_len)) {
        grid_free(&env->grid);
        return 0;
    }

    env->scan = (float *)malloc((size_t)cfg->ray_count * sizeof(float));
    if (!env->scan) {
        grid_free(&env->grid);
        waypoints_free(&env->wp);
        if (err) snprintf(err, err_len, "out of memory for %d LiDAR rays", cfg->ray_count);
        return 0;
    }

    return 1;
}

void env_free(Env *env)
{
    grid_free(&env->grid);
    waypoints_free(&env->wp);
    free(env->scan);
    env->scan = NULL;
}

/* ------------------------------------------------------------------------- */
/* Collision                                                                  */
/* ------------------------------------------------------------------------- */

int env_car_collides(const Env *env)
{
    const Config *cfg = env->cfg;

    /* Testing only the car's centre point would let it slice a wall with its
     * corner while the centre was still over clear road. Testing only the four
     * corners is better but can still miss a wall that passes between them.
     *
     * So: sample the whole rectangle on a lattice, at half a cell spacing. Two
     * samples half a cell apart cannot have a cell between them that neither
     * lands in, so no wall -- however thin, at whatever angle -- can slip
     * through the sampling. For a 0.58 x 0.31 m body on a 0.06 m grid that is
     * roughly 20 x 11 points, which is nothing to check a few hundred times a
     * second, and it is obviously correct, which the clever alternatives are
     * not. */
    float spacing = 0.5f * env->grid.resolution;

    float to_front = 0.5f * cfg->wheelbase_m + 0.5f * cfg->car_length_m;
    float to_rear  = 0.5f * cfg->car_length_m - 0.5f * cfg->wheelbase_m;
    float half_w   = 0.5f * cfg->car_width_m;

    float c = cosf(env->car.heading);
    float s = sinf(env->car.heading);

    /* Walk the body frame; the <= and the explicit clamp on the last sample
     * make sure the far edges are actually tested rather than falling just
     * short of them. */
    for (float bx = -to_rear; bx <= to_front + 1e-4f; bx += spacing) {
        float fx = bx > to_front ? to_front : bx;

        for (float by = -half_w; by <= half_w + 1e-4f; by += spacing) {
            float fy = by > half_w ? half_w : by;

            float wx = env->car.x + c * fx - s * fy;
            float wy = env->car.y + s * fx + c * fy;

            if (grid_is_blocked_world(&env->grid, cfg, wx, wy))
                return 1;
        }
    }

    return 0;
}

/* ------------------------------------------------------------------------- */
/* Reset                                                                      */
/* ------------------------------------------------------------------------- */

Observation env_reset(Env *env)
{
    const Config *cfg = env->cfg;

    /* Re-seeding here rather than in env_init means episode N from a given
     * seed is identical whether it is the first episode or the hundredth. If
     * the RNG carried over between episodes, a bug found on episode 40 could
     * only be reproduced by replaying all 40. */
    rng_init(&env->rng, cfg->rng_seed);

    int start = cfg->start_waypoint;
    if (start < 0 || start >= env->wp.count) start = 0;

    float heading = waypoints_heading(&env->wp, start);

    float x = env->wp.x[start];
    float y = env->wp.y[start];

    /* Optional starting scatter. Zero by default so the acceptance run is
     * exactly repeatable; useful when you want to know whether a driver
     * survives a start it has not seen. The offset is applied SIDEWAYS
     * (perpendicular to the track direction) because nudging the car along the
     * track just changes where the lap starts, which is not interesting. */
    if (cfg->start_lateral_jitter_m > 0.0f) {
        float offset = rng_range(&env->rng, -cfg->start_lateral_jitter_m,
                                             cfg->start_lateral_jitter_m);
        x += -sinf(heading) * offset;
        y +=  cosf(heading) * offset;
    }
    if (cfg->start_heading_jitter_rad > 0.0f) {
        heading += rng_range(&env->rng, -cfg->start_heading_jitter_rad,
                                         cfg->start_heading_jitter_rad);
    }

    car_reset(&env->car, x, y, heading, 0.0f);

    /* -1 asks for a global nearest-point search. This is the one moment it is
     * correct to do that: there is no previous index to search around yet. */
    env->wp_index        = waypoints_nearest(&env->wp, x, y, -1, 0);
    env->arclength_m     = waypoints_arclength(&env->wp, env->wp_index, x, y);
    env->distance_along_m = 0.0f;

    env->sim_time_s    = 0.0f;
    env->control_steps = 0;
    env->reason        = DONE_RUNNING;

    lidar_scan(&env->grid, cfg, &env->rng, x, y, heading, env->scan);

    Observation obs;
    obs.scan       = env->scan;
    obs.scan_count = cfg->ray_count;
    obs.speed_mps  = env->car.speed;
    return obs;
}

/* ------------------------------------------------------------------------- */
/* Step                                                                       */
/* ------------------------------------------------------------------------- */

StepResult env_step(Env *env, Action action)
{
    const Config *cfg = env->cfg;

    int   substeps = config_steps_per_control(cfg);
    int   crashed  = 0;

    /* Hold the action constant across the physics substeps. This is not a
     * shortcut -- it is what actually happens on a real vehicle, where the
     * controller updates far less often than the world evolves. Letting the
     * driver act every physics step would quietly make it a much better
     * driver than it could be in hardware. */
    for (int i = 0; i < substeps; i++) {
        car_step(&env->car, cfg, action.steering_rad, action.target_speed_mps,
                 cfg->physics_dt_s);
        env->sim_time_s += cfg->physics_dt_s;

        /* Checked EVERY substep, not once per control period. The whole reason
         * the physics step is small is so the car cannot cross a wall inside
         * one step; testing only at the end of the control period would throw
         * that away and let it pass through walls between checks. */
        if (env_car_collides(env)) {
            crashed = 1;
            break;
        }
    }

    env->control_steps++;

    /* --- Progress ------------------------------------------------------- */
    env->wp_index = waypoints_nearest(&env->wp, env->car.x, env->car.y,
                                      env->wp_index, cfg->nearest_search_window);

    float new_s = waypoints_arclength(&env->wp, env->wp_index, env->car.x, env->car.y);
    float delta = waypoints_forward_delta(&env->wp, env->arclength_m, new_s);

    env->arclength_m      = new_s;
    env->distance_along_m += delta;

    float reward = delta * cfg->reward_progress_scale;

    /* --- Termination ---------------------------------------------------- */
    /* Order matters. Crashing beats finishing: a car that clips the wall on
     * the line has not completed a clean lap, and reporting it as a lap would
     * make the acceptance criterion meaningless. */
    DoneReason reason = DONE_RUNNING;

    if (crashed) {
        reason  = DONE_CRASH;
        reward += cfg->reward_crash;
    } else if (env->distance_along_m >= env->wp.total_length * cfg->lap_fraction) {
        reason  = DONE_LAP;
        reward += cfg->reward_lap;
    } else if (env->sim_time_s >= cfg->max_episode_time_s) {
        reason = DONE_TIMEOUT;
    }

    env->reason = reason;

    /* Scan from wherever the car ended up, crashed or not. A crashed car's
     * scan is meaningful (it reads ~0 in the direction of the wall) and the
     * caller may want to look at it. */
    lidar_scan(&env->grid, cfg, &env->rng,
               env->car.x, env->car.y, env->car.heading, env->scan);

    StepResult result;
    result.obs.scan       = env->scan;
    result.obs.scan_count = cfg->ray_count;
    result.obs.speed_mps  = env->car.speed;
    result.reward         = reward;
    result.done           = (reason != DONE_RUNNING);
    result.reason         = reason;
    return result;
}

float env_lap_progress(const Env *env)
{
    if (env->wp.total_length <= 0.0f) return 0.0f;
    return env->distance_along_m / env->wp.total_length;
}
