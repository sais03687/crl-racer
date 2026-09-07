#ifndef ENV_H
#define ENV_H

#include <stddef.h>

#include "config.h"
#include "dynamics.h"
#include "grid.h"
#include "lidar.h"
#include "rng.h"
#include "waypoints.h"

/*
 * ============================================================================
 * THE ENVIRONMENT
 * ============================================================================
 *
 * Everything below this point -- the map, the car, the laser, the clock -- is
 * assembled here into one object with two operations:
 *
 *     reset()        put the car back on the grid, hand back a first look
 *     step(action)   apply an action, advance time, hand back
 *                    (observation, reward, done)
 *
 * If that shape looks familiar it is because it is the standard interface for
 * this kind of simulator, and it is a genuinely good one regardless of what
 * drives it. It draws a hard line between "the world" and "the thing acting in
 * the world": the environment never knows what a driver is, and the driver
 * only ever sees what step() returns. You can swap Follow-The-Gap for a human
 * on a keyboard, or for anything else later, without touching a line in here.
 *
 * ---------------------------------------------------------------------------
 * The three return values
 * ---------------------------------------------------------------------------
 *
 * OBSERVATION -- what the driver is allowed to know. Here: the laser scan and
 * the car's own speed. Deliberately NOT the car's position, nor the
 * centerline. A driver that could read (x, y) off the environment could
 * memorise the track, which is a different and much easier problem than the
 * one this simulator poses.
 *
 * REWARD -- a single number scoring the step just taken. Here it is progress
 * along the centerline, in metres. Note "along the centerline", not "distance
 * travelled": driving in circles in the middle of the track covers ground but
 * earns nothing, and going backwards earns a negative amount. Crashing and
 * finishing add a one-off penalty and bonus.
 *
 * The centerline is used to MEASURE progress, but is never shown to the
 * driver. That separation is the whole trick: the score can be computed from
 * privileged knowledge the actor does not have.
 *
 * DONE -- whether the episode is over, plus why. Three ways to end: the car
 * touched a wall, the car completed a lap, or the clock ran out. The last one
 * exists so that a car sitting still, or crawling in a circle, still
 * terminates instead of hanging the program.
 */

typedef enum DoneReason {
    DONE_RUNNING = 0,   /* not finished */
    DONE_CRASH,
    DONE_LAP,
    DONE_TIMEOUT
} DoneReason;

/* What the driver commands. Both are requests, and both get clamped to the
 * config's limits inside the physics -- see dynamics.h. */
typedef struct Action {
    float steering_rad;
    float target_speed_mps;
} Action;

typedef struct Observation {
    /* Points into the environment's own buffer, which is overwritten by the
     * next step(). Borrowed, not owned: copy it if you need to keep it. This
     * avoids a malloc per step, and there is exactly one observation alive at
     * a time anyway. */
    const float *scan;
    int          scan_count;
    float        speed_mps;
} Observation;

typedef struct StepResult {
    Observation obs;
    float       reward;
    int         done;
    DoneReason  reason;
} StepResult;

typedef struct Env {
    const Config *cfg;

    Grid      grid;
    Waypoints wp;
    CarState  car;
    Rng       rng;

    float *scan;   /* cfg->ray_count floats, reused every step */

    /* --- progress bookkeeping --- */
    int   wp_index;             /* nearest centerline point, tracked between steps */
    float arclength_m;          /* how far round the lap, from waypoints_arclength */
    float distance_along_m;     /* accumulated forward progress, wrap-corrected */

    /* --- clock --- */
    float sim_time_s;
    int   control_steps;

    DoneReason reason;
} Env;

/*
 * Load a track and get ready to run. `map_yaml` is the map's YAML sidecar,
 * `centerline_csv` its centerline file.
 *
 * Returns 1 on success, 0 on failure with an explanation in `err`. Anything
 * already allocated is released on the failure path, so a failed init leaves
 * nothing behind and env_free() on it is safe.
 */
int  env_init(Env *env, const Config *cfg,
              const char *map_yaml, const char *centerline_csv,
              char *err, size_t err_len);

void env_free(Env *env);

/*
 * Put the car back at the start and return its first observation.
 *
 * The RNG is re-seeded from the config here, not in env_init, so that every
 * episode from a given seed is identical no matter how many ran before it.
 */
Observation env_reset(Env *env);

/* Advance by one CONTROL period, which is several physics steps (see
 * config.h). The action is held constant across them, exactly as a real
 * controller's output is held between its updates. */
StepResult env_step(Env *env, Action action);

/* Is the car's body overlapping a blocked cell right now? Exposed because the
 * renderer likes to say so, and because it is worth being able to test. */
int env_car_collides(const Env *env);

/* Fraction of a lap completed, 0..1+. For the progress line in the HUD. */
float env_lap_progress(const Env *env);

const char *done_reason_name(DoneReason reason);

#endif /* ENV_H */
