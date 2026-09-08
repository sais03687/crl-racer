#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/*
 * ============================================================================
 * CONFIGURATION
 * ============================================================================
 *
 * A simulator is really just a big pile of numbers: how fast the car may go,
 * how many laser beams the sensor fires, how often the physics advances. If
 * those numbers get sprinkled through the code as literals, then changing one
 * means hunting through every file, and worse, you can never be sure you found
 * them all.
 *
 * So every tunable number in this project lives in this one struct. The rule
 * is absolute: if a value could reasonably be changed to study the car's
 * behaviour, it is a field here, and the code that needs it receives a
 * `const Config *`. No other file invents its own constant.
 *
 * Two payoffs. First, you can read this one header and know the entire
 * behavioural surface of the simulator. Second, an experiment -- "what happens
 * with 108 rays instead of 1080?" -- is a one-line change, not an archaeology
 * expedition.
 *
 * ---------------------------------------------------------------------------
 * A note on units, because mixing them up is the classic robotics bug:
 * every distance is in METRES, every angle in RADIANS, every time in SECONDS.
 * Field names carry the unit as a suffix (_m, _rad, _s, _mps) so you cannot
 * misread them at the call site. Degrees appear only where a human reads them.
 * ---------------------------------------------------------------------------
 */
typedef struct Config {
    /* ---------------- LiDAR ---------------- */
    int   ray_count;              /* number of rays per scan */
    float lidar_fov_rad;          /* total angular span, centred on heading */
    float lidar_max_range_m;      /* readings are clamped to this */
    float lidar_noise_stddev_m;   /* 0 keeps runs bit-identical; see rng.h */

    /* ---------------- Time ----------------
     * Two different clocks, and it matters that they are different.
     *
     * The PHYSICS clock advances the car's position. It has to tick fast, for
     * a reason specific to grid worlds: the walls on an F1TENTH map are drawn
     * as lines about one pixel thick (~0.06 m). If the car moved 0.10 m per
     * physics step, it could start on one side of a wall and finish on the
     * other with no step in between that was ever inside it -- the collision
     * check would see nothing and the car would drive through the barrier.
     * That failure is called tunnelling. The cure is to keep the per-step
     * movement smaller than one cell.
     *
     * The CONTROL clock is how often the driver is allowed to pick a new
     * steering angle and speed. Real controllers run far slower than physics
     * (a real car's computer cannot think 200 times a second), so the
     * simulator holds the last action constant across several physics steps.
     */
    float physics_dt_s;
    float control_hz;             /* driver is asked for a new action this often */

    /* ---------------- Vehicle ---------------- */
    float wheelbase_m;            /* front-to-rear axle distance, L in the model */
    float car_length_m;           /* footprint used for wall contact */
    float car_width_m;
    float max_steer_rad;          /* steering command is clamped to +/- this */
    float max_speed_mps;          /* target speed is clamped to [0, this] */
    float min_speed_mps;          /* floor the driver is allowed to request */
    float max_accel_mps2;         /* magnitude limit on |dv/dt|, both directions */
    float max_lateral_accel_mps2

    /* ---------------- Occupancy interpretation ----------------
     * A map is a greyscale image. Deciding which grey counts as "wall" is a
     * convention, and we follow the one ROS uses (see grid.h for the full
     * story). These two numbers are only fallbacks -- each map's YAML file
     * normally supplies its own. */
    float default_occupied_thresh;
    float default_free_thresh;
    int   treat_unknown_as_occupied;  /* cells between the two thresholds */

    /* ---------------- Environment ---------------- */
    int   start_waypoint;         /* centerline index the car resets onto */
    float start_lateral_jitter_m; /* reset noise, 0 by default (reproducible) */
    float start_heading_jitter_rad;
    float lap_fraction;           /* arc-length fraction counted as one lap */
    float max_episode_time_s;     /* safety net so a stuck car still terminates */
    int   nearest_search_window;  /* +/- waypoints searched around the last index */

    /* ---------------- Reward ----------------
     * The environment hands back a single number each step saying how well
     * that step went. Here it is simply "how many metres of track did you
     * advance", plus a large one-off bonus or penalty at the end. */
    float reward_progress_scale;
    float reward_crash;
    float reward_lap;

    /* ---------------- Follow-The-Gap driver ----------------
     * Knobs for the hand-written driver. See ftg.h for what each stage does. */
    float ftg_range_clip_m;       /* readings beyond this are treated as this */
    int   ftg_smooth_window;      /* mean filter half-width, in rays; 0 disables */
    float ftg_bubble_radius_m;    /* safety bubble drawn around the closest point */
    float ftg_gap_threshold_m;    /* a ray is "open" when it reads beyond this */
    float ftg_steer_gain;         /* scales the raw bearing to the target ray */

    /* ---------------- ASCII renderer ---------------- */
    int   render_enabled;         /* off by default, per spec */
    int   render_cols;
    int   render_rows;
    float render_view_width_m;    /* world width covered by the viewport */
    float render_char_aspect;     /* terminal cell height / width, keeps circles round */
    int   render_every_n_steps;   /* control steps between frames */
    int   render_use_ansi_clear;
    int   render_show_centerline;

    /* Playback speed, as a multiple of simulated time: 1.0 shows the lap at
     * the rate the car would really drive it, 0.5 is half speed, 2.0 double.
     *
     * 0 disables pacing entirely and the frames are printed as fast as the
     * machine can produce them -- which on this simulator is around 30x real
     * time, far too fast to watch, but the right setting when you are piping
     * frames to a file rather than looking at them.
     *
     * This affects ONLY when frames are printed. The simulation itself has no
     * idea what time it is; see the note in render.c. */
    float render_speed;

    /* ---------------- Reproducibility ---------------- */
    uint64_t rng_seed;
} Config;

/* Defaults tuned for the 1/10-scale F1TENTH platform on the published maps. */
Config config_default(void);

/* These are quantities you could compute from the fields above. They are
 * functions rather than extra struct fields on purpose: if `control_hz` were
 * stored alongside a precomputed `steps_per_control`, someone could change one
 * and forget the other, and the struct would quietly describe an impossible
 * simulator. Deriving on demand makes that mistake impossible. */
int   config_steps_per_control(const Config *c);
float config_control_dt_s(const Config *c);
float config_ray_angle_increment(const Config *c);

/* Where ray i points, as an angle relative to whichever way the car is facing.
 * Ray 0 is the far right (most negative angle), the middle ray looks straight
 * ahead, and the last ray is the far left. */
float config_ray_bearing(const Config *c, int i);

void config_print(const Config *c);

#endif /* CONFIG_H */
