#include "config.h"

#include "mathf.h"

#include <stdio.h>

Config config_default(void)
{
    Config c;

    /* These numbers are copied from the Hokuyo UST-10LX, the laser scanner
     * bolted to the real F1TENTH car: it sweeps 270 degrees, reports 1080
     * samples per sweep, and sees about 10 m. Matching the real hardware is a
     * deliberate choice -- a driver tuned against a fantasy sensor with a
     * 360-degree view and infinite range would not transfer to the car. */
    c.ray_count            = 1080;
    c.lidar_fov_rad        = 4.712389f;   /* 270 degrees */
    c.lidar_max_range_m    = 10.0f;
    c.lidar_noise_stddev_m = 0.0f;

    /* 200 Hz physics against 50 Hz control. At the 6 m/s speed cap the car
     * advances 0.03 m per physics step, comfortably under the ~0.06 m cell
     * size of the published maps, so the swept path cannot jump over a wall
     * that is only one pixel thick. */
    c.physics_dt_s = 0.005f;
    c.control_hz   = 50.0f;

    /* Published F1TENTH chassis geometry. */
    c.wheelbase_m    = 0.33f;
    c.car_length_m   = 0.58f;
    c.car_width_m    = 0.31f;
    c.max_steer_rad  = 0.4189f;   /* 24 degrees */
    c.max_speed_mps  = 12.0f;
    c.min_speed_mps  = 1.5f;
    c.max_accel_mps2 = 6.0f;
    c.max_lateral_accel_mps2 = 8.0f;

    /* map_server's defaults; the YAML overrides these per map. */
    c.default_occupied_thresh   = 0.65f;
    c.default_free_thresh       = 0.196f;
    c.treat_unknown_as_occupied = 1;

    c.start_waypoint           = 0;
    c.start_lateral_jitter_m   = 0.0f;
    c.start_heading_jitter_rad = 0.0f;
    c.lap_fraction             = 1.0f;
    c.max_episode_time_s       = 300.0f;
    /* To measure progress we keep asking "which centerline point is the car
     * nearest to?". We search only a window of points around the previous
     * answer, not the whole track. That is not for speed -- 800 points is
     * nothing. It is for correctness: real circuits pass close to themselves
     * (think of a hairpin doubling back), so a global nearest-point search can
     * suddenly decide the car has teleported half a lap forward. Restricting
     * the search to points adjacent to where the car already was makes the
     * answer follow the car continuously. */
    c.nearest_search_window = 40;

    c.reward_progress_scale = 1.0f;
    c.reward_crash          = -100.0f;
    c.reward_lap            = 100.0f;

    c.ftg_range_clip_m    = 6.0f;
    c.ftg_smooth_window   = 2;
    c.ftg_bubble_radius_m = 0.35f;
    c.ftg_gap_threshold_m = 2.0f;
    c.ftg_steer_gain      = 1.0f;

    c.render_enabled         = 0;   /* off by default, as specified */
    c.render_cols            = 100;
    c.render_rows            = 38;
    c.render_view_width_m    = 26.0f;
    c.render_char_aspect     = 2.0f;
    c.render_every_n_steps   = 2;
    c.render_use_ansi_clear  = 1;
    c.render_show_centerline = 0;
    /* Real time. The renderer is off unless you ask for it, and asking for it
     * means you intend to watch, so watchable is the right default. It costs
     * nothing when rendering is off, because nothing reads this field then. */
    c.render_speed           = 1.0f;

    c.rng_seed = 42u;

    return c;
}

float config_control_dt_s(const Config *c)
{
    return 1.0f / c->control_hz;
}

int config_steps_per_control(const Config *c)
{
    int n = (int)(config_control_dt_s(c) / c->physics_dt_s + 0.5f);
    return n < 1 ? 1 : n;
}

float config_ray_angle_increment(const Config *c)
{
    /* Both endpoints of the field of view are sampled, so n rays span n-1 gaps. */
    if (c->ray_count <= 1) return 0.0f;
    return c->lidar_fov_rad / (float)(c->ray_count - 1);
}

float config_ray_bearing(const Config *c, int i)
{
    if (c->ray_count <= 1) return 0.0f;
    return -0.5f * c->lidar_fov_rad + (float)i * config_ray_angle_increment(c);
}

void config_print(const Config *c)
{
    printf("config:\n");
    printf("  lidar        : %d rays, fov %.1f deg, max range %.2f m, noise %.3f m\n",
           c->ray_count, rad_to_deg(c->lidar_fov_rad),
           (double)c->lidar_max_range_m, (double)c->lidar_noise_stddev_m);
    printf("  timing       : physics %.4f s (%.0f Hz), control %.0f Hz, %d substeps\n",
           (double)c->physics_dt_s, 1.0 / (double)c->physics_dt_s,
           (double)c->control_hz, config_steps_per_control(c));
    printf("  vehicle      : L %.2f m, body %.2f x %.2f m, steer +/-%.1f deg,"
           " speed [%.1f, %.1f] m/s, accel %.1f m/s^2\n",
           (double)c->wheelbase_m, (double)c->car_length_m, (double)c->car_width_m,
           rad_to_deg(c->max_steer_rad),
           (double)c->min_speed_mps, (double)c->max_speed_mps, (double)c->max_accel_mps2);
    printf("  follow-gap   : clip %.1f m, smooth %d, bubble %.2f m, gap thresh %.1f m,"
           " steer gain %.2f\n",
           (double)c->ftg_range_clip_m, c->ftg_smooth_window,
           (double)c->ftg_bubble_radius_m, (double)c->ftg_gap_threshold_m,
           (double)c->ftg_steer_gain);
    printf("  seed         : %llu\n", (unsigned long long)c->rng_seed);
}
