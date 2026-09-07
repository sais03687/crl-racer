/*
 * ============================================================================
 * ENTRY POINT
 * ============================================================================
 *
 * Wires the pieces together and runs the loop that is the heart of every
 * simulator of this kind:
 *
 *     observation = env.reset()
 *     until done:
 *         action = driver(observation)
 *         observation, reward, done = env.step(action)
 *
 * That is the whole program. Everything else in this file is argument parsing
 * and printing.
 *
 * Note what is NOT here: no timing against the wall clock, no sleeping, no
 * frame limiter. The simulation runs as fast as the machine can go and its
 * notion of time is purely `sim_time_s`, accumulated one physics step at a
 * time. That is what makes a run reproducible -- a simulator whose behaviour
 * depended on how fast your computer happened to be would give different
 * results on a laptop and a server.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "env.h"
#include "ftg.h"
#include "render.h"

static void usage(const char *program)
{
    printf(
        "usage: %s <map.yaml> <centerline.csv> [options]\n"
        "\n"
        "  Drives one lap of an F1TENTH track with the Follow-The-Gap driver\n"
        "  and prints the lap time.\n"
        "\n"
        "options:\n"
        "  --render               show the ASCII view (off by default)\n"
        "  --speed X              playback speed vs real time (default 1.0;\n"
        "                         0 = as fast as possible, 0.5 = half speed)\n"
        "  --view-width M         metres across the rendered viewport (default 26)\n"
        "  --frame-skip N         render every Nth control step (default 2)\n"
        "  --no-ansi              do not clear the screen between frames\n"
        "  --show-centerline      overlay the centerline waypoints\n"
        "  --seed N               PRNG seed (default 42)\n"
        "  --max-speed M          speed cap in m/s (default 6.0)\n"
        "  --lidar-noise M        per-beam noise stddev in metres (default 0)\n"
        "  --time-limit S         give up after S simulated seconds (default 300)\n"
        "  --quiet                suppress the periodic progress lines\n"
        "\n"
        "example:\n"
        "  %s maps/IMS/IMS_map.yaml maps/IMS/IMS_centerline.csv --render\n",
        program, program);
}

/*
 * Read the value that follows a flag.
 *
 * Returns NULL and complains if it is missing, so that `--seed` with nothing
 * after it is a clear error rather than a read past the end of argv.
 */
static const char *next_arg(int argc, char **argv, int *i, const char *flag)
{
    if (*i + 1 >= argc) {
        fprintf(stderr, "error: %s needs a value\n", flag);
        return NULL;
    }
    return argv[++(*i)];
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        usage(argv[0]);
        return argc < 2 ? 1 : 0;
    }

    const char *map_yaml       = argv[1];
    const char *centerline_csv = argv[2];

    /* Start from the defaults and let flags override individual fields. The
     * config struct stays the single source of truth; the command line is just
     * a way of editing it without recompiling. */
    Config cfg = config_default();
    int quiet = 0;

    for (int i = 3; i < argc; i++) {
        const char *a = argv[i];
        const char *v;

        if      (strcmp(a, "--render") == 0)           cfg.render_enabled = 1;
        else if (strcmp(a, "--no-ansi") == 0)          cfg.render_use_ansi_clear = 0;
        else if (strcmp(a, "--show-centerline") == 0)  cfg.render_show_centerline = 1;
        else if (strcmp(a, "--quiet") == 0)            quiet = 1;
        else if (strcmp(a, "--help") == 0)           { usage(argv[0]); return 0; }
        else if (strcmp(a, "--speed") == 0) {
            if (!(v = next_arg(argc, argv, &i, a))) return 1;
            cfg.render_speed = strtof(v, NULL);
        }
        else if (strcmp(a, "--view-width") == 0) {
            if (!(v = next_arg(argc, argv, &i, a))) return 1;
            cfg.render_view_width_m = strtof(v, NULL);
        } else if (strcmp(a, "--frame-skip") == 0) {
            if (!(v = next_arg(argc, argv, &i, a))) return 1;
            cfg.render_every_n_steps = (int)strtol(v, NULL, 10);
            if (cfg.render_every_n_steps < 1) cfg.render_every_n_steps = 1;
        } else if (strcmp(a, "--seed") == 0) {
            if (!(v = next_arg(argc, argv, &i, a))) return 1;
            cfg.rng_seed = strtoull(v, NULL, 10);
        } else if (strcmp(a, "--max-speed") == 0) {
            if (!(v = next_arg(argc, argv, &i, a))) return 1;
            cfg.max_speed_mps = strtof(v, NULL);
        } else if (strcmp(a, "--lidar-noise") == 0) {
            if (!(v = next_arg(argc, argv, &i, a))) return 1;
            cfg.lidar_noise_stddev_m = strtof(v, NULL);
        } else if (strcmp(a, "--time-limit") == 0) {
            if (!(v = next_arg(argc, argv, &i, a))) return 1;
            cfg.max_episode_time_s = strtof(v, NULL);
        } else {
            fprintf(stderr, "error: unknown option '%s'\n\n", a);
            usage(argv[0]);
            return 1;
        }
    }

    /* ---------------------------------------------------------------- setup */
    char err[512] = { 0 };
    Env env;
    if (!env_init(&env, &cfg, map_yaml, centerline_csv, err, sizeof err)) {
        fprintf(stderr, "error: %s\n", err);
        return 1;
    }

    FollowTheGap driver;
    if (!ftg_init(&driver, &cfg)) {
        fprintf(stderr, "error: out of memory setting up the driver\n");
        env_free(&env);
        return 1;
    }

    Renderer renderer;
    int have_renderer = 0;
    if (cfg.render_enabled) {
        if (!renderer_init(&renderer, &cfg)) {
            fprintf(stderr, "error: out of memory setting up the renderer\n");
            ftg_free(&driver);
            env_free(&env);
            return 1;
        }
        have_renderer = 1;
    }

    /* A short report of what was actually loaded. Worth printing every time:
     * a map that loaded with the wrong resolution, or a centerline that landed
     * on top of the walls, shows up here rather than as inexplicable driving
     * ten seconds later. */
    long free_cells, occupied_cells, unknown_cells;
    grid_count_states(&env.grid, &free_cells, &occupied_cells, &unknown_cells);

    printf("map          : %s\n", map_yaml);
    printf("  grid       : %d x %d cells at %.5f m/cell (%.1f x %.1f m)\n",
           env.grid.width, env.grid.height, (double)env.grid.resolution,
           env.grid.width * (double)env.grid.resolution,
           env.grid.height * (double)env.grid.resolution);
    printf("  origin     : (%.3f, %.3f) m, rotation %.3f rad\n",
           (double)env.grid.origin_x, (double)env.grid.origin_y,
           (double)env.grid.origin_theta);
    printf("  cells      : %ld free, %ld occupied, %ld unknown\n",
           free_cells, occupied_cells, unknown_cells);
    printf("centerline   : %s\n", centerline_csv);
    printf("  waypoints  : %d points, %.2f m around\n",
           env.wp.count, (double)env.wp.total_length);
    config_print(&cfg);
    printf("\n");

    /* ----------------------------------------------------------------- run */
    Observation obs = env_reset(&env);

    /* A car that starts inside a wall means the map and the centerline
     * disagree about their coordinate frame -- almost always a mismatched pair
     * of files. Say so plainly instead of reporting an instant crash and
     * leaving the reader to guess. */
    if (env_car_collides(&env)) {
        fprintf(stderr,
                "error: the car starts in contact with a wall at (%.2f, %.2f).\n"
                "       the map and centerline probably do not belong to the same track.\n",
                (double)env.car.x, (double)env.car.y);
        if (have_renderer) renderer_free(&renderer);
        ftg_free(&driver);
        env_free(&env);
        return 1;
    }

    float total_reward = 0.0f;
    int   steps        = 0;
    float next_report  = 0.0f;
    StepResult step;

    for (;;) {
        Action action = ftg_plan(&driver, &cfg, &obs);

        step = env_step(&env, action);
        obs  = step.obs;

        total_reward += step.reward;
        steps++;

        if (have_renderer && (steps % cfg.render_every_n_steps) == 0)
            renderer_draw(&renderer, &cfg, &env, &driver);

        if (step.done) break;

        /* Without the renderer there is otherwise no sign of life during a
         * thirty-second lap. One line every couple of simulated seconds is
         * enough to see it is progressing and roughly how fast. */
        if (!quiet && !have_renderer && env.sim_time_s >= next_report) {
            printf("  t %6.2fs  lap %5.1f%%  speed %5.2f m/s\n",
                   (double)env.sim_time_s,
                   (double)(env_lap_progress(&env) * 100.0f),
                   (double)env.car.speed);
            fflush(stdout);
            next_report = env.sim_time_s + 2.0f;
        }
    }

    /* Draw the final frame even if it fell between frame-skips, so the picture
     * on screen matches the outcome being reported below it. */
    if (have_renderer)
        renderer_draw(&renderer, &cfg, &env, &driver);

    /* -------------------------------------------------------------- result */
    printf("\n");
    printf("outcome      : %s\n", done_reason_name(step.reason));
    printf("  sim time   : %.3f s\n", (double)env.sim_time_s);
    printf("  distance   : %.2f m along the centerline (%.1f%% of a lap)\n",
           (double)env.distance_along_m, (double)(env_lap_progress(&env) * 100.0f));
    printf("  steps      : %d control steps\n", steps);
    printf("  reward     : %.2f total\n", (double)total_reward);

    int exit_code;
    if (step.reason == DONE_LAP) {
        printf("\nLAP TIME: %.3f s  (average speed %.2f m/s)\n",
               (double)env.sim_time_s,
               (double)(env.wp.total_length / env.sim_time_s));
        exit_code = 0;
    } else {
        printf("\nno lap completed (%s)\n", done_reason_name(step.reason));
        exit_code = 2;
    }

    if (have_renderer) renderer_free(&renderer);
    ftg_free(&driver);
    env_free(&env);

    return exit_code;
}
