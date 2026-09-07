#include "render.h"

#include "mathf.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* The character set, named so the drawing code below reads as intent rather
 * than as punctuation. */
#define CH_FREE       ' '
#define CH_WALL       '#'
#define CH_RAY        '.'
#define CH_TARGET_RAY '*'
#define CH_CENTERLINE '+'

int renderer_init(Renderer *r, const Config *cfg)
{
    memset(r, 0, sizeof *r);

    r->cols = cfg->render_cols  > 0 ? cfg->render_cols  : 80;
    r->rows = cfg->render_rows  > 0 ? cfg->render_rows  : 24;

    r->canvas = (char *)malloc((size_t)r->cols * (size_t)r->rows);
    if (!r->canvas) return 0;

    return 1;
}

void renderer_free(Renderer *r)
{
    free(r->canvas);
    r->canvas = NULL;
}

/* ------------------------------------------------------------------------- */
/* Playback pacing                                                            */
/* ------------------------------------------------------------------------- */

/*
 * Seconds elapsed since the (sec, nsec) instant passed in.
 *
 * timespec_get is the C11 standard wall clock -- no POSIX, no Windows API, so
 * this stays inside the libc-only rule the project is built under.
 *
 * The subtraction happens in integers, before anything becomes a float, for
 * the precision reason explained in render.h.
 */
static float seconds_since(long sec, long nsec)
{
    struct timespec now;
    if (!timespec_get(&now, TIME_UTC)) return 0.0f;

    return (float)(now.tv_sec - sec) + 1e-9f * (float)(now.tv_nsec - nsec);
}

/*
 * Hold this frame back until real time has caught up with simulated time.
 *
 * Two things worth being clear about.
 *
 * FIRST: this does not touch the simulation. The physics has already run; all
 * that is being delayed is the printing. Nothing the car does depends on how
 * long this function waits, so a paced run and an unpaced run produce
 * identical results -- which is exactly the property you want, because a
 * simulator whose outcome changed when you watched it would be useless.
 *
 * SECOND: it busy-waits, and that is a real cost -- one core spins for as long
 * as you are watching. It is done this way because C11 has no portable sleep.
 * thrd_sleep is part of the optional threads library, which plenty of
 * toolchains do not ship (mingw-w64 among them), and Sleep/nanosleep are
 * platform APIs rather than standard C. Given the choice between a dependency
 * and a spinning loop in a mode you opt into deliberately to watch an
 * animation, the spinning loop is the smaller price. If this ever needs to run
 * unattended for a long time, that is the trade to revisit.
 */
static void pace_frame(Renderer *r, const Config *cfg, const Env *env)
{
    if (cfg->render_speed <= 0.0f) return;   /* pacing disabled: print flat out */

    if (!r->pacing_started) {
        /* First frame: start both clocks here and return immediately. There is
         * nothing to wait for yet, and anchoring on the first frame rather
         * than on program start means map loading does not count against the
         * playback. */
        struct timespec now;
        if (timespec_get(&now, TIME_UTC)) {
            r->pace_wall_sec  = (long)now.tv_sec;
            r->pace_wall_nsec = (long)now.tv_nsec;
        }
        r->pace_sim_start_s = env->sim_time_s;
        r->pacing_started   = 1;
        return;
    }

    /* When this frame is due, in real seconds since the first frame. */
    float target = (env->sim_time_s - r->pace_sim_start_s) / cfg->render_speed;

    /* If the machine has fallen behind, target is already in the past and this
     * loop does not run at all -- playback simply degrades to as-fast-as-
     * possible rather than accumulating a debt it tries to pay back later. */
    while (seconds_since(r->pace_wall_sec, r->pace_wall_nsec) < target) {
        /* spin */
    }
}

/* ------------------------------------------------------------------------- */
/* Viewport arithmetic                                                        */
/* ------------------------------------------------------------------------- */

/*
 * Everything the renderer needs to convert between world metres and canvas
 * characters, computed once per frame. Gathering it into a struct keeps the
 * five drawing passes below from each recomputing (and each getting subtly
 * wrong) the same conversion.
 */
typedef struct {
    float centre_x, centre_y;   /* world point at the middle of the canvas */
    float m_per_col;
    float m_per_row;
    float half_cols;
    float half_rows;
} View;

static View view_for(const Renderer *r, const Config *cfg, const Env *env)
{
    View v;
    v.centre_x  = env->car.x;
    v.centre_y  = env->car.y;
    v.m_per_col = cfg->render_view_width_m / (float)r->cols;
    /* Taller characters cover more world per row -- see the header. */
    v.m_per_row = v.m_per_col * cfg->render_char_aspect;
    v.half_cols = 0.5f * (float)(r->cols - 1);
    v.half_rows = 0.5f * (float)(r->rows - 1);
    return v;
}

/* Canvas cell -> the world point at its centre. */
static void cell_to_world(const View *v, int col, int row, float *wx, float *wy)
{
    *wx = v->centre_x + ((float)col - v->half_cols) * v->m_per_col;
    /* Minus, because canvas rows increase downward while world y increases
     * upward. Same flip as in grid.c, for the same reason. */
    *wy = v->centre_y - ((float)row - v->half_rows) * v->m_per_row;
}

/* World point -> canvas cell. Returns 0 if it falls outside the canvas. */
static int world_to_cell(const View *v, const Renderer *r,
                         float wx, float wy, int *col, int *row)
{
    float fc = (wx - v->centre_x) / v->m_per_col + v->half_cols;
    float fr = (v->centre_y - wy) / v->m_per_row + v->half_rows;

    /* Round to nearest rather than truncating, so a point lands in the
     * character whose centre it is closest to. Truncation would bias every
     * plotted point down and left by half a character, which is enough to make
     * the car look like it is driving through the wall it is next to. */
    int c = (int)floorf(fc + 0.5f);
    int rr = (int)floorf(fr + 0.5f);

    if (c < 0 || c >= r->cols || rr < 0 || rr >= r->rows) return 0;

    *col = c;
    *row = rr;
    return 1;
}

static void put(Renderer *r, int col, int row, char ch)
{
    r->canvas[(size_t)row * (size_t)r->cols + (size_t)col] = ch;
}

static char get(const Renderer *r, int col, int row)
{
    return r->canvas[(size_t)row * (size_t)r->cols + (size_t)col];
}

/* ------------------------------------------------------------------------- */
/* Drawing passes                                                             */
/* ------------------------------------------------------------------------- */

/*
 * Pass 1: the map.
 *
 * For each character on screen, ask what is at the world point it represents.
 * Note this SAMPLES the grid rather than averaging over the cells a character
 * covers: at typical zoom one character spans several grid cells, so a
 * one-cell-thick wall can fall between two samples and the wall appears to
 * have gaps in it. That is a display artefact only -- the physics and the
 * LiDAR never sample; they walk every cell. Zoom in (a smaller
 * render_view_width_m) and the walls become solid.
 */
static void draw_map(Renderer *r, const Config *cfg, const Env *env, const View *v)
{
    for (int row = 0; row < r->rows; row++) {
        for (int col = 0; col < r->cols; col++) {
            float wx, wy;
            cell_to_world(v, col, row, &wx, &wy);
            put(r, col, row,
                grid_is_blocked_world(&env->grid, cfg, wx, wy) ? CH_WALL : CH_FREE);
        }
    }
}

/* Pass 2: the centerline, if asked for. Off by default -- it is useful when
 * checking that the CSV and the map really do share a coordinate frame, and
 * clutter the rest of the time. */
static void draw_centerline(Renderer *r, const Env *env, const View *v)
{
    for (int i = 0; i < env->wp.count; i++) {
        int col, row;
        if (world_to_cell(v, r, env->wp.x[i], env->wp.y[i], &col, &row))
            if (get(r, col, row) == CH_FREE)
                put(r, col, row, CH_CENTERLINE);
    }
}

/*
 * Pass 3: the rays.
 *
 * Each ray is drawn from the car out to exactly the distance it reported, so
 * the picture shows what the sensor SAW, not what is there. A ray that stops
 * in mid-air is a ray that hit something -- and if it stops in mid-air where
 * there is visibly no wall, the ray caster has a bug and you have just found
 * it by looking.
 *
 * Ray marks never overwrite walls, so the map stays readable underneath.
 */
static void draw_rays(Renderer *r, const Config *cfg, const Env *env,
                      const FollowTheGap *driver, const View *v)
{
    /* Step half a character at a time so the drawn line has no holes in it. */
    float step = 0.5f * fminf(v->m_per_col, v->m_per_row);
    if (step <= 0.0f) return;

    int target = driver ? driver->target_ray : -1;

    for (int i = 0; i < cfg->ray_count; i++) {
        float angle = env->car.heading + config_ray_bearing(cfg, i);
        float range = env->scan[i];

        float dx = cosf(angle);
        float dy = sinf(angle);
        char  mark = (i == target) ? CH_TARGET_RAY : CH_RAY;

        for (float t = 0.0f; t <= range; t += step) {
            int col, row;
            if (!world_to_cell(v, r, env->car.x + dx * t, env->car.y + dy * t,
                               &col, &row))
                break;   /* left the viewport; the rest of this ray is off-screen */

            char existing = get(r, col, row);
            if (existing == CH_FREE || existing == CH_CENTERLINE ||
                (existing == CH_RAY && mark == CH_TARGET_RAY))
                put(r, col, row, mark);
        }
    }
}

/* Pass 4: the car, drawn as an arrow so its heading is visible at a glance. */
static void draw_car(Renderer *r, const Env *env, const View *v)
{
    /* Eight compass directions is all ASCII can express. Snap the heading to
     * the nearest one: shift by half a sector before dividing so that, for
     * example, anything within 22.5 degrees of east reads as '>'. */
    static const char ARROWS[8] = { '>', '/', '^', '\\', '<', '/', 'v', '\\' };

    float a = env->car.heading;
    while (a < 0.0f) a += TWO_PI_F;

    int sector = (int)((a + PI_F / 8.0f) / (PI_F / 4.0f)) & 7;

    int col, row;
    if (world_to_cell(v, r, env->car.x, env->car.y, &col, &row))
        put(r, col, row, ARROWS[sector]);
}

/* ------------------------------------------------------------------------- */
/* Frame                                                                      */
/* ------------------------------------------------------------------------- */

void renderer_draw(Renderer *r, const Config *cfg, const Env *env,
                   const FollowTheGap *driver)
{
    /* Wait before drawing rather than after, so the frame already on screen
     * stays up for its full duration instead of being replaced early. */
    pace_frame(r, cfg, env);

    View v = view_for(r, cfg, env);

    draw_map(r, cfg, env, &v);
    if (cfg->render_show_centerline) draw_centerline(r, env, &v);
    draw_rays(r, cfg, env, driver, &v);
    draw_car(r, env, &v);

    /* Move the cursor home and clear, so successive frames overwrite each
     * other in place instead of scrolling past. If your terminal shows the
     * escape codes as literal text, set render_use_ansi_clear to 0. */
    if (cfg->render_use_ansi_clear)
        fputs("\x1b[H\x1b[2J", stdout);

    /* One fwrite per row, then a newline. The canvas holds no newlines of its
     * own so that the index arithmetic above stays a plain rows-by-cols. */
    for (int row = 0; row < r->rows; row++) {
        fwrite(r->canvas + (size_t)row * (size_t)r->cols, 1, (size_t)r->cols, stdout);
        fputc('\n', stdout);
    }

    /* The closest reading is the number that predicts a crash, so it earns a
     * place on the heads-up display next to the state. */
    float closest = env->scan[0];
    for (int i = 1; i < cfg->ray_count; i++)
        if (env->scan[i] < closest) closest = env->scan[i];

    printf("t %6.2fs | speed %5.2f m/s | heading %+7.1f deg | lap %5.1f%% |"
           " nearest obstacle %5.2f m | %s\n",
           (double)env->sim_time_s,
           (double)env->car.speed,
           (double)rad_to_deg(env->car.heading),
           (double)(env_lap_progress(env) * 100.0f),
           (double)closest,
           done_reason_name(env->reason));

    fflush(stdout);
}
