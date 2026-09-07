#ifndef RENDER_H
#define RENDER_H

#include "config.h"
#include "env.h"
#include "ftg.h"

/*
 * ============================================================================
 * ASCII RENDERER
 * ============================================================================
 *
 * Draws the world into the terminal, one character per viewport cell. There is
 * no graphics library here and no dependency beyond stdio -- the "display" is
 * a rectangle of chars that gets printed.
 *
 * This is not decoration. Almost every bug in a simulator like this is a bug
 * you can SEE in about two seconds and cannot find in an afternoon of reading
 * numbers: rays that stop short, a car facing backwards, a map flipped
 * vertically, a gap chosen on the wrong side. Numbers will not tell you the
 * map is upside down. A picture tells you instantly.
 *
 *      #########################
 *      #.........   ...........#      #  wall
 *      #......      *   ........#      .  a LiDAR ray, drawn to the exact
 *      #....    >        .......#         distance it reported
 *      #......      *   ........#      *  the ray Follow-The-Gap is aiming at
 *      #.........   ...........#      >  the car, pointing east
 *      #########################
 *
 * ---------------------------------------------------------------------------
 * Two decisions worth explaining
 * ---------------------------------------------------------------------------
 *
 * THE CAMERA FOLLOWS THE CAR. Drawing the whole 2000x2000 map into 100x38
 * characters would put the entire car and all its rays inside one character.
 * A window of a couple of dozen metres around the car shows the thing you
 * actually want to look at. The window stays axis-aligned with the world
 * (north is always up) rather than rotating with the car, because a rotating
 * view makes it very hard to tell whether the CAR turned or the WORLD did.
 *
 * TERMINAL CHARACTERS ARE NOT SQUARE. They are roughly twice as tall as they
 * are wide. If you give a row and a column the same number of metres, every
 * circle becomes an ellipse and, worse, the car's heading looks wrong -- a
 * 45 degree ray does not appear at 45 degrees. So a row covers
 * `render_char_aspect` times as many metres as a column does, which cancels
 * the distortion out.
 */

typedef struct Renderer {
    char *canvas;   /* rows * cols characters, row-major, no newlines stored */
    int   cols;
    int   rows;

    /* --- playback pacing ---
     * The simulation runs as fast as the CPU allows, which is around thirty
     * times faster than the car actually drives. To make that watchable
     * the renderer holds each frame back until real time has caught up with
     * simulated time. These fields remember where both clocks were when the
     * first frame went out, so every later frame is timed against that single
     * origin rather than against the previous frame -- which means a slow frame
     * does not push the whole rest of the lap late.
     *
     * The wall-clock origin is kept as integer seconds and nanoseconds, not as
     * a float. Seconds-since-1970 is around 1.7 billion, and a float has only
     * about seven significant digits, so storing it as a float would quantise
     * the clock to roughly two-minute steps. Subtracting first and converting
     * second keeps the numbers small enough for float to be exact. */
    int  pacing_started;
    long pace_wall_sec;
    long pace_wall_nsec;
    float pace_sim_start_s;
} Renderer;

int  renderer_init(Renderer *r, const Config *cfg);
void renderer_free(Renderer *r);

/*
 * Draw one frame and print it.
 *
 * `driver` may be NULL; if given, the beam it is aiming at is highlighted,
 * which is the fastest way to see whether the gap-finding is doing what you
 * think it is.
 */
void renderer_draw(Renderer *r, const Config *cfg, const Env *env,
                   const FollowTheGap *driver);

#endif /* RENDER_H */
