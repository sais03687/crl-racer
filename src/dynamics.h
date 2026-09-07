#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "config.h"

/*
 * ============================================================================
 * KINEMATIC BICYCLE MODEL
 * ============================================================================
 *
 * This is the physics of the car, and it is four numbers and four lines of
 * arithmetic. It is worth understanding exactly what it does and does not
 * claim, because the two words in its name are both doing work.
 *
 * ---------------------------------------------------------------------------
 * "BICYCLE": four wheels collapsed into two
 * ---------------------------------------------------------------------------
 *
 * A real car's two front wheels turn by slightly different amounts (the inner
 * wheel is on a tighter circle -- that is what Ackermann steering geometry is
 * for), and likewise for the rears. Tracking all four separately buys you
 * almost nothing for path planning, so the standard simplification is to merge
 * each axle into a single wheel on the centreline. You get a bicycle: one
 * steerable wheel at the front, one fixed wheel at the rear, `wheelbase_m`
 * apart.
 *
 *                    front wheel, turned by delta
 *                        /
 *                       o
 *                      /
 *                     /     <- wheelbase L
 *                    /
 *                   o------> heading
 *              rear wheel
 *          (x, y) lives here
 *
 * ---------------------------------------------------------------------------
 * "KINEMATIC": geometry only, no forces
 * ---------------------------------------------------------------------------
 *
 * The model assumes the wheels roll without ever sliding sideways. There is no
 * mass, no grip limit, no weight transfer, no tyre model. The rear wheel
 * therefore always travels exactly along the direction the car is pointing,
 * and the whole thing reduces to trigonometry.
 *
 * What that buys you: it is trivially stable, has no parameters to identify,
 * and cannot blow up numerically. What it costs you: the car CANNOT SKID. Take
 * a corner at 40 m/s and this model will calmly go round it. A real F1TENTH
 * car would understeer into the wall. The kinematic model is a good
 * approximation while the car is not near the limit of grip, which for a
 * 1/10-scale car on carpet is most of the time -- and the acceleration and
 * steering limits in Config are what keep us inside that regime.
 *
 * (The step up from here is the "dynamic" bicycle model, which adds slip
 * angles and tyre forces. It is a much bigger object and needs parameters you
 * have to measure on a real car. Not this phase.)
 *
 * ---------------------------------------------------------------------------
 * The equations
 * ---------------------------------------------------------------------------
 *
 *     dx/dt      = v * cos(theta)
 *     dy/dt      = v * sin(theta)
 *     dtheta/dt  = v / L * tan(delta)
 *     dv/dt      = acceleration, limited in magnitude
 *
 * The third one is the only non-obvious line. Geometry says the car traces a
 * circle of radius R = L / tan(delta) -- a straight-ahead wheel (delta = 0)
 * gives tan = 0 and R = infinity, and turning the wheel harder tightens the
 * circle. Angular velocity around a circle is v / R, and substituting R gives
 * v * tan(delta) / L. Note that it scales with v: at a standstill, turning the
 * steering wheel rotates the car not at all, which is correct and is why you
 * cannot spin this car on the spot.
 *
 * These are integrated with explicit Euler (new = old + rate * dt), the
 * simplest scheme there is. Its error grows with step size, and at the 5 ms
 * step this simulator uses that error is far below anything that matters here.
 * A fancier integrator (RK4) would be more accurate per step but is not the
 * limiting factor: the model's own assumptions are wronger than its
 * integration.
 */

typedef struct CarState {
    float x;         /* metres, world frame; the REAR AXLE, not the centre */
    float y;
    float heading;   /* radians, 0 = +x axis, increasing counter-clockwise */
    float speed;     /* metres per second, forward positive */
} CarState;

void car_reset(CarState *car, float x, float y, float heading, float speed);

/*
 * Advance the car by one physics step.
 *
 * `steer_cmd` and `target_speed` are what the DRIVER asked for; both are
 * clamped to the limits in Config here, at the boundary, rather than trusting
 * every caller to do it. A driver is free to output nonsense -- and a learned
 * one certainly will, early on -- and the physics must stay physical anyway.
 *
 * `target_speed` is a target, not a command: the car accelerates toward it at
 * no more than max_accel_mps2, so speed changes take time. That single
 * limitation is most of what makes the car feel like a vehicle rather than a
 * cursor, and it is why a driver that brakes only when it sees a wall will
 * still hit the wall.
 */
void car_step(CarState *car, const Config *cfg,
              float steer_cmd, float target_speed, float dt);

/*
 * The four corners of the car's rectangular footprint, in world coordinates,
 * written into `out_x[4]` / `out_y[4]` starting at the front-left and going
 * round.
 *
 * (x, y) is the rear axle, so the body extends forward from it. The collision
 * check in env.c uses these: testing a single point would let the car clip a
 * wall with its nose while its axle was still over clear road.
 */
void car_footprint(const CarState *car, const Config *cfg,
                   float *out_x, float *out_y);

#endif /* DYNAMICS_H */
