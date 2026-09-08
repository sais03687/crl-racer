#include "dynamics.h"

#include "mathf.h"

void car_reset(CarState *car, float x, float y, float heading, float speed)
{
    car->x = x;
    car->y = y;
    car->heading = heading;
    car->speed = speed;
}

void car_step(CarState *car, const Config *cfg,
              float steer_cmd, float target_speed, float dt)
{
    /* Clamp at the boundary, once. Everything downstream can then assume the
     * commands are physically achievable. */
    float delta  = clampf(steer_cmd, -cfg->max_steer_rad, cfg->max_steer_rad);
    float target = clampf(target_speed, 0.0f, cfg->max_speed_mps);

    /* --- Speed: approach the target, but only so fast ---------------------
     * The most the speed can change this step is max_accel * dt. If the target
     * is further away than that, we move by exactly that much; if it is
     * nearer, we arrive. Writing it as a clamp on the DIFFERENCE rather than
     * as separate accelerate/brake branches means the same line handles both,
     * and the car can never overshoot its target and oscillate around it. */
    float max_delta_v = cfg->max_accel_mps2 * dt;
    float dv = clampf(target - car->speed, -max_delta_v, max_delta_v);
    car->speed += dv;

    /* --- Pose: integrate the bicycle equations ---------------------------
     * The rates are evaluated using the speed we just updated. Using the new
     * speed rather than the old one is a small choice (it makes this a
     * semi-implicit Euler step) and it behaves slightly better under
     * acceleration, because position responds in the same step as the speed
     * change that caused it rather than one step late. */
    float v = car->speed;
    car->x += v * cosf(car->heading) * dt;
    car->y += v * sinf(car->heading) * dt;
    
    float delta_grip = atanf(cfg->max_lateral_accel_mps2 * cfg->wheelbase_m / (v * v)); //solving for delta_grip with the equation for lateral acceleration
    float delta_bound = fminf(cfg->max_steer_rad, delta_grip); //find the smaller of the 2 limits and use that as the bound
    float delta_eff = clampf(delta, -delta_bound, delta_bound); //using the delta_bound minimal limit from the last line in order to actually clamp it.
    
    car->heading += (v / cfg->wheelbase_m) * tanf(delta_eff) * dt;

    /* Keep the heading in (-pi, pi]. Not needed by sin/cos, which are happy
     * with any magnitude, but it keeps the number readable in the renderer and
     * stops it drifting toward the range where float spacing gets coarse after
     * a long run of laps. */
    car->heading = wrap_angle(car->heading);
}

void car_footprint(const CarState *car, const Config *cfg,
                   float *out_x, float *out_y)
{
    /* (x, y) is the rear axle. Place the body so the two axles sit
     * symmetrically inside it: the body's centre is half a wheelbase ahead of
     * the rear axle, which leaves equal overhangs front and rear. */
    float to_front = 0.5f * cfg->wheelbase_m + 0.5f * cfg->car_length_m;
    float to_rear = 0.5f * cfg->car_length_m - 0.5f * cfg->wheelbase_m;
    float half_w = 0.5f * cfg->car_width_m;

    /* Corners in the car's own frame: x forward, y to the left. */
    const float bx[4] = {to_front, to_front, -to_rear, -to_rear};
    const float by[4] = {half_w, -half_w, -half_w, half_w};

    float c = cosf(car->heading);
    float s = sinf(car->heading);

    /* Rotate into the world frame, then translate. This is the standard 2D
     * rotation; doing it in that order (rotate about the origin, then move)
     * is what keeps the car's shape rigid as it turns. */
    for (int i = 0; i < 4; i++) {
        out_x[i] = car->x + c * bx[i] - s * by[i];
        out_y[i] = car->y + s * bx[i] + c * by[i];
    }
}
