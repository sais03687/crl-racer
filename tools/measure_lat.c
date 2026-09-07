/* measure_lat.c -- how much grip is Follow-The-Gap actually asking for?
 *
 * Runs a lap and records the lateral acceleration the car demands at every
 * control step:   a_lat = v^2 * tan(steer) / wheelbase
 *
 * A real 1/10-scale car on a hard floor manages roughly 8-12 m/s^2 before
 * the tyres let go. Anything far above that is physics the simulator is
 * granting for free.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "env.h"
#include "ftg.h"

static int cmpf(const void *a, const void *b){
    float x = *(const float*)a, y = *(const float*)b;
    return (x > y) - (x < y);
}

int main(int argc, char **argv)
{
    if (argc < 4) { fprintf(stderr, "usage: %s map.yaml center.csv max_speed\n", argv[0]); return 1; }

    Config cfg = config_default();
    cfg.max_speed_mps = (float)atof(argv[3]);

    Env env; char err[256];
    if (!env_init(&env, &cfg, argv[1], argv[2], err, sizeof err)) {
        fprintf(stderr, "env_init: %s\n", err); return 1;
    }

    FollowTheGap ftg; ftg_init(&ftg, &cfg);
    Observation obs = env_reset(&env);

    int cap = 200000, n = 0;
    float *lat = malloc((size_t)cap * sizeof *lat);
    double sum = 0.0; float peak = 0.0f;

    for (int i = 0; i < cap; i++) {
        Action a = ftg_plan(&ftg, &cfg, &obs);

        /* Demand is evaluated at the speed the car is actually doing when the
         * command is applied, which is the speed in this observation. */
        float v = obs.speed_mps;
        float al = fabsf(v * v * tanf(a.steering_rad) / cfg.wheelbase_m);
        lat[n++] = al; sum += al; if (al > peak) peak = al;

        StepResult r = env_step(&env, a);
        obs = r.obs;
        if (r.done) {
            printf("outcome: %s\n",
                   r.reason == DONE_LAP ? "lap complete" :
                   r.reason == DONE_CRASH ? "crash" : "timeout");
            printf("lap time: %.3f s\n", env.sim_time_s);
            break;
        }
    }

    qsort(lat, (size_t)n, sizeof *lat, cmpf);
    printf("max speed cap      : %.1f m/s\n", cfg.max_speed_mps);
    printf("lateral accel mean : %6.2f m/s^2\n", sum / n);
    printf("               p50 : %6.2f\n", lat[n/2]);
    printf("               p90 : %6.2f\n", lat[(int)(n*0.90)]);
    printf("               p99 : %6.2f\n", lat[(int)(n*0.99)]);
    printf("               max : %6.2f\n", peak);

    /* How much of the lap is spent beyond what a real car could hold? */
    for (float thr = 8.0f; thr <= 12.1f; thr += 4.0f) {
        int over = 0;
        for (int i = 0; i < n; i++) if (lat[i] > thr) over++;
        printf("  %% of lap above %.0f m/s^2 : %5.1f%%\n", thr, 100.0 * over / n);
    }

    free(lat); env_free(&env);
    return 0;
}
