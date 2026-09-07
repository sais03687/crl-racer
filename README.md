# crl-racer

Reinforcement learning from scratch in C, on a 2D simulator modelled on the
[F1TENTH / RoboRacer](https://roboracer.ai) platform.

No PyTorch, no NumPy, no physics library. The autograd engine, the optimiser,
PPO, the simulator and the LiDAR model are all written here. The only external
dependency is libc.

**Goal:** train an agent that beats a classical baseline on lap time, and be
able to explain every line of why.

---

## Status

Phase 1 is working. Follow-The-Gap drives a full lap of Silverstone in
**79.476 s** with a 6 m/s speed cap, reproducibly from a fixed seed.

That lap time is **not yet a useful baseline** — see "The open problem" below.

| Phase | State |
|---|---|
| 1. Simulator and a non-learning driver | working, needs the grip fix |
| 2. Autograd engine you can trust | not started |
| 3. PPO | not started |
| 4. Benchmarks and optimisation | not started |

---

## Build and run

```sh
make                 # builds build/racer and build/run_tests
make test            # 100 unit tests

./build/racer maps/Silverstone/Silverstone_map.yaml \
              maps/Silverstone/Silverstone_centerline.csv

./build/racer maps/IMS/IMS_map.yaml \
              maps/IMS/IMS_centerline.csv --render
```

Sanitizers (`-fsanitize=address,undefined`) are on by default. Keep them on —
the whole thing is manual memory management and they cost nothing at this scale.

Three tracks are included, taken from
[`f1tenth_racetracks`](https://github.com/f1tenth/f1tenth_racetracks):
Silverstone, IMS and Spielberg, each downscaled to 1:10.

---

## The open problem

The physics is a kinematic bicycle model, which has no tyre model and therefore
no grip limit. The car can corner at any speed without sliding.

Measured with `tools/measure_lat.c`, which records the lateral acceleration
Follow-The-Gap demands at every control step. A real 1/10-scale car holds
roughly 10 m/s² before the tyres let go:

| Speed cap | Lap time | Avg speed | p99 lateral | Peak lateral | % of lap over 8 m/s² |
|---|---|---|---|---|---|
| 6 m/s | 79.5 s | 5.76 | 9.1 | 13.7 | 2.3% |
| 8 m/s | 60.1 s | 7.62 | 16.1 | 35.9 | 15.2% |
| 12 m/s | 40.6 s | 11.27 | 47.3 | 113.1 | 35.1% |
| 16 m/s | 31.1 s | 14.74 | 94.0 | 281.6 | 55.6% |

281 m/s² is about 29g.

The deeper issue is not that the numbers are unphysical. It is that **the speed
cap is doing the job grip should be doing.** A global speed limit punishes
straights and corners equally; grip only punishes corners. That difference is
the entire racing line — braking for a corner so you can be quick on the
straight after it. With a global cap there is nothing to trade, so flat out is
optimal, which is why Follow-The-Gap averages ~95% of the cap at every setting.

An RL agent could at best tie a fifty-line heuristic. There is no racing line
to learn.

**The fix:** cap the steering angle by what the tyres can hold,

```
delta_grip = atan(a_max * L / v^2)
```

then raise the speed cap so grip becomes the binding constraint in corners
rather than the global limit. See the TODO in `src/dynamics.c`.

Follow-The-Gap may stop completing laps once this lands, because it is purely
reactive and cannot brake for a corner it has not yet seen. That is not a
regression — it is the first evidence the baseline is beatable.

---

## Layout

```
src/
  config.*      every tunable constant, in one struct
  grid.*        occupancy grid, world <-> cell transforms
  image.*       PNG decode  \  no external deps, so these are
  inflate.*     zlib inflate /  written out. Not worth reading.
  waypoints.*   centerline loader, lap progress
  dynamics.*    kinematic bicycle model          <- the grip fix goes here
  lidar.*       DDA ray marching
  ftg.*         Follow-The-Gap driver
  env.*         reset() / step(action)
  render.*      ASCII terminal renderer
  rng.*         seeded PRNG, for reproducible runs
tests/          100 unit tests
tools/
  measure_lat.c lateral acceleration harness
reference/
  grid_transforms.c   standalone worked example of the coordinate
                      transforms, with its own tests
```

The `env` layer is the seam. Everything on the simulator side of it knows
nothing about drivers, and everything on the driver side knows nothing about
physics. That is what will let a PPO policy drop in where Follow-The-Gap is now.

---

## Next

1. Grip limit in `car_step`, plus three tests (grip binds at speed, mechanical
   limit binds when slow, no NaN at rest)
2. Re-measure the baseline with `tools/measure_lat.c` — p99 should land at or
   just under 8.0
3. Rewrite `lidar.c` and `ftg.c` from scratch, without reading the current ones
4. Gradient checker, before adding a single autograd op

---

## Notes

Phase 1 was scaffolded quickly to establish feasibility and give the learning
work something to run against. Everything from the grip fix onward is written
by hand. The engine and PPO in particular are the point of the project, and
generating them would defeat it.
