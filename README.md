# crl-racer

Teaching a simulated racecar to drive, with everything written from scratch in C.

No PyTorch, no NumPy, no physics engine, no BLAS. The neural network, the
automatic differentiation, the optimizer, the reinforcement learning algorithm,
the vehicle physics and the LiDAR sensor model are all in this repository. The
only external dependency is the C standard library.

---

## What this is

A 1/10-scale car drives a real racing circuit. It cannot see the track. Its only
input is a fan of laser range readings, the same sensor a real
[F1TENTH / RoboRacer](https://roboracer.ai) car uses. From those distances it has
to decide how to steer and how fast to go.

Two drivers are being built against the same interface:

- **Follow-The-Gap**, a classical rule with no learning in it. Find the widest
  stretch of open space in the sensor readings and steer at it.
- **A PPO agent**, a neural network trained by trial and error over thousands of
  laps.

The point is the comparison. Follow-The-Gap is the honest baseline the learned
agent has to beat, and every result here is reported against it.

---

## Why write it in C

Because the parts that matter get hidden otherwise.

`model.fit()` is one line and teaches nothing about backpropagation.
`torch.optim.Adam` is an import. Calling PPO from a library means never having to
confront why the clipped objective exists. Writing them means the ideas are
either understood or the code does not work.

C also fits the problem. Training needs millions of simulated laps, and
simulation is a tight numerical loop with no allocation in it. That is the case
where C actually beats Python instead of just matching it. Rollouts across
threads scale cleanly with no interpreter lock in the way.

---

## Architecture

```
       simulator                env                 driver
  +------------------+      +---------+      +----------------+
  | track   -> grid  |      | reset() |      | Follow-The-Gap |
  | physics -> pose  | ---> | step()  | ---> |       or       |
  | LiDAR   -> range | <--- |         | <--- |  PPO policy    |
  +------------------+      +---------+      +----------------+
                                 |                    ^
                            transitions          new weights
                                 v                    |
                          +--------------+   +----------------+
                          | PPO trainer  |-->| autograd engine|
                          +--------------+   +----------------+
```

`env` is the seam. Nothing on the simulator side knows a driver exists, and
nothing on the driver side knows anything about physics. Both drivers take the
same observation and return the same action. That is what makes the benchmark a
fair test of the algorithm instead of a test of two different setups.

---

## Status

| Phase | State |
|---|---|
| 1. Simulator, LiDAR, Follow-The-Gap | done |
| 2. Autograd engine with gradient checking | not started |
| 3. PPO with continuous steering | not started |
| 4. Multi-seed benchmarks and profiling | not started |

Follow-The-Gap laps Silverstone in **79.660 s** with a 6 m/s speed cap. The run
is reproducible from a fixed seed and gives the same time on different machines.
That is the baseline every learned agent gets measured against.

---

## Build and run

```sh
make                 # build/racer and build/run_tests
make test            # 100 unit tests

./build/racer maps/Silverstone/Silverstone_map.yaml \
              maps/Silverstone/Silverstone_centerline.csv

# watch it drive, with every LiDAR ray drawn
./build/racer maps/IMS/IMS_map.yaml \
              maps/IMS/IMS_centerline.csv --render
```

Address and undefined behavior sanitizers are on by default. This is manual
memory management throughout, and at this scale they cost nothing.

Three circuits are included from
[`f1tenth_racetracks`](https://github.com/f1tenth/f1tenth_racetracks):
Silverstone, IMS and Spielberg, each downscaled to 1:10. Using real track
geometry means lap times can be compared to published results instead of being
self-graded.

---

## Layout

```
src/
  config.*      every tunable constant, in one struct
  grid.*        occupancy grid; world meters <-> cell indices
  waypoints.*   centerline loader and lap progress
  dynamics.*    kinematic bicycle model, with a grip limit
  lidar.*       DDA grid marching, the sensor
  ftg.*         Follow-The-Gap driver
  env.*         reset() / step(action), the seam
  render.*      ASCII terminal renderer
  rng.*         seeded PRNG, so runs reproduce exactly
  image.*       PNG decode      -- no external dependencies, so these are
  inflate.*     zlib inflate    -- written out. Plumbing, not the point.
tests/          100 unit tests
tools/
  measure_lat.c instrumentation harness for cornering forces
reference/
  grid_transforms.c   standalone worked example of the coordinate math
```

---

## Grip, and why the baseline has a ceiling

The physics started as a kinematic bicycle model. That model has no tires in it,
so there was no grip limit and the car could corner at any speed without sliding.
Measured with `tools/measure_lat.c`, Follow-The-Gap was demanding up to
281 m/s2 of lateral acceleration. That is about 29g. A real 1/10-scale car holds
around 10.

The bigger problem was not that the numbers were unphysical. It was that the
speed cap was doing the job grip should be doing. **A global speed limit punishes
straights and corners equally. Grip only punishes corners.** That difference is
the whole racing line: you brake for a corner so you can be fast on the straight
after it. With a global cap there was nothing to trade, so driving flat out was
already optimal. Follow-The-Gap averaged about 95% of the cap at every setting,
which means a learned agent could at best tie a fifty line heuristic.

Steering is now bounded by what the tires can supply:

    delta_grip = atan(a_max * L / v^2)

The v^2 in the denominator means the bound shrinks with the square of speed. That
is why a corner has a right speed. Peak demand now lands at exactly 8.00 m/s2,
which is the configured limit.

| Speed cap | Result | Lap time |
|---|---|---|
| 6 m/s | lap complete | 79.660 s |
| 8 m/s | wall contact | — |
| 12 m/s | wall contact | — |

**Follow-The-Gap now fails above 6 m/s, and that is the useful result.** It is
purely reactive. It only slows down once it is already steering hard, so it
cannot brake for a corner it has not reached yet. It arrives too fast,
understeers, and runs wide into the wall.

That ceiling is structural, not a tuning problem. It is exactly what a learned
agent can break through by looking ahead. The premise of this project is now
measured instead of assumed.

---

## Roadmap

1. Reimplement `lidar.c` and `ftg.c` by hand, without reading the scaffolded ones
2. Gradient checker: finite differences against the backward pass, written before
   a single autograd operation is added
3. Autograd operations, batching, a two-headed policy and value network, Adam
4. PPO: rollout buffer, GAE advantages, clipped update, entropy bonus
5. Reward shaping and domain randomization for sim-to-real robustness
6. Multi-seed benchmarks against Follow-The-Gap and against published
   `f1tenth_benchmarks` numbers
7. Threaded rollouts, then profile-guided optimization

---

## Provenance

Phase 1, meaning the simulator, sensor and Follow-The-Gap, was scaffolded with AI
assistance to establish feasibility quickly and give the learning work something
to run against. It is deliberately the least conceptually interesting part: file
parsing, geometry and a well-known heuristic.

Everything from the grip fix onward is written by hand, including the LiDAR and
Follow-The-Gap reimplementations. The autograd engine and PPO especially, since
implementing them is the entire point.
