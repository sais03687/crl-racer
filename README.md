# crl-racer

Teaching a simulated racecar to drive, with everything written from scratch in C.

No PyTorch, no NumPy, no physics engine, no BLAS. The neural network, the
automatic differentiation, the optimiser, the reinforcement learning algorithm,
the vehicle physics and the LiDAR sensor model are all in this repository. The
only external dependency is the C standard library.

---

## What this is

A 1/10-scale car drives a real racing circuit. It cannot see the track — its
only input is a fan of laser range readings, the same sensor a real
[F1TENTH / RoboRacer](https://roboracer.ai) car uses. From those distances it
must decide how to steer and how fast to go.

Two drivers are being built against the same interface:

- **Follow-The-Gap**, a classical rule with no learning in it. Find the widest
  stretch of open space in the sensor readings and steer at it.
- **A PPO agent**, a neural network trained by trial and error over thousands
  of laps.

The point is the comparison. Follow-The-Gap is the honest baseline the learned
agent has to beat, and every result here is reported against it.

---

## Why write it in C

Because the parts that matter get hidden otherwise.

`model.fit()` is one line and teaches nothing about backpropagation.
`torch.optim.Adam` is an import. Calling PPO from a library means never
confronting why the clipped objective exists. Writing them means the ideas are
either understood or the code does not work.

C also happens to fit the problem. Training needs millions of simulated laps,
and simulation is a tight numerical loop with no allocation — the case where C
genuinely beats Python rather than merely matching it. Rollouts across threads
scale cleanly with no interpreter lock in the way.

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
same observation and return the same action, which is what makes the benchmark
a fair test of the algorithm rather than of two different setups.

---

## Status

| Phase | State |
|---|---|
| 1. Simulator, LiDAR, Follow-The-Gap | working |
| 2. Autograd engine with gradient checking | not started |
| 3. PPO with continuous steering | not started |
| 4. Multi-seed benchmarks and profiling | not started |

Follow-The-Gap currently laps Silverstone in **79.476 s**, reproducibly from a
fixed seed and identically across platforms. See "Known limitation" for why that
number is provisional.

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

Address and undefined-behaviour sanitizers are on by default. This is manual
memory management throughout, and at this scale they cost nothing.

Three circuits are included from
[`f1tenth_racetracks`](https://github.com/f1tenth/f1tenth_racetracks) —
Silverstone, IMS and Spielberg — downscaled to 1:10. Using real track geometry
means lap times are comparable to published results rather than self-graded.

---

## Layout

```
src/
  config.*      every tunable constant, in one struct
  grid.*        occupancy grid; world metres <-> cell indices
  waypoints.*   centerline loader and lap progress
  dynamics.*    kinematic bicycle model
  lidar.*       DDA grid marching -- the sensor
  ftg.*         Follow-The-Gap driver
  env.*         reset() / step(action) -- the seam
  render.*      ASCII terminal renderer
  rng.*         seeded PRNG, so runs reproduce exactly
  image.*       PNG decode      -- no external dependencies, so these are
  inflate.*     zlib inflate    -- written out. Plumbing, not the point.
tests/          100 unit tests
tools/
  measure_lat.c instrumentation harness for cornering forces
reference/
  grid_transforms.c   standalone worked example of the coordinate maths
```

---

## Known limitation

The physics is a kinematic bicycle model, which has no tyre model and so no grip
limit — the car can corner at any speed without sliding.

That makes the current baseline uninformative, and not for the obvious reason.
Measured with `tools/measure_lat.c`:

| Speed cap | Lap time | Avg speed | p99 lateral | % of lap over 8 m/s^2 |
|---|---|---|---|---|
| 6 m/s | 79.5 s | 5.76 | 9.1 | 2.3% |
| 8 m/s | 60.1 s | 7.62 | 16.1 | 15.2% |
| 12 m/s | 40.6 s | 11.27 | 47.3 | 35.1% |
| 16 m/s | 31.1 s | 14.74 | 94.0 | 55.6% |

A real 1/10-scale car holds about 10 m/s^2 before the tyres let go, so the upper
rows are physically impossible. But the deeper problem is that the speed cap is
doing the job grip should be doing. **A global speed limit punishes straights and
corners equally; grip only punishes corners.** That difference is the entire
racing line — braking for a corner to be quick on the straight after it. With a
global cap there is nothing to trade, so flat out is optimal, which is why
Follow-The-Gap averages ~95% of the cap at every setting and why a learned agent
could at best tie it.

The fix is to bound the steering angle by what the tyres can supply and raise the
speed cap so grip binds in corners instead. `src/dynamics.c` marks where.

---

## Roadmap

1. Grip limit in the physics, then re-measure the baseline
2. Gradient checker — finite differences against the backward pass, written
   before a single autograd operation is added
3. Autograd operations, batching, a two-headed policy-and-value network, Adam
4. PPO: rollout buffer, GAE advantages, clipped update, entropy bonus
5. Reward shaping and domain randomisation for sim-to-real robustness
6. Multi-seed benchmarks against Follow-The-Gap and against published
   `f1tenth_benchmarks` numbers
7. Threaded rollouts, then profile-guided optimisation

---

## Provenance

Phase 1 — the simulator, sensor and Follow-The-Gap — was scaffolded with AI
assistance to establish feasibility quickly and give the learning work something
to run against. It is deliberately the least conceptually interesting part: file
parsing, geometry and a well-known heuristic.

Everything from the grip fix onward is written by hand, including the LiDAR and
Follow-The-Gap reimplementations. The autograd engine and PPO especially, since
implementing them is the entire point.
