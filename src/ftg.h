#ifndef FTG_H
#define FTG_H

#include "config.h"
#include "env.h"

/*
 * ============================================================================
 * FOLLOW-THE-GAP
 * ============================================================================
 *
 * A driver that reads a laser scan and outputs a steering angle and a speed.
 * It does not know where it is, has no map, keeps no memory between steps, and
 * has no model of the track. Each call looks at one scan and decides where to
 * point. That it can drive a Formula 1 circuit at all is the interesting part.
 *
 * The idea in one sentence: find the widest stretch of open space in front of
 * you, aim at the middle of it, and slow down in proportion to how hard you
 * are turning.
 *
 * ---------------------------------------------------------------------------
 * The five stages
 * ---------------------------------------------------------------------------
 *
 * 1. CLIP AND SMOOTH. Readings beyond a few metres are all equally "far
 *    enough" -- caring about the difference between 8 m and 10 m of clear road
 *    just adds noise to the decision, so everything past a threshold is
 *    flattened to that threshold. Then a small moving average is taken over
 *    neighbouring beams, so that one freak reading cannot single-handedly
 *    conjure or destroy a gap.
 *
 * 2. FIND THE CLOSEST POINT. Whatever is nearest is the thing most likely to
 *    be hit, so it anchors the next stage.
 *
 * 3. DRAW A SAFETY BUBBLE. Zero out every reading within some radius of that
 *    closest point -- not a fixed number of beams, but however many beams
 *    subtend that radius at that distance.
 *
 *    This is the stage that makes the algorithm work, and it is the one worth
 *    thinking about. Without it, consider a doorway: the beams that pass
 *    through the opening read long, so the gap looks wide and inviting, and
 *    the car aims at its centre -- and clips the door frame with a wheel,
 *    because the algorithm was steering a POINT through a gap while driving a
 *    car that has width. The bubble erases the space immediately around the
 *    nearest obstacle, which shrinks every gap by roughly the clearance you
 *    need, so aiming at the middle of what remains keeps the whole car out of
 *    trouble. It is how a point-like planner is made safe for a body.
 *
 *    Note that the bubble is angular, not a fixed beam count. A 0.35 m bubble
 *    around something 5 m away covers a handful of beams; the same bubble
 *    around something 0.4 m away covers a huge arc. That is exactly right --
 *    the closer a thing is, the more of your options it should remove.
 *
 * 4. FIND THE LONGEST GAP. Scan for the longest unbroken run of beams still
 *    reading beyond the gap threshold. Longest, not nearest and not deepest:
 *    the widest opening is the one with the most room for error.
 *
 * 5. AIM AND SET SPEED. Steer toward the middle of that run, and set speed
 *    inversely to how much steering that took -- full speed on a straight,
 *    down to a crawl in a hairpin. This is the entire "how fast should I go"
 *    logic, and it is enough, because the only reason to steer hard is that
 *    something is in the way.
 *
 * ---------------------------------------------------------------------------
 * What it cannot do
 * ---------------------------------------------------------------------------
 *
 * Worth being clear about, since a later phase will want to beat it. It is
 * purely reactive: it cannot brake for a corner it has not yet seen, so its
 * cornering speed is set by how late it can afford to decide. It has no idea
 * of a racing line and will drive round the middle of the track, which is not
 * the fast way round. And it does not know it is racing -- it takes no account
 * of lap time at all. It is a very good baseline and a poor racer.
 */

typedef struct FollowTheGap {
    /* Scratch buffer for the clipped, smoothed, bubbled copy of the scan. The
     * incoming scan is const and belongs to the environment, and stages 1-3
     * are destructive, so the driver works on its own copy. Allocated once at
     * init rather than per call. */
    float *processed;
    int    count;

    /* The last decision, kept purely so the renderer and any debugging output
     * can show what the driver was looking at. Nothing reads these back as
     * input -- the driver is stateless between calls by design. */
    int closest_ray;
    int gap_start;      /* inclusive */
    int gap_end;        /* inclusive */
    int target_ray;
} FollowTheGap;

int  ftg_init(FollowTheGap *d, const Config *cfg);
void ftg_free(FollowTheGap *d);

/* One scan in, one action out. */
Action ftg_plan(FollowTheGap *d, const Config *cfg, const Observation *obs);

#endif /* FTG_H */
