#ifndef MATHF_H
#define MATHF_H

#include <math.h>

/*
 * Three tiny float helpers used all over the project.
 *
 * They live in a header as `static inline` rather than in a .c file because
 * each is a couple of operations, and the alternative -- a function call
 * through another translation unit for a min/max pair -- adds indirection to
 * read for no benefit. `static` keeps each translation unit's copy private, so
 * there is no duplicate-symbol problem at link time.
 *
 * Everything here is float, never double. That is a project-wide rule: mixing
 * the two silently promotes expressions to double, and then a "float"
 * simulator is quietly doing double-precision work in half its arithmetic,
 * which is both slower and makes results depend on where the promotions
 * happened to land.
 */

/*
 * M_PI is NOT part of standard C -- it is a POSIX extension, and building with
 * -std=c11 (rather than -std=gnu11) correctly hides it. Rather than loosening
 * the standard flag for one constant, define it here, once.
 */
#define PI_F     3.14159265358979f
#define TWO_PI_F 6.28318530717959f

static inline float rad_to_deg(float radians) { return radians * (180.0f / PI_F); }
static inline float deg_to_rad(float degrees) { return degrees * (PI_F / 180.0f); }

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/*
 * Fold an angle into (-pi, pi].
 *
 * Headings accumulate: a car going in circles will have its heading grow past
 * 2*pi, then 4*pi, and so on. That is harmless for sin and cos, but the moment
 * you SUBTRACT two headings to ask "how far apart are these directions?" it
 * matters enormously -- 0.1 rad and 6.2 rad are 0.18 rad apart, not 6.1. Any
 * difference of angles should be passed through this before it is used.
 */
static inline float wrap_angle(float a)
{
    while (a >   PI_F) a -= TWO_PI_F;
    while (a <= -PI_F) a += TWO_PI_F;
    return a;
}

/* Straight-line distance between two points. Named to read like the geometry
 * it performs at the call site. */
static inline float distf(float ax, float ay, float bx, float by)
{
    float dx = bx - ax;
    float dy = by - ay;
    return sqrtf(dx * dx + dy * dy);
}

#endif /* MATHF_H */
