#include "waypoints.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Rows are read into a growable buffer because we do not know the point count
 * until the file ends. Doubling on overflow means the number of reallocations
 * is logarithmic in the file size rather than linear. */
#define INITIAL_CAPACITY 256

static int grow(float **arr, int capacity)
{
    float *p = (float *)realloc(*arr, (size_t)capacity * sizeof(float));
    if (!p) return 0;
    *arr = p;
    return 1;
}

int waypoints_load(const char *csv_path, Waypoints *w, char *err, size_t err_len)
{
    memset(w, 0, sizeof *w);

    FILE *f = fopen(csv_path, "r");
    if (!f) {
        if (err) snprintf(err, err_len, "cannot open centerline '%s'", csv_path);
        return 0;
    }

    int capacity = INITIAL_CAPACITY;
    if (!grow(&w->x, capacity) || !grow(&w->y, capacity) ||
        !grow(&w->w_right, capacity) || !grow(&w->w_left, capacity)) {
        fclose(f);
        waypoints_free(w);
        if (err) snprintf(err, err_len, "out of memory reading centerline");
        return 0;
    }

    char line[1024];
    int n = 0;
    int line_number = 0;

    while (fgets(line, sizeof line, f)) {
        line_number++;

        /* '#' starts a comment. The published files use it for the header row,
         * which is why we can skip the header without special-casing it. */
        char *hash = strchr(line, '#');
        if (hash) *hash = '\0';

        /* Pull out up to four numbers, skipping whatever separators sit
         * between them. strtof tells us where it stopped, and if it did not
         * move at all we have hit something that is not a number and the row
         * is done. Accepting commas, semicolons and bare whitespace equally
         * means the same reader handles every variant of these files in the
         * wild without a format flag. */
        float values[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        int   got = 0;
        char *p   = line;

        while (got < 4) {
            while (*p == ',' || *p == ';' || *p == ' ' || *p == '\t' ||
                   *p == '\r' || *p == '\n')
                p++;
            if (!*p) break;

            char *end = NULL;
            float v = strtof(p, &end);
            if (end == p) break;

            values[got++] = v;
            p = end;
        }

        if (got < 2) continue;   /* blank line, or the header once '#' is cut */

        if (n == capacity) {
            capacity *= 2;
            if (!grow(&w->x, capacity) || !grow(&w->y, capacity) ||
                !grow(&w->w_right, capacity) || !grow(&w->w_left, capacity)) {
                fclose(f);
                waypoints_free(w);
                if (err) snprintf(err, err_len, "out of memory at waypoint %d", n);
                return 0;
            }
        }

        w->x[n]       = values[0];
        w->y[n]       = values[1];
        w->w_right[n] = values[2];
        w->w_left[n]  = values[3];
        n++;
    }

    fclose(f);

    if (n < 3) {
        waypoints_free(w);
        if (err) snprintf(err, err_len,
                          "centerline '%s' has only %d usable points (need at least 3);"
                          " last line read was %d", csv_path, n, line_number);
        return 0;
    }

    w->count = n;

    /* --- Precompute arc length ------------------------------------------ */
    if (!grow(&w->s, n)) {
        waypoints_free(w);
        if (err) snprintf(err, err_len, "out of memory for arc lengths");
        return 0;
    }

    w->s[0] = 0.0f;
    for (int i = 1; i < n; i++) {
        float dx = w->x[i] - w->x[i - 1];
        float dy = w->y[i] - w->y[i - 1];
        w->s[i] = w->s[i - 1] + sqrtf(dx * dx + dy * dy);
    }

    /* The track is a closed loop, so the total length includes the segment
     * from the last point back to the first. Forgetting that closing segment
     * makes every lap read as slightly short and the wrap-around arithmetic in
     * waypoints_forward_delta drift by that amount each lap. */
    float cdx = w->x[0] - w->x[n - 1];
    float cdy = w->y[0] - w->y[n - 1];
    w->total_length = w->s[n - 1] + sqrtf(cdx * cdx + cdy * cdy);

    return 1;
}

void waypoints_free(Waypoints *w)
{
    free(w->x);
    free(w->y);
    free(w->w_right);
    free(w->w_left);
    free(w->s);
    memset(w, 0, sizeof *w);
}

/* Wrap an index into [0, count) the long way round, so that -1 becomes the
 * last point. C's % keeps the sign of the dividend, so a bare i % count would
 * give -1 and index out of bounds. */
static int wrap_index(int i, int count)
{
    i %= count;
    if (i < 0) i += count;
    return i;
}

int waypoints_nearest(const Waypoints *w, float x, float y, int prev, int window)
{
    int best_index = 0;
    float best_d2  = -1.0f;   /* squared distance; sqrt would not change the ordering */

    int first, span;
    if (prev >= 0 && window > 0 && 2 * window + 1 < w->count) {
        first = prev - window;
        span  = 2 * window + 1;
    } else {
        first = 0;
        span  = w->count;
    }

    for (int k = 0; k < span; k++) {
        int i = wrap_index(first + k, w->count);
        float dx = w->x[i] - x;
        float dy = w->y[i] - y;
        float d2 = dx * dx + dy * dy;
        if (best_d2 < 0.0f || d2 < best_d2) {
            best_d2 = d2;
            best_index = i;
        }
    }

    return best_index;
}

/*
 * Project (px, py) onto the segment from point a to point b, and return how
 * far along that segment the projection lands, as a fraction in [0, 1], along
 * with the squared distance from the point to the segment.
 *
 * This is the standard dot-product projection: the component of (p - a) in the
 * direction of (b - a), divided by the segment's length. Clamping to [0, 1]
 * keeps the answer on the segment rather than on its infinite extension, which
 * matters when the car is past the end of one segment and onto the next.
 */
static void project_on_segment(float ax, float ay, float bx, float by,
                               float px, float py, float *t_out, float *d2_out)
{
    float ex = bx - ax;
    float ey = by - ay;
    float len2 = ex * ex + ey * ey;

    float t;
    if (len2 < 1e-12f) {
        /* Duplicated waypoints do occur in these files. Treat a zero-length
         * segment as the single point a, rather than dividing by zero. */
        t = 0.0f;
    } else {
        t = ((px - ax) * ex + (py - ay) * ey) / len2;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
    }

    float cx = ax + t * ex;
    float cy = ay + t * ey;
    float dx = px - cx;
    float dy = py - cy;

    *t_out  = t;
    *d2_out = dx * dx + dy * dy;
}

float waypoints_arclength(const Waypoints *w, int nearest, float x, float y)
{
    /* The nearest POINT is not enough to say where along the line the car is:
     * it could be just before that point or just after it. So look at both
     * segments touching it and take whichever the car is genuinely closer to. */
    int prev = wrap_index(nearest - 1, w->count);
    int next = wrap_index(nearest + 1, w->count);

    float t_before, d2_before;
    project_on_segment(w->x[prev], w->y[prev], w->x[nearest], w->y[nearest],
                       x, y, &t_before, &d2_before);

    float t_after, d2_after;
    project_on_segment(w->x[nearest], w->y[nearest], w->x[next], w->y[next],
                       x, y, &t_after, &d2_after);

    if (d2_before <= d2_after) {
        /* On the incoming segment: start from the earlier point's arc length
         * and add the fraction of that segment covered. */
        /* prev > nearest only when nearest is point 0 and prev wrapped to the
         * last point, in which case the segment between them is the closing
         * one that s[] does not cover. */
        float seg = (prev > nearest) ? w->total_length - w->s[prev]
                                     : w->s[nearest] - w->s[prev];
        float s = w->s[prev] + t_before * seg;
        return s >= w->total_length ? s - w->total_length : s;
    } else {
        float seg = (next < nearest) ? w->total_length - w->s[nearest]
                                     : w->s[next] - w->s[nearest];
        float s = w->s[nearest] + t_after * seg;
        return s >= w->total_length ? s - w->total_length : s;
    }
}

float waypoints_forward_delta(const Waypoints *w, float from, float to)
{
    float d = to - from;
    float half = 0.5f * w->total_length;

    /* Fold the difference into (-half, +half]. A jump larger than half a lap
     * is always better explained as a small move in the other direction across
     * the start/finish line. */
    while (d >  half) d -= w->total_length;
    while (d < -half) d += w->total_length;

    return d;
}

float waypoints_heading(const Waypoints *w, int i)
{
    int j = wrap_index(i + 1, w->count);
    return atan2f(w->y[j] - w->y[i], w->x[j] - w->x[i]);
}
