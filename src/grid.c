#include "grid.h"

#include "image.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Coordinate transforms                                                      */
/* ------------------------------------------------------------------------- */

void grid_world_to_grid(const Grid *g, float wx, float wy, int *col, int *row)
{
    /* Step 1: express the point relative to the map's origin corner, undoing
     * any map rotation. Rotating by -theta is the same as rotating by theta
     * with the sine terms' signs swapped. */
    float dx = wx - g->origin_x;
    float dy = wy - g->origin_y;

    float c = cosf(g->origin_theta);
    float s = sinf(g->origin_theta);

    float mx =  c * dx + s * dy;   /* metres right along the map's bottom edge */
    float my = -s * dx + c * dy;   /* metres up along the map's left edge */

    /* Step 2: metres to cells. floorf, not a cast: casting to int truncates
     * toward zero, so -0.4 would become cell 0 instead of cell -1, and points
     * just off the left or bottom edge would be silently pulled back inside. */
    int cx = (int)floorf(mx / g->resolution);
    int cy = (int)floorf(my / g->resolution);

    /* Step 3: cy counts up from the bottom; array rows count down from the
     * top. See the header for why this flip lives here and nowhere else. */
    *col = cx;
    *row = g->height - 1 - cy;
}

void grid_grid_to_world(const Grid *g, int col, int row, float *wx, float *wy)
{
    /* Exactly the inverse of the above, and the + 0.5 puts us at the cell's
     * centre rather than its lower-left corner. */
    float mx = ((float)col + 0.5f) * g->resolution;
    float my = ((float)(g->height - 1 - row) + 0.5f) * g->resolution;

    float c = cosf(g->origin_theta);
    float s = sinf(g->origin_theta);

    *wx = g->origin_x + c * mx - s * my;
    *wy = g->origin_y + s * mx + c * my;
}

void grid_world_to_cellf(const Grid *g, float wx, float wy,
                         float *fcol, float *frow)
{
    float dx = wx - g->origin_x;
    float dy = wy - g->origin_y;

    float c = cosf(g->origin_theta);
    float s = sinf(g->origin_theta);

    float mx =  c * dx + s * dy;
    float my = -s * dx + c * dy;

    /* Columns are straightforward: metres right, divided by cell size.
     *
     * Rows are the flipped axis. `my / resolution` counts cells UP from the
     * bottom, and we want cells DOWN from the top, so subtract from the
     * height. Sanity check the two ends: a point on the top edge has
     * my = height*resolution and lands at row 0.0; a point just inside the
     * bottom edge lands just under `height`, i.e. inside row height-1. */
    *fcol = mx / g->resolution;
    *frow = (float)g->height - my / g->resolution;
}

void grid_world_dir_to_cell_dir(const Grid *g, float dx, float dy,
                                float *dcol, float *drow)
{
    float c = cosf(g->origin_theta);
    float s = sinf(g->origin_theta);

    /* Rotate into the map frame exactly as for a position, but with no
     * translation -- a direction has no origin. */
    float mdx =  c * dx + s * dy;
    float mdy = -s * dx + c * dy;

    *dcol =  mdx;
    *drow = -mdy;   /* the same flip as above, differentiated */
}

/* ------------------------------------------------------------------------- */
/* Queries                                                                    */
/* ------------------------------------------------------------------------- */

int grid_in_bounds(const Grid *g, int col, int row)
{
    return col >= 0 && col < g->width && row >= 0 && row < g->height;
}

uint8_t grid_cell(const Grid *g, int col, int row)
{
    /* Off the map is reported as occupied rather than free. A ray that leaves
     * the map should stop at the edge, and a car that leaves it should count
     * as having crashed -- both are safer than pretending the void is road. */
    if (!grid_in_bounds(g, col, row)) return CELL_OCCUPIED;
    return g->cells[(size_t)row * (size_t)g->width + (size_t)col];
}

int grid_is_blocked(const Grid *g, const Config *cfg, int col, int row)
{
    uint8_t state = grid_cell(g, col, row);
    if (state == CELL_OCCUPIED) return 1;
    if (state == CELL_UNKNOWN)  return cfg->treat_unknown_as_occupied;
    return 0;
}

int grid_is_blocked_world(const Grid *g, const Config *cfg, float wx, float wy)
{
    int col, row;
    grid_world_to_grid(g, wx, wy, &col, &row);
    return grid_is_blocked(g, cfg, col, row);
}

void grid_count_states(const Grid *g, long *free_cells, long *occupied_cells,
                       long *unknown_cells)
{
    long f = 0, o = 0, u = 0;
    size_t n = (size_t)g->width * (size_t)g->height;

    for (size_t i = 0; i < n; i++) {
        if      (g->cells[i] == CELL_FREE)     f++;
        else if (g->cells[i] == CELL_OCCUPIED) o++;
        else                                   u++;
    }

    *free_cells     = f;
    *occupied_cells = o;
    *unknown_cells  = u;
}

/* ------------------------------------------------------------------------- */
/* The map YAML                                                               */
/* ------------------------------------------------------------------------- */

/*
 * This is NOT a YAML parser, and does not pretend to be one. Real YAML has
 * anchors, nesting, multi-line strings and a specification longer than this
 * whole project. A map sidecar is six lines of `key: value`, and writing a
 * reader for exactly that shape is a dozen lines you can verify by eye,
 * whereas a general parser would be a thousand you could not.
 *
 * If a future map needs real YAML, this function should be replaced -- not
 * extended one special case at a time until it is a bad YAML parser.
 */
typedef struct {
    char  image[512];
    float resolution;
    float origin[3];
    int   negate;
    float occupied_thresh;
    float free_thresh;
    int   have_resolution;
    int   have_origin;
} MapMeta;

/* Trim ASCII whitespace and any surrounding quotes, in place. */
static char *trim(char *s)
{
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;

    size_t n = strlen(s);
    while (n > 0) {
        char c = s[n - 1];
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') n--;
        else break;
    }
    s[n] = '\0';

    if (n >= 2 && (s[0] == '"' || s[0] == '\'') && s[n - 1] == s[0]) {
        s[n - 1] = '\0';
        s++;
    }
    return s;
}

static int parse_map_yaml(const char *path, const Config *cfg, MapMeta *m,
                          char *err, size_t err_len)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        if (err) snprintf(err, err_len, "cannot open map YAML '%s'", path);
        return 0;
    }

    /* Start from the config's fallbacks so a YAML that omits a threshold still
     * yields a usable map. resolution and origin have no sensible default --
     * guessing them would silently misplace the whole track -- so they are
     * tracked and required. */
    memset(m, 0, sizeof *m);
    m->occupied_thresh = cfg->default_occupied_thresh;
    m->free_thresh     = cfg->default_free_thresh;

    char line[1024];
    while (fgets(line, sizeof line, f)) {
        char *hash = strchr(line, '#');
        if (hash) *hash = '\0';                  /* strip comments */

        char *colon = strchr(line, ':');
        if (!colon) continue;                    /* blank or junk line */

        *colon = '\0';
        char *key   = trim(line);
        char *value = trim(colon + 1);
        if (!*key || !*value) continue;

        if (strcmp(key, "image") == 0) {
            snprintf(m->image, sizeof m->image, "%s", value);
        } else if (strcmp(key, "resolution") == 0) {
            m->resolution = strtof(value, NULL);
            m->have_resolution = 1;
        } else if (strcmp(key, "origin") == 0) {
            /* Written as a flow sequence: [x, y, theta]. Walk the string
             * pulling out up to three numbers and ignore the brackets. */
            char *p = value;
            int i = 0;
            while (i < 3 && *p) {
                while (*p == '[' || *p == ',' || *p == ' ') p++;
                if (!*p || *p == ']') break;
                char *end = NULL;
                m->origin[i++] = strtof(p, &end);
                if (end == p) break;             /* not a number: stop */
                p = end;
            }
            m->have_origin = (i >= 2);
        } else if (strcmp(key, "negate") == 0) {
            m->negate = (int)strtol(value, NULL, 10);
        } else if (strcmp(key, "occupied_thresh") == 0) {
            m->occupied_thresh = strtof(value, NULL);
        } else if (strcmp(key, "free_thresh") == 0) {
            m->free_thresh = strtof(value, NULL);
        }
        /* Any other key (mode, etc.) is deliberately ignored. */
    }

    fclose(f);

    if (!m->image[0])                     { if (err) snprintf(err, err_len, "map YAML has no 'image' key"); return 0; }
    if (!m->have_resolution || m->resolution <= 0.0f)
                                          { if (err) snprintf(err, err_len, "map YAML has no usable 'resolution'"); return 0; }
    if (!m->have_origin)                  { if (err) snprintf(err, err_len, "map YAML has no usable 'origin'"); return 0; }

    return 1;
}

/*
 * Turn "maps/IMS/IMS_map.yaml" plus an image name into a path the process can
 * actually open. The YAML names its image without a directory, so it is only
 * meaningful relative to the YAML's own location -- resolving it against the
 * current working directory instead is why "it works when I cd into the map
 * folder" bugs happen.
 */
static void resolve_sibling_path(const char *yaml_path, const char *image_name,
                                 char *out, size_t out_len)
{
    /* Accept either separator: these files are written on Linux and read here
     * on Windows as often as not. */
    const char *last_slash     = strrchr(yaml_path, '/');
    const char *last_backslash = strrchr(yaml_path, '\\');
    const char *cut = last_slash > last_backslash ? last_slash : last_backslash;

    int absolute = image_name[0] == '/' || image_name[0] == '\\' ||
                   (image_name[0] && image_name[1] == ':');

    if (!cut || absolute) {
        snprintf(out, out_len, "%s", image_name);
    } else {
        int dir_len = (int)(cut - yaml_path + 1);   /* keep the separator */
        snprintf(out, out_len, "%.*s%s", dir_len, yaml_path, image_name);
    }
}

/* ------------------------------------------------------------------------- */
/* Loading                                                                    */
/* ------------------------------------------------------------------------- */

int grid_load(const char *yaml_path, const Config *cfg, Grid *g,
              char *err, size_t err_len)
{
    memset(g, 0, sizeof *g);

    MapMeta meta;
    if (!parse_map_yaml(yaml_path, cfg, &meta, err, err_len))
        return 0;

    char image_path[1024];
    resolve_sibling_path(yaml_path, meta.image, image_path, sizeof image_path);

    GrayImage img;
    char image_err[256] = { 0 };
    if (!image_load(image_path, &img, image_err, sizeof image_err)) {
        if (err) snprintf(err, err_len, "loading map image: %s", image_err);
        return 0;
    }

    size_t count = (size_t)img.width * (size_t)img.height;
    uint8_t *cells = (uint8_t *)malloc(count);
    if (!cells) {
        image_free(&img);
        if (err) snprintf(err, err_len, "out of memory for %dx%d grid", img.width, img.height);
        return 0;
    }

    /* Classify every pixel once, at load time, so that the inner loops of the
     * ray caster and the collision check compare a small integer instead of
     * redoing this float arithmetic millions of times per second. This is the
     * one place the map's greyscale meaning is interpreted. */
    for (size_t i = 0; i < count; i++) {
        float p = meta.negate ? (float)img.pixels[i] / 255.0f
                              : (255.0f - (float)img.pixels[i]) / 255.0f;

        if (p > meta.occupied_thresh)      cells[i] = CELL_OCCUPIED;
        else if (p < meta.free_thresh)     cells[i] = CELL_FREE;
        else                               cells[i] = CELL_UNKNOWN;
    }

    g->cells        = cells;
    g->width        = img.width;
    g->height       = img.height;
    g->resolution   = meta.resolution;
    g->origin_x     = meta.origin[0];
    g->origin_y     = meta.origin[1];
    g->origin_theta = meta.origin[2];

    image_free(&img);
    return 1;
}

void grid_free(Grid *g)
{
    free(g->cells);
    memset(g, 0, sizeof *g);
}
