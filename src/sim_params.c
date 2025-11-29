// Runtime simulation parameters implementation.
// Loads values from a text file and falls back to sim_const.h defaults if anything goes wrong.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sim_const.h"
#include "sim_params.h"

// Internal global parameter set
static SimParams g_params;
static int g_params_initialized = 0;

// Fill g_params with factory defaults from sim_const.h
static void sim_params_init_defaults(void)
{
    // World size from sim_const.h (cast double -> int for SimParams)
    g_params.world_width  = (int)SIM_WORLD_WIDTH;
    g_params.world_height = (int)SIM_WORLD_HEIGHT;

    // Drone dynamics
    g_params.mass    = SIM_DEFAULT_MASS;
    g_params.damping = SIM_DEFAULT_DAMPING;
    g_params.dt      = SIM_DEFAULT_DT;

    // Input force scaling
    g_params.force_step = SIM_DEFAULT_FORCE_STEP;
    g_params.max_force  = SIM_DEFAULT_MAX_FORCE;

    // Potential-field repulsion
    g_params.rho = SIM_DEFAULT_RHO;
    g_params.eta = SIM_DEFAULT_ETA;

    // Environment population
    g_params.num_obstacles = SIM_DEFAULT_NUM_OBSTACLES;
    g_params.num_targets   = SIM_DEFAULT_NUM_TARGETS;

    g_params_initialized = 1;
}

int sim_params_load(const char *path)
{
    char line[256];
    char key[64];
    char value[64];
    const char *use_path;
    FILE *fp;

    if (!g_params_initialized) {
        sim_params_init_defaults();
    }

    // Use default path if caller passes NULL
    use_path = (path != NULL) ? path : SIM_PARAMS_DEFAULT_PATH;

    fp = fopen(use_path, "r");
    if (!fp) {
        // Could not open file, keep defaults and tell caller it failed
        return -1;
    }

    while (fgets(line, sizeof(line), fp)) {
        char *p = line;

        // Skip leading spaces / tabs
        while (*p == ' ' || *p == '\t') {
            ++p;
        }

        // Skip empty lines and comments starting with # or //
        if (*p == '\0' || *p == '\n' || *p == '#') {
            continue;
        }
        if (p[0] == '/' && p[1] == '/') {
            continue;
        }

        if (sscanf(p, "%63s %63s", key, value) != 2) {
            continue;
        }

        // Accept both legacy short keys and more explicit names
        if (strcmp(key, "world_width") == 0 || strcmp(key, "width") == 0) {
            g_params.world_width = (int)strtol(value, NULL, 10);
        } else if (strcmp(key, "world_height") == 0 || strcmp(key, "height") == 0) {
            g_params.world_height = (int)strtol(value, NULL, 10);
        } else if (strcmp(key, "num_obstacles") == 0 || strcmp(key, "obstacles") == 0) {
            g_params.num_obstacles = (int)strtol(value, NULL, 10);
        } else if (strcmp(key, "num_targets") == 0 || strcmp(key, "targets") == 0) {
            g_params.num_targets = (int)strtol(value, NULL, 10);
        } else if (strcmp(key, "mass") == 0) {
            g_params.mass = strtod(value, NULL);
        } else if (strcmp(key, "damping") == 0 || strcmp(key, "coefficient") == 0) {
            g_params.damping = strtod(value, NULL);
        } else if (strcmp(key, "dt") == 0 || strcmp(key, "refresh") == 0) {
            g_params.dt = strtod(value, NULL);
        } else if (strcmp(key, "force_step") == 0) {
            g_params.force_step = strtod(value, NULL);
        } else if (strcmp(key, "max_force") == 0) {
            g_params.max_force = strtod(value, NULL);
        } else if (strcmp(key, "rho") == 0 || strcmp(key, "radius") == 0) {
            g_params.rho = strtod(value, NULL);
        } else if (strcmp(key, "eta") == 0) {
            g_params.eta = strtod(value, NULL);
        }
        // Unknown keys are ignored on purpose
    }

    fclose(fp);
    return 0;
}

const SimParams *sim_params_get(void)
{
    if (!g_params_initialized) {
        sim_params_init_defaults();
    }
    return &g_params;
}

void sim_params_get_copy(SimParams *out)
{
    if (!g_params_initialized) {
        sim_params_init_defaults();
    }
    if (out) {
        *out = g_params;
    }
}
