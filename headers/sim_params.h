/*
    Runtime simulation parameters (mass, damping, dt, force limits, obstacle density, etc.).
    Idea: instead of hardcoding everything in sim_const.h, we will eventually load
    these values from a config file (bin/conf/drone_parameters.conf) and expose
    them through a small API.
*/
#ifndef SIM_PARAMS_H
#define SIM_PARAMS_H

/* 
    Default config file path used if sim_params_load(NULL) is called.
    The path is interpreted relative to the current working directory.
 */
#define SIM_PARAMS_DEFAULT_PATH "../../bin/conf/drone_parameters.conf"

/* 
    Global simulation parameters.
    Units:
    - world_width / world_height: simulation coordinates
    - mass: kg
    - damping: N·s/m (viscous damping coefficient)
    - dt: seconds (integration time step)
    - force_step: N (increment per key press)
    - max_force: N (clamp magnitude)
    - rho: meters (perception distance for repulsion)
    - eta: N·m (repulsion gain)
 */
typedef struct {
    // World geometry (simulation coordinates)
    int    world_width;
    int    world_height;

    // Drone dynamics
    double mass;
    double damping;
    double dt;

    // User command forces
    double force_step;
    double max_force;

    // Potential-field repulsion parameters
    double rho;
    double eta;

    // Environment population
    int    num_obstacles;
    int    num_targets;
} SimParams;

/* 
    Load parameters from a text file.
    
    If path is NULL, SIM_PARAMS_DEFAULT_PATH is used
    On error, reasonable defaults (from sim_const.h) remain in effect
    Returns:
    0 on success
    -1 on failure (file unreadable / parse error)
 */
int sim_params_load(const char *path);

// Return a pointer to the current parameter set
const SimParams *sim_params_get(void);
// Convenience copy 
void sim_params_get_copy(SimParams *out);

#endif