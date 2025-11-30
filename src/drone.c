#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/select.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>        // fabs for motion deadzone

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_const.h"
#include "sim_log.h"
#include "sim_params.h"   // runtime parameters (mass, damping, dt, world size)

// Flag set by the SIGINT handler to request a clean shutdown
static volatile sig_atomic_t running = 1;

// Async-signal-safe SIGINT handler: just flip the running flag
static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

// Keep drone inside [0, world_width] × [0, world_height].
// Also zero velocity components that point into the wall so we don't keep
// bouncing or sliding along the boundary forever.
static void apply_world_bounds(DroneState *d, double world_width, double world_height)
{
    if (d->x < 0.0) {
        d->x = 0.0;
        if (d->vx < 0.0) d->vx = 0.0;      // kill velocity into left wall
    } else if (d->x > world_width) {
        d->x = world_width;
        if (d->vx > 0.0) d->vx = 0.0;      // kill velocity into right wall
    }

    if (d->y < 0.0) {
        d->y = 0.0;
        if (d->vy < 0.0) d->vy = 0.0;      // kill velocity into bottom wall
    } else if (d->y > world_height) {
        d->y = world_height;
        if (d->vy > 0.0) d->vy = 0.0;      // kill velocity into top wall
    }
}

// Zero out very small velocities so we don't get visual jitter from tiny
// residual motion near equilibrium (especially near walls).
static void apply_motion_deadzone(DroneState *d)
{
    const double V_EPS = 0.01;  // tweakable: "small enough" velocity

    if (fabs(d->vx) < V_EPS) d->vx = 0.0;
    if (fabs(d->vy) < V_EPS) d->vy = 0.0;
}

int main(int argc, char *argv[])
{
    sim_log_init("drone");
    signal(SIGINT, handle_sigint);

    // Load runtime parameters in this process
    if (sim_params_load(NULL) != 0) {
        fprintf(stderr,
                "drone: warning: could not load '%s', using built-in defaults\n",
                SIM_PARAMS_DEFAULT_PATH);
    }
    const SimParams *params = sim_params_get();

    // FDs for anonymous pipes are passed via argv by master:
    //   ./drone <fd_cmd_in> <fd_state_out>
    if (argc < 3) {
        fprintf(stderr, "drone: usage: %s <fd_cmd_in> <fd_state_out>\n", argv[0]);
        return EXIT_FAILURE;
    }

    int fd_cmd_in    = atoi(argv[SIM_ARG_DRONE_CMD_IN]);
    int fd_state_out = atoi(argv[SIM_ARG_DRONE_STATE_OUT]);

    // Use dt, mass, damping and world size from parameter file (or defaults)
    const double dt           = params->dt;
    const double mass         = params->mass;
    const double damping      = params->damping;
    const double world_width  = (double)params->world_width;
    const double world_height = (double)params->world_height;

    unsigned int sleep_us = (unsigned int)(dt * 1e6);

    sim_log_info("drone: started (dt=%.3f, M=%.3f, K=%.3f)\n",
                 dt, mass, damping);

    DroneState   d;
    CommandState c;

    // Start the drone at the center of the world
    d.x  = world_width  / 2.0;
    d.y  = world_height / 2.0;
    d.vx = 0.0;
    d.vy = 0.0;

    c.fx       = 0.0;
    c.fy       = 0.0;
    c.brake    = 0;
    c.reset    = 0;
    c.quit     = 0;
    c.last_key = 0;

    while (running) {
        // Wait up to dt for a new CommandState from bb_server
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd_cmd_in, &readfds);

        struct timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = sleep_us;

        int ready = select(fd_cmd_in + 1, &readfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            perror("drone: select");
            break;
        }

        if (ready > 0 && FD_ISSET(fd_cmd_in, &readfds)) {
            CommandState new_c;
            ssize_t r = read_full(fd_cmd_in, &new_c, sizeof(new_c));
            if (r == (ssize_t)sizeof(new_c)) {
                int reset_edge = (new_c.reset == 1 && c.reset == 0);
                c = new_c;

                if (c.quit) {
                    sim_log_info("drone: quit flag set, exiting\n");
                    break;
                }

                if (reset_edge) {
                    // Reset back to center of the world
                    d.x  = world_width  / 2.0;
                    d.y  = world_height / 2.0;
                    d.vx = 0.0;
                    d.vy = 0.0;
                }
            } else if (r == 0) {
                sim_log_info("drone: cmd pipe EOF, exiting\n");
                break;
            } else if (r < 0) {
                perror("drone: read_full(fd_cmd_in)");
                break;
            }
        }

        double fx = c.fx;
        double fy = c.fy;

        double ax = (fx - damping * d.vx) / mass;
        double ay = (fy - damping * d.vy) / mass;

        d.vx += ax * dt;
        d.vy += ay * dt;

        // Kill tiny velocities to avoid jitter when we're almost at rest
        apply_motion_deadzone(&d);

        d.x  += d.vx * dt;
        d.y  += d.vy * dt;

        apply_world_bounds(&d, world_width, world_height);

        if (write_full(fd_state_out, &d, sizeof(d)) != (ssize_t)sizeof(d)) {
            perror("drone: write_full(fd_state_out)");
            break;
        }
    }

    sim_log_info("drone: exiting\n");
    close(fd_cmd_in);
    close(fd_state_out);
    return EXIT_SUCCESS;
}
