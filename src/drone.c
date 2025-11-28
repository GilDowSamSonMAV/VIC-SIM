#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/select.h>
#include <errno.h>
#include <fcntl.h>

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_const.h"
#include "sim_log.h"

// Flag set by the SIGINT handler to request a clean shutdown
static volatile sig_atomic_t running = 1;

// Async-signal-safe SIGINT handler: just flip the running flag
static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

// Keep drone inside [0, SIM_WORLD_WIDTH] × [0, SIM_WORLD_HEIGHT]
static void apply_world_bounds(DroneState *d)
{
    if (d->x < 0.0) {
        d->x = 0.0;
        if (d->vx < 0.0) d->vx = 0.0;
    } else if (d->x > SIM_WORLD_WIDTH) {
        d->x = SIM_WORLD_WIDTH;
        if (d->vx > 0.0) d->vx = 0.0;
    }

    if (d->y < 0.0) {
        d->y = 0.0;
        if (d->vy < 0.0) d->vy = 0.0;
    } else if (d->y > SIM_WORLD_HEIGHT) {
        d->y = SIM_WORLD_HEIGHT;
        if (d->vy > 0.0) d->vy = 0.0;
    }
}

int main(int argc, char *argv[])
{
    sim_log_init("drone");
    signal(SIGINT, handle_sigint);

    // FDs for anonymous pipes are passed via argv by master:
    //   ./drone <fd_cmd_in> <fd_state_out>
    if (argc < 3) {
        fprintf(stderr, "drone: usage: %s <fd_cmd_in> <fd_state_out>\n", argv[0]);
        return EXIT_FAILURE;
    }

    int fd_cmd_in    = atoi(argv[SIM_ARG_DRONE_CMD_IN]);
    int fd_state_out = atoi(argv[SIM_ARG_DRONE_STATE_OUT]);

    const double dt      = SIM_DEFAULT_DT;
    const double mass    = SIM_DEFAULT_MASS;
    const double damping = SIM_DEFAULT_DAMPING;

    unsigned int sleep_us = (unsigned int)(dt * 1e6);

    sim_log_info("drone: started (dt=%.3f, M=%.3f, K=%.3f)\n",
                 dt, mass, damping);

    DroneState   d;
    CommandState c;

    d.x  = 0.0;
    d.y  = 0.0;
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
                    d.x  = 0.0;
                    d.y  = 0.0;
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

        d.x  += d.vx * dt;
        d.y  += d.vy * dt;

        apply_world_bounds(&d);

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
