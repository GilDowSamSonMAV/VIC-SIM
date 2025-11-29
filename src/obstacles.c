// Obstacles process.
// Generates a batch of static obstacles and sends them once to bb_server
// via an anonymous pipe. bb_server stores them in WorldState and uses them
// for drawing / repulsion in Phase 3.

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_params.h"
#include "sim_log.h"

static volatile sig_atomic_t running = 1;

static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

int main(int argc, char *argv[])
{
    sim_log_init("obstacles");
    signal(SIGINT, handle_sigint);

    if (argc < 2) {
        fprintf(stderr, "obstacles: usage: %s <fd_obstacles_out>\n", argv[0]);
        return EXIT_FAILURE;
    }

    int fd_obs_out = atoi(argv[SIM_ARG_OBS_OUT]);

    // Load runtime parameters for obstacles (world size, counts, etc.)
    if (sim_params_load(NULL) != 0) {
        fprintf(stderr,
                "obstacles: warning: could not load '%s', using built-in defaults\n",
                SIM_PARAMS_DEFAULT_PATH);
    }

    const SimParams *params = sim_params_get();

    fprintf(stderr,
            "obstacles: started (world=%dx%d, num_obstacles=%d)\n",
            params->world_width,
            params->world_height,
            params->num_obstacles);

    int num = params->num_obstacles;
    if (num < 0) {
        num = 0;
    }
    if (num > SIM_MAX_OBSTACLES) {
        num = SIM_MAX_OBSTACLES;
    }

    Obstacle obstacles[SIM_MAX_OBSTACLES];

    // Seed RNG with time and PID to avoid identical maps across runs
    srand((unsigned)time(NULL) ^ (unsigned)getpid());

    // Simple static obstacles: random positions in the world, fixed radius
    const double radius = 1.0;

    for (int i = 0; i < num; ++i) {
        double margin = radius;  // keep inside world bounds with a small margin

        double x_range = (double)params->world_width  - 2.0 * margin;
        double y_range = (double)params->world_height - 2.0 * margin;
        if (x_range < 0.0) x_range = 0.0;
        if (y_range < 0.0) y_range = 0.0;

        double rx = (double)rand() / (double)RAND_MAX;
        double ry = (double)rand() / (double)RAND_MAX;

        obstacles[i].x      = margin + rx * x_range;
        obstacles[i].y      = margin + ry * y_range;
        obstacles[i].radius = radius;
        obstacles[i].active = 1;
    }

    // Mark unused slots as inactive
    for (int i = num; i < SIM_MAX_OBSTACLES; ++i) {
        obstacles[i].x      = 0.0;
        obstacles[i].y      = 0.0;
        obstacles[i].radius = 0.0;
        obstacles[i].active = 0;
    }

    if (num > 0) {
        ssize_t expected = (ssize_t)(num * (int)sizeof(Obstacle));
        ssize_t w = write_full(fd_obs_out, obstacles,
                               (size_t)(num * (int)sizeof(Obstacle)));
        if (w != expected) {
            perror("obstacles: write_full(fd_obs_out)");
            fprintf(stderr, "obstacles: write_full returned %zd (expected %zd)\n",
                    w, expected);
            close(fd_obs_out);
            return EXIT_FAILURE;
        }
        fprintf(stderr, "obstacles: sent %d obstacles to bb_server\n", num);
    } else {
        fprintf(stderr, "obstacles: num_obstacles <= 0, nothing sent\n");
    }

    close(fd_obs_out);
    sim_log_info("obstacles: exiting\n");
    return EXIT_SUCCESS;
}
