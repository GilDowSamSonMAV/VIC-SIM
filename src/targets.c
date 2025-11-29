// Targets process.
// Generates a batch of static targets and sends them once to bb_server
// via an anonymous pipe. bb_server stores them in WorldState and uses them
// for drawing / scoring / repulsion in Phase 3.

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_params.h"
#include "sim_log.h"

// Flag set by the SIGINT handler to request a clean shutdown (unused for now)
static volatile sig_atomic_t running = 1;

static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

int main(int argc, char *argv[])
{
    sim_log_init("targets");
    signal(SIGINT, handle_sigint);

    if (argc < 2) {
        fprintf(stderr, "targets: usage: %s <fd_targets_out>\n", argv[0]);
        return EXIT_FAILURE;
    }

    int fd_tgt_out = atoi(argv[SIM_ARG_TGT_OUT]);

    // Load runtime parameters for targets (world size, counts, etc.)
    if (sim_params_load(NULL) != 0) {
        fprintf(stderr,
                "targets: warning: could not load '%s', using built-in defaults\n",
                SIM_PARAMS_DEFAULT_PATH);
    }

    const SimParams *params = sim_params_get();

    fprintf(stderr,
            "targets: started (world=%dx%d, num_targets=%d)\n",
            params->world_width,
            params->world_height,
            params->num_targets);

    int num = params->num_targets;
    if (num < 0) {
        num = 0;
    }
    if (num > SIM_MAX_TARGETS) {
        num = SIM_MAX_TARGETS;
    }

    Target targets[SIM_MAX_TARGETS];

    // Seed RNG with time and PID to avoid identical maps across runs
    srand((unsigned)time(NULL) ^ (unsigned)getpid());

    // Simple static targets: random positions in the world, fixed radius
    const double radius = 1.0;

    for (int i = 0; i < num; ++i) {
        double margin = radius;  // keep inside world bounds with a small margin

        double x_range = (double)params->world_width  - 2.0 * margin;
        double y_range = (double)params->world_height - 2.0 * margin;
        if (x_range < 0.0) x_range = 0.0;
        if (y_range < 0.0) y_range = 0.0;

        double rx = (double)rand() / (double)RAND_MAX;
        double ry = (double)rand() / (double)RAND_MAX;

        targets[i].x      = margin + rx * x_range;
        targets[i].y      = margin + ry * y_range;
        targets[i].radius = radius;
        targets[i].id     = i + 1;
        targets[i].active = 1;

        clock_gettime(CLOCK_REALTIME, &targets[i].time_created);
    }

    // Mark unused slots as inactive
    for (int i = num; i < SIM_MAX_TARGETS; ++i) {
        targets[i].x      = 0.0;
        targets[i].y      = 0.0;
        targets[i].radius = 0.0;
        targets[i].id     = 0;
        targets[i].active = 0;
        targets[i].time_created.tv_sec  = 0;
        targets[i].time_created.tv_nsec = 0;
    }

    if (num > 0) {
        ssize_t expected = (ssize_t)(num * (int)sizeof(Target));
        ssize_t w = write_full(fd_tgt_out, targets,
                               (size_t)(num * (int)sizeof(Target)));
        if (w != expected) {
            perror("targets: write_full(fd_tgt_out)");
            fprintf(stderr, "targets: write_full returned %zd (expected %zd)\n",
                    w, expected);
            close(fd_tgt_out);
            return EXIT_FAILURE;
        }
        fprintf(stderr, "targets: sent %d targets to bb_server\n", num);
    } else {
        fprintf(stderr, "targets: num_targets <= 0, nothing sent\n");
    }

    close(fd_tgt_out);
    sim_log_info("targets: exiting\n");
    return EXIT_SUCCESS;
}
