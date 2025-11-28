#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sys/select.h>
#include <fcntl.h>

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_const.h"
#include "sim_log.h"
#include "sim_ui.h"

// Crucial integer type used for providing variables that can be 
// read and written by both the main prog and sign handler
// without introducing race conditions. 
static volatile sig_atomic_t running = 1; 

// Async-signal-safe SIGINT handler: just flip the running flag
static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

int main(int argc, char *argv[])
{
    sim_log_init("bb_server"); //Not used for now
    signal(SIGINT, handle_sigint);

    // Make stderr unbuffered for debugging
    setbuf(stderr, NULL);

    // FDs for anonymous pipes are now passed via argv by master:
    //   ./bb_server <fd_drone_state_in> <fd_drone_cmd_out> <fd_input_cmd_in>
    if (argc < 4) {
        fprintf(stderr,
                "bb_server: usage: %s <fd_drone_state_in> <fd_drone_cmd_out> <fd_input_cmd_in>\n",
                argv[0]);
        return EXIT_FAILURE;
    }

    int fd_drone_in  = atoi(argv[SIM_ARG_BB_DRONE_STATE_IN]);
    int fd_drone_out = atoi(argv[SIM_ARG_BB_DRONE_CMD_OUT]);
    int fd_input_in  = atoi(argv[SIM_ARG_BB_INPUT_CMD_IN]);

    fprintf(stderr,
            "bb_server: pipe FDs: drone_in=%d drone_out=%d input_in=%d\n",
            fd_drone_in, fd_drone_out, fd_input_in);

    WorldState world;

    // Initialize world state
    world.drone.x  = 0.0;
    world.drone.y  = 0.0;
    world.drone.vx = 0.0;
    world.drone.vy = 0.0;

    world.cmd.fx       = 0.0;
    world.cmd.fy       = 0.0;
    world.cmd.brake    = 0;
    world.cmd.reset    = 0;
    world.cmd.quit     = 0;
    world.cmd.last_key = 0;

    // Init UI and show menu
    ui_init();
    fprintf(stderr, "bb_server: ui_init() done, entering start menu\n");

    int start_sim = 0;
    while (!start_sim && running) { // Handling choices of menu
        int choice = ui_show_start_menu();
        fprintf(stderr, "bb_server: menu choice=%d (0=Start,1=Instr,2=Quit)\n", choice);

        if (choice == UI_MENU_QUIT) {
            running = 0;
        } else if (choice == UI_MENU_INSTRUCTIONS) {
            ui_show_instructions();
        } else if (choice == UI_MENU_START) {
            start_sim = 1;
        }
    }

    fprintf(stderr, "bb_server: after menu loop: start_sim=%d running=%d\n",
            start_sim, running);

    if (!running || !start_sim) {
        fprintf(stderr, "bb_server: exiting from menu\n");
        ui_shutdown();
        close(fd_drone_in);
        close(fd_drone_out);
        close(fd_input_in);
        return 0;
    }

    sim_log_info("bb_server: entering main loop\n");
    fprintf(stderr, "bb_server: entering main loop\n");

    // Main display + IPC loop (pipe-based, no shared memory)
    while (running) {
        fd_set readfds;
        FD_ZERO(&readfds);

        FD_SET(fd_drone_in, &readfds);
        FD_SET(fd_input_in, &readfds);

        int maxfd = (fd_drone_in > fd_input_in) ? fd_drone_in : fd_input_in;

        struct timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = 33333; // Approx 30 Hz

        int ready = select(maxfd + 1, &readfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) {
                fprintf(stderr, "bb_server: select interrupted by signal (EINTR)\n");
                continue;
            }
            endwin();
            perror("bb_server: select");
            break;
        }

        if (ready > 0) {
            // Data from drone (updated DroneState)
            if (FD_ISSET(fd_drone_in, &readfds)) {
                DroneState ds;
                ssize_t r = read_full(fd_drone_in, &ds, sizeof(ds));
                fprintf(stderr, "bb_server: read_full(drone) r=%zd (expected=%zu)\n",
                        r, sizeof(ds));

                if (r == (ssize_t)sizeof(ds)) {
                    world.drone = ds;
                } else if (r == 0) {
                    // EOF: drone closed its pipe
                    sim_log_info("bb_server: drone pipe EOF\n");
                    fprintf(stderr, "bb_server: drone pipe EOF, stopping\n");
                    running = 0;
                } else if (r < 0) {
                    endwin();
                    perror("bb_server: read_full(drone)");
                    running = 0;
                }
            }

            // Data from input (updated CommandState)
            if (FD_ISSET(fd_input_in, &readfds)) {
                CommandState cs;
                ssize_t r = read_full(fd_input_in, &cs, sizeof(cs));
                fprintf(stderr, "bb_server: read_full(input) r=%zd (expected=%zu)\n",
                        r, sizeof(cs));

                if (r == (ssize_t)sizeof(cs)) {
                    world.cmd = cs;
                    fprintf(stderr,
                            "bb_server: new cmd: fx=%.2f fy=%.2f brake=%d reset=%d quit=%d last_key=%d\n",
                            cs.fx, cs.fy, cs.brake, cs.reset, cs.quit, cs.last_key);

                    // Forward latest command to drone so it can update physics
                    ssize_t w = write_full(fd_drone_out, &cs, sizeof(cs));
                    fprintf(stderr, "bb_server: write_full(drone) w=%zd (expected=%zu)\n",
                            w, sizeof(cs));
                    if (w != (ssize_t)sizeof(cs)) {
                        endwin();
                        perror("bb_server: write_full(drone)");
                        running = 0;
                    }
                } else if (r == 0) {
                    sim_log_info("bb_server: input pipe EOF\n");
                    fprintf(stderr, "bb_server: input pipe EOF, stopping\n");
                    running = 0;
                } else if (r < 0) {
                    endwin();
                    perror("bb_server: read_full(input)");
                    running = 0;
                }
            }
        }

        ui_draw(&world);

        if (world.cmd.quit) {
            sim_log_info("bb_server: quit flag set, exiting\n");
            fprintf(stderr, "bb_server: quit flag set in world.cmd, exiting loop\n");
            break;
        }
    }

    ui_shutdown();

    close(fd_drone_in);
    close(fd_drone_out);
    close(fd_input_in);

    sim_log_info("bb_server: exited\n");
    fprintf(stderr, "bb_server: exited\n");
    return EXIT_SUCCESS;
}
