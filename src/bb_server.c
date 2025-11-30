#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sys/select.h>
#include <fcntl.h>
#include <curses.h>

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_const.h"
#include "sim_log.h"
#include "sim_ui.h"
#include "sim_params.h"

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
    sim_log_init("bb_server");
    signal(SIGINT, handle_sigint);

    // we add the music
    pid_t music = fork();

    if (music == 0) {
        // Detach completely
        int fd = open("/dev/null", O_RDWR);
        if (fd >= 0) {
            dup2(fd, STDIN_FILENO);
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            if (fd > 2) close(fd);
        }

        // Try MP3 looping with mpg123 
        execlp("mpg123", "mpg123", "q", "--loop", "-1", "music.mp3", (char *)NULL);

        // Fallback: try WAV looping 
        execlp("bash", "bash", "-c",
               "while true; do paplay music.wav || aplay music.wav; done",
               (char *)NULL);

        // if both of them fail we call perror and exit
        perror("Music!");
        _exit(1);  
    } 


    // Load parameters in this process (master's load does not carry across exec)
    if (sim_params_load(NULL) != 0) {
        sim_log_info("bb_server: could not load '%s', using built-in defaults",
                     SIM_PARAMS_DEFAULT_PATH);
    }

    // Get current runtime parameters
    const SimParams *params = sim_params_get();
    sim_log_info("bb_server: params world=%dx%d obstacles=%d targets=%d "
                 "mass=%.2f damping=%.2f dt=%.3f",
                 params->world_width,
                 params->world_height,
                 params->num_obstacles,
                 params->num_targets,
                 params->mass,
                 params->damping,
                 params->dt);

    // FDs for anonymous pipes are now passed via argv by master:
    //   ./bb_server <fd_drone_state_in> <fd_drone_cmd_out> <fd_input_cmd_in>
    //               <fd_obstacles_in> <fd_targets_in>
    if (argc < 6) {
        fprintf(stderr,
                "bb_server: usage: %s <fd_drone_state_in> <fd_drone_cmd_out> "
                "<fd_input_cmd_in> <fd_obstacles_in> <fd_targets_in>\n",
                argv[0]);
        return EXIT_FAILURE;
    }

    int fd_drone_in  = atoi(argv[SIM_ARG_BB_DRONE_STATE_IN]);
    int fd_drone_out = atoi(argv[SIM_ARG_BB_DRONE_CMD_OUT]);
    int fd_input_in  = atoi(argv[SIM_ARG_BB_INPUT_CMD_IN]);
    int fd_obs_in    = atoi(argv[SIM_ARG_BB_OBS_IN]);
    int fd_tgt_in    = atoi(argv[SIM_ARG_BB_TGT_IN]);

    sim_log_info("bb_server: pipe FDs: drone_in=%d drone_out=%d "
                 "input_in=%d obs_in=%d tgt_in=%d",
                 fd_drone_in, fd_drone_out, fd_input_in, fd_obs_in, fd_tgt_in);

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

    world.num_obstacles = 0;
    for (int i = 0; i < SIM_MAX_OBSTACLES; ++i) {
        world.obstacles[i].x      = 0.0;
        world.obstacles[i].y      = 0.0;
        world.obstacles[i].radius = 0.0;
        world.obstacles[i].active = 0;
    }

    world.num_targets = 0;
    for (int i = 0; i < SIM_MAX_TARGETS; ++i) {
        world.targets[i].x      = 0.0;
        world.targets[i].y      = 0.0;
        world.targets[i].radius = 0.0;
        world.targets[i].id     = 0;
        world.targets[i].active = 0;
        world.targets[i].time_created.tv_sec  = 0;
        world.targets[i].time_created.tv_nsec = 0;
    }

    world.score = 0.0;

    // Init UI and show menu
    ui_init();

    int start_sim = 0;
    while (!start_sim && running) { // Handling choices of menu
        int choice = ui_show_start_menu();
        sim_log_info("bb_server: menu choice=%d (0=Start,1=Instr,2=Quit)", choice);

        if (choice == UI_MENU_QUIT) {
            running = 0;
        } else if (choice == UI_MENU_INSTRUCTIONS) {
            ui_show_instructions();
        } else if (choice == UI_MENU_START) {
            start_sim = 1;
        }
    }

    sim_log_info("bb_server: after menu loop: start_sim=%d running=%d",
                 start_sim, running);

    if (!running || !start_sim) {
        ui_shutdown();
        close(fd_drone_in);
        close(fd_drone_out);
        close(fd_input_in);
        close(fd_obs_in);
        close(fd_tgt_in);
        sim_log_info("bb_server: exiting from menu");
        return 0;
    }

    sim_log_info("bb_server: entering main loop");

    // Precompute how many obstacles/targets we expect from generators
    int obs_to_read = params->num_obstacles;
    if (obs_to_read > SIM_MAX_OBSTACLES) {
        obs_to_read = SIM_MAX_OBSTACLES;
    } else if (obs_to_read < 0) {
        obs_to_read = 0;
    }

    int tgt_to_read = params->num_targets;
    if (tgt_to_read > SIM_MAX_TARGETS) {
        tgt_to_read = SIM_MAX_TARGETS;
    } else if (tgt_to_read < 0) {
        tgt_to_read = 0;
    }

    // Main display + IPC loop (pipe-based, no shared memory)
    while (running) {
        fd_set readfds;
        FD_ZERO(&readfds);

        FD_SET(fd_drone_in, &readfds);
        FD_SET(fd_input_in, &readfds);
        FD_SET(fd_obs_in,   &readfds);
        FD_SET(fd_tgt_in,   &readfds);

        int maxfd = fd_drone_in;
        if (fd_input_in > maxfd)  maxfd = fd_input_in;
        if (fd_obs_in   > maxfd)  maxfd = fd_obs_in;
        if (fd_tgt_in   > maxfd)  maxfd = fd_tgt_in;

        struct timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = 33333; // Approx 30 Hz

        int ready = select(maxfd + 1, &readfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) {
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

                if (r == (ssize_t)sizeof(ds)) {
                    world.drone = ds;
                } else if (r == 0) {
                    // EOF: drone closed its pipe
                    sim_log_info("bb_server: drone pipe EOF");
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

                if (r == (ssize_t)sizeof(cs)) {
                    world.cmd = cs;

                    // Forward latest command to drone so it can update physics
                    ssize_t w = write_full(fd_drone_out, &cs, sizeof(cs));
                    if (w != (ssize_t)sizeof(cs)) {
                        endwin();
                        perror("bb_server: write_full(drone)");
                        running = 0;
                    }
                } else if (r == 0) {
                    sim_log_info("bb_server: input pipe EOF");
                    running = 0;
                } else if (r < 0) {
                    endwin();
                    perror("bb_server: read_full(input)");
                    running = 0;
                }
            }

            // Data from obstacles (Obstacle array)
            if (FD_ISSET(fd_obs_in, &readfds) && obs_to_read > 0) {
                ssize_t expected = (ssize_t)(obs_to_read * (int)sizeof(Obstacle));
                ssize_t r = read_full(fd_obs_in, world.obstacles,
                                      (size_t)(obs_to_read * (int)sizeof(Obstacle)));

                if (r == expected) {
                    world.num_obstacles = obs_to_read;
                } else if (r == 0) {
                    sim_log_info("bb_server: obstacles pipe EOF");
                    // Keep last known obstacles, just don't expect more updates
                    obs_to_read = 0;
                } else if (r < 0) {
                    endwin();
                    perror("bb_server: read_full(obstacles)");
                    running = 0;
                }
            }

            // Data from targets (Target array)
            if (FD_ISSET(fd_tgt_in, &readfds) && tgt_to_read > 0) {
                ssize_t expected = (ssize_t)(tgt_to_read * (int)sizeof(Target));
                ssize_t r = read_full(fd_tgt_in, world.targets,
                                      (size_t)(tgt_to_read * (int)sizeof(Target)));

                if (r == expected) {
                    world.num_targets = tgt_to_read;
                } else if (r == 0) {
                    sim_log_info("bb_server: targets pipe EOF");
                    tgt_to_read = 0;
                } else if (r < 0) {
                    endwin();
                    perror("bb_server: read_full(targets)");
                    running = 0;
                }
            }
        }

        ui_draw(&world);

        if (world.cmd.quit) {
            sim_log_info("bb_server: quit flag set, exiting");
            break;
        }
    }

    ui_shutdown();

    close(fd_drone_in);
    close(fd_drone_out);
    close(fd_input_in);
    close(fd_obs_in);
    close(fd_tgt_in);

    sim_log_info("bb_server: exited");
    sim_log_close();
    return EXIT_SUCCESS;
}
