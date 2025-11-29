#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "sim_ipc.h"
#include "sim_params.h"

int main(void)
{
    // Load runtime parameters from config file (or fall back to defaults)
    if (sim_params_load(NULL) != 0) {
        fprintf(stderr,
                "master: warning: could not load '%s', using built-in defaults\n",
                SIM_PARAMS_DEFAULT_PATH);
    }

    int pipe_drone_cmd[2];    // bb_server -> drone (CommandState)
    int pipe_drone_state[2];  // drone -> bb_server (DroneState)
    int pipe_input_cmd[2];    // input -> bb_server (CommandState)
    int pipe_obstacles[2];    // obstacles -> bb_server (Obstacle[])
    int pipe_targets[2];      // targets   -> bb_server (Target[])

    if (pipe(pipe_drone_cmd) == -1) {
        perror("master: pipe_drone_cmd");
        return EXIT_FAILURE;
    }
    if (pipe(pipe_drone_state) == -1) {
        perror("master: pipe_drone_state");
        return EXIT_FAILURE;
    }
    if (pipe(pipe_input_cmd) == -1) {
        perror("master: pipe_input_cmd");
        return EXIT_FAILURE;
    }
    if (pipe(pipe_obstacles) == -1) {
        perror("master: pipe_obstacles");
        return EXIT_FAILURE;
    }
    if (pipe(pipe_targets) == -1) {
        perror("master: pipe_targets");
        return EXIT_FAILURE;
    }

    // bb_server
    pid_t bb_pid = fork();
    if (bb_pid < 0) {
        perror("master: fork bb_server");
        return EXIT_FAILURE;
    }

    if (bb_pid == 0) {
        // Child: bb_server in its own Konsole window
        // Keep:
        //   pipe_drone_state[0]
        //   pipe_drone_cmd[1]
        //   pipe_input_cmd[0]
        //   pipe_obstacles[0]
        //   pipe_targets[0]

        // Close unused ends in this child
        close(pipe_drone_state[1]);
        close(pipe_drone_cmd[0]);
        close(pipe_input_cmd[1]);
        close(pipe_obstacles[1]);
        close(pipe_targets[1]);

        char fd_drone_state_in[16];
        char fd_drone_cmd_out[16];
        char fd_input_cmd_in[16];
        char fd_obs_in[16];
        char fd_tgt_in[16];

        snprintf(fd_drone_state_in, sizeof(fd_drone_state_in), "%d", pipe_drone_state[0]);
        snprintf(fd_drone_cmd_out,   sizeof(fd_drone_cmd_out),   "%d", pipe_drone_cmd[1]);
        snprintf(fd_input_cmd_in,    sizeof(fd_input_cmd_in),    "%d", pipe_input_cmd[0]);
        snprintf(fd_obs_in,          sizeof(fd_obs_in),          "%d", pipe_obstacles[0]);
        snprintf(fd_tgt_in,          sizeof(fd_tgt_in),          "%d", pipe_targets[0]);

        // Konsole -T "BB_SERVER" -e ./bb_server <fds...>
        execlp("konsole", "konsole",
               "-T", "BB_SERVER",
               "-e", "./bb_server",
               fd_drone_state_in,
               fd_drone_cmd_out,
               fd_input_cmd_in,
               fd_obs_in,
               fd_tgt_in,
               (char *)NULL);

        // Fallback: run directly if Konsole is unavailable
        execl("./bb_server", "./bb_server",
              fd_drone_state_in,
              fd_drone_cmd_out,
              fd_input_cmd_in,
              fd_obs_in,
              fd_tgt_in,
              (char *)NULL);

        perror("master: exec bb_server");
        _exit(EXIT_FAILURE);
    }

    // Input
    pid_t input_pid = fork();
    if (input_pid < 0) {
        perror("master: fork input");
        return EXIT_FAILURE;
    }

    if (input_pid == 0) {
        // Keep: pipe_input_cmd[1]
        // Close unused ends
        close(pipe_input_cmd[0]);
        close(pipe_drone_cmd[0]);
        close(pipe_drone_cmd[1]);
        close(pipe_drone_state[0]);
        close(pipe_drone_state[1]);
        close(pipe_obstacles[0]);
        close(pipe_obstacles[1]);
        close(pipe_targets[0]);
        close(pipe_targets[1]);

        char fd_cmd_out[16];
        snprintf(fd_cmd_out, sizeof(fd_cmd_out), "%d", pipe_input_cmd[1]);

        // Konsole -T "INPUT" -e ./input <fd_cmd_out>
        execlp("konsole", "konsole",
               "-T", "INPUT",
               "-e", "./input",
               fd_cmd_out,
               (char *)NULL);

        // Fallback: run directly
        execl("./input", "./input", fd_cmd_out, (char *)NULL);

        perror("master: exec input");
        _exit(EXIT_FAILURE);
    }

    // Drone
    pid_t drone_pid = fork();
    if (drone_pid < 0) {
        perror("master: fork drone");
        return EXIT_FAILURE;
    }

    if (drone_pid == 0) {
        // Keep:
        //   cmd_in:    pipe_drone_cmd[0] (read)
        //   state_out: pipe_drone_state[1] (write)

        // Close unused ends
        close(pipe_drone_cmd[1]);
        close(pipe_drone_state[0]);
        close(pipe_input_cmd[0]);
        close(pipe_input_cmd[1]);
        close(pipe_obstacles[0]);
        close(pipe_obstacles[1]);
        close(pipe_targets[0]);
        close(pipe_targets[1]);

        char fd_cmd_in[16];
        char fd_state_out[16];

        snprintf(fd_cmd_in,    sizeof(fd_cmd_in),    "%d", pipe_drone_cmd[0]);
        snprintf(fd_state_out, sizeof(fd_state_out), "%d", pipe_drone_state[1]);

        execl("./drone", "./drone", fd_cmd_in, fd_state_out, (char *)NULL);

        perror("master: exec drone");
        _exit(EXIT_FAILURE);
    }

    // Obstacles
    pid_t obstacles_pid = fork();
    if (obstacles_pid < 0) {
        perror("master: fork obstacles");
        return EXIT_FAILURE;
    }

    if (obstacles_pid == 0) {
        // Keep: pipe_obstacles[1] (write to bb_server)
        // Close all other ends
        close(pipe_obstacles[0]);
        close(pipe_drone_cmd[0]);   close(pipe_drone_cmd[1]);
        close(pipe_drone_state[0]); close(pipe_drone_state[1]);
        close(pipe_input_cmd[0]);   close(pipe_input_cmd[1]);
        close(pipe_targets[0]);     close(pipe_targets[1]);

        char fd_obs_out[16];
        snprintf(fd_obs_out, sizeof(fd_obs_out), "%d", pipe_obstacles[1]);

        execl("./obstacles", "./obstacles", fd_obs_out, (char *)NULL);
        perror("master: exec obstacles");
        _exit(EXIT_FAILURE);
    }

    // Targets
    pid_t targets_pid = fork();
    if (targets_pid < 0) {
        perror("master: fork targets");
        return EXIT_FAILURE;
    }

    if (targets_pid == 0) {
        // Keep: pipe_targets[1] (write to bb_server)
        // Close all other ends
        close(pipe_targets[0]);
        close(pipe_drone_cmd[0]);   close(pipe_drone_cmd[1]);
        close(pipe_drone_state[0]); close(pipe_drone_state[1]);
        close(pipe_input_cmd[0]);   close(pipe_input_cmd[1]);
        close(pipe_obstacles[0]);   close(pipe_obstacles[1]);

        char fd_tgt_out[16];
        snprintf(fd_tgt_out, sizeof(fd_tgt_out), "%d", pipe_targets[1]);

        execl("./targets", "./targets", fd_tgt_out, (char *)NULL);
        perror("master: exec targets");
        _exit(EXIT_FAILURE);
    }

    // Close unused pipes in master
    close(pipe_drone_cmd[0]);   close(pipe_drone_cmd[1]);
    close(pipe_drone_state[0]); close(pipe_drone_state[1]);
    close(pipe_input_cmd[0]);   close(pipe_input_cmd[1]);
    close(pipe_obstacles[0]);   close(pipe_obstacles[1]);
    close(pipe_targets[0]);     close(pipe_targets[1]);

    // Wait for children
    int status;
    (void)waitpid(drone_pid, &status, 0);
    (void)waitpid(input_pid, &status, 0);
    (void)waitpid(bb_pid, &status, 0);
    (void)waitpid(obstacles_pid, &status, 0);
    (void)waitpid(targets_pid, &status, 0);

    return EXIT_SUCCESS;
}
