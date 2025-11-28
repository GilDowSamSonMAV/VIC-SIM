#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "sim_ipc.h"

int main(void)
{
    int pipe_drone_cmd[2];    // bb_server -> drone (CommandState)
    int pipe_drone_state[2];  // drone -> bb_server (DroneState)
    int pipe_input_cmd[2];    // input -> bb_server (CommandState)

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

    // bb_server
    pid_t bb_pid = fork();
    if (bb_pid < 0) {
        perror("master: fork bb_server");
        return EXIT_FAILURE;
    }

    if (bb_pid == 0) {
        // Child: bb_server in its own Konsole window
        // Keep:
        // pipe_drone_state[0]
        // pipe_drone_cmd[1]
        // pipe_input_cmd[0]

        // Close unused ends in this child
        close(pipe_drone_state[1]);
        close(pipe_drone_cmd[0]);  
        close(pipe_input_cmd[1]);  


        char fd_drone_state_in[16];
        char fd_drone_cmd_out[16];
        char fd_input_cmd_in[16];

        snprintf(fd_drone_state_in, sizeof(fd_drone_state_in), "%d", pipe_drone_state[0]);
        snprintf(fd_drone_cmd_out,   sizeof(fd_drone_cmd_out),   "%d", pipe_drone_cmd[1]);
        snprintf(fd_input_cmd_in,    sizeof(fd_input_cmd_in),    "%d", pipe_input_cmd[0]);

        // Konsole -T "BB_SERVER" -e ./bb_server <fds...>
        execlp("konsole", "konsole",
               "-T", "BB_SERVER",
               "-e", "./bb_server",
               fd_drone_state_in,
               fd_drone_cmd_out,
               fd_input_cmd_in,
               (char *)NULL);

        // Fallback: run directly if Konsole is unavailable
        execl("./bb_server", "./bb_server",
              fd_drone_state_in,
              fd_drone_cmd_out,
              fd_input_cmd_in,
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
        // Keep
        // cmd_in: pipe_drone_cmd[0] (read)
        // state_out: pipe_drone_state[1] (write)

        // Close unused ends
        close(pipe_drone_cmd[1]);
        close(pipe_drone_state[0]);
        close(pipe_input_cmd[0]);
        close(pipe_input_cmd[1]);

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
        // No pipes needed
        close(pipe_drone_cmd[0]);  close(pipe_drone_cmd[1]);
        close(pipe_drone_state[0]);close(pipe_drone_state[1]);
        close(pipe_input_cmd[0]);  close(pipe_input_cmd[1]);

        execl("./obstacles", "./obstacles", (char *)NULL);
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
        // No pipes needed
        close(pipe_drone_cmd[0]);  close(pipe_drone_cmd[1]);
        close(pipe_drone_state[0]);close(pipe_drone_state[1]);
        close(pipe_input_cmd[0]);  close(pipe_input_cmd[1]);

        execl("./targets", "./targets", (char *)NULL);
        perror("master: exec targets");
        _exit(EXIT_FAILURE);
    }

    // Close unused pipes
    close(pipe_drone_cmd[0]);  close(pipe_drone_cmd[1]);
    close(pipe_drone_state[0]);close(pipe_drone_state[1]);
    close(pipe_input_cmd[0]);  close(pipe_input_cmd[1]);

    // Wait for children (at least main three)
    int status;
    (void)waitpid(drone_pid, &status, 0);
    (void)waitpid(input_pid, &status, 0);
    (void)waitpid(bb_pid, &status, 0);
    (void)waitpid(obstacles_pid, &status, 0);
    (void)waitpid(targets_pid, &status, 0);

    return EXIT_SUCCESS;
}
