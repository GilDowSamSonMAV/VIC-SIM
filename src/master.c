#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>  // mkfifo, unlink
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "sim_ipc.h"    

// Specification of a child process to spawn from master
typedef struct {
    const char *binary;       // e.g. "./drone"
    int         use_terminal; // 1 = run in separate terminal window
    const char *term_title;   // optional title
} ChildSpec;

static int create_fifo(const char *path)
{
    // Remove any stale FIFO from previous runs
    unlink(path);
    if (mkfifo(path, 0666) == -1) {
        if (errno == EEXIST) {
            return 0; // already there, fine
        }
        perror("master: mkfifo");
        fprintf(stderr, "  path = %s\n", path);
        return -1;
    }
    return 0;
}

int main(void)
{
    // Create named FIFOs for IPC
    if (create_fifo(SIM_FIFO_DRONE_CMD)   == -1 ||
        create_fifo(SIM_FIFO_DRONE_STATE) == -1 ||
        create_fifo(SIM_FIFO_INPUT_CMD)   == -1) {
        fprintf(stderr, "master: failed to create FIFOs\n");
        return EXIT_FAILURE;
    }

    // Fork bb_server first -> it will own the world and main UI
    pid_t bb_pid = fork();
    if (bb_pid < 0) {
        perror("master: fork bb_server");
        return EXIT_FAILURE;
    }

    if (bb_pid == 0) {
        // Child: run bb_server in its own Terminator window
        //   terminator -T "BB_SERVER" -x ./bb_server
        char *term_argv[] = {
            "terminator", "-T", "BB_SERVER", "-x", "./bb_server", NULL
        };
        execvp("terminator", term_argv);

        // Fallback: run directly in this terminal if terminator fails
        char *argv2[] = { "./bb_server", NULL };
        execvp("./bb_server", argv2);

        perror("master: execvp bb_server");
        _exit(EXIT_FAILURE);
    }

    // Define the other simulator processes to launch
    ChildSpec others[] = {
        {"./drone",     0,      NULL     },
        {"./input",     1,      "INPUT"  },
        {"./obstacles", 0,      NULL     },
        {"./targets",   0,      NULL     }
    };
    const int n_others = (int)(sizeof(others)/sizeof(others[0]));
    pid_t pids[n_others];

    // Fork and exec each child process
    for (int i = 0; i < n_others; ++i) {
        pid_t pid = fork();
        if (pid < 0) {
            perror("master: fork");
            exit(EXIT_FAILURE);
        }

        if (pid == 0) {
            if (others[i].use_terminal) {
                // Run in its own Terminator window:
                //   terminator -T "<TITLE>" -x ./binary
                char *term_argv[] = {
                    "terminator", "-T", (char *)others[i].term_title,
                    "-x", (char *)others[i].binary,
                    NULL
                };
                execvp("terminator", term_argv);
            }

            // Fallback: run directly in this terminal
            char *argv2[] = { (char *)others[i].binary, NULL };
            execvp(others[i].binary, argv2);

            perror("master: execvp child");
            _exit(EXIT_FAILURE);
        }

        pids[i] = pid;
    }

    // Wait for all non-bb_server children to finish
    for (int i = 0; i < n_others; ++i) {
        int status;
        (void)waitpid(pids[i], &status, 0);
    }

    // Wait for bb_server to terminate
    (void)waitpid(bb_pid, NULL, 0);

    // Cleanup FIFOs on exit
    unlink(SIM_FIFO_DRONE_CMD);
    unlink(SIM_FIFO_DRONE_STATE);
    unlink(SIM_FIFO_INPUT_CMD);

    return EXIT_SUCCESS;
}
