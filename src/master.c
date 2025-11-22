#include <errno.h>
#include <fcntl.h>      
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>   
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "sim_ipc.h"    

// Specification of a child process to spawn from master
typedef struct {
    const char *binary;       // e.g. "./drone"
    int         use_terminal; // 1 = run in separate terminal window
    const char *term_title;   // terminal title if use_terminal=1
} ChildSpec;

// Wait until the shared memory created by bb_server becomes available
static int wait_for_shm_ready(const char *name,
                              int max_attempts,
                              unsigned int delay_us)
{
    for (int i = 0; i < max_attempts; ++i) {
        int fd = shm_open(name, O_RDWR, 0666);
        if (fd != -1) {
            close(fd);
            return 0; 
        }

        if (errno != ENOENT) {
            perror("master: shm_open (wait_for_shm_ready)");
            return -1;
        }
        usleep(delay_us);
    }
    return -1;
}

int main(void)
{
    // Fork bb_server first-> it will create SHM and the semaphore
    pid_t bb_pid = fork();
    if (bb_pid < 0) {
        perror("master: fork bb_server");
        return EXIT_FAILURE;
    }

    if (bb_pid == 0) {
        // Child: try to run bb_server in its own terminal window
        char *term_argv[] = {
            "terminator", "-T", "BB_SERVER", "-x", "./bb_server", NULL
        };
        execvp("terminator", term_argv);

        // Fallback: run bb_server directly in this terminal if terminator fails
        char *argv2[] = { "./bb_server", NULL };
        execvp("./bb_server", argv2);

        perror("master: execvp bb_server");
        _exit(EXIT_FAILURE);
    }

    // Wait for bb_server to create the shared memory blackboard
    if (wait_for_shm_ready(SIM_SHM_WORLD, 50, 100000) != 0) {
        fprintf(stderr,
                "master: timeout waiting for shared memory '%s'\n",
                SIM_SHM_WORLD);
        return EXIT_FAILURE;
    }

    // Define the other simulator processes to launch
    ChildSpec others[] = {
        {"./drone", 0, NULL},
        {"./input", 1, "INPUT"},
        {"./obstacles", 0, NULL},
        {"./targets", 0, NULL}
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
                char *term_argv[] = {
                    "terminator", "-T", (char *)others[i].term_title,
                    "-x", (char *)others[i].binary,
                    NULL
                };
                execvp("terminator", term_argv);
            }

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

    // No unlinking of the shared memory or semaphore here (Done in bb_server.c)
    return EXIT_SUCCESS;
}
