/*
    IPC identifiers for the shared WorldState stuct defined in sim_types.h.

    SHM/SEM identifiers are kept for compatibility with older phases but are
    not used in the current pipe-based implementation.
*/

#ifndef SIM_IPC_H
#define SIM_IPC_H

#define SIM_SHM_WORLD   "/sim_world_shm"
#define SIM_SEM_WORLD   "/sim_world_sem"

#include <sys/types.h>
#include <stddef.h>

/*
    Anonymous pipe FD positions in argv for each process.

    master will create all pipes with pipe(), then fork/exec the children
    and pass the relevant FD numbers as command-line arguments:

      bb_server argv layout:
        ./bb_server <fd_drone_state_in>
                    <fd_drone_cmd_out>
                    <fd_input_cmd_in>
                    <fd_obstacles_in>
                    <fd_targets_in>

      drone argv layout:
        ./drone <fd_cmd_in> <fd_state_out>

      input argv layout:
        ./input <fd_cmd_out>

      obstacles argv layout:
        ./obstacles <fd_obstacles_out>

      targets argv layout:
        ./targets <fd_targets_out>
*/

#define SIM_ARG_BB_DRONE_STATE_IN   1
#define SIM_ARG_BB_DRONE_CMD_OUT    2
#define SIM_ARG_BB_INPUT_CMD_IN     3

#define SIM_ARG_BB_OBS_IN           4
#define SIM_ARG_BB_TGT_IN           5

#define SIM_ARG_DRONE_CMD_IN        1
#define SIM_ARG_DRONE_STATE_OUT     2

#define SIM_ARG_INPUT_CMD_OUT       1

#define SIM_ARG_OBS_OUT             1

#define SIM_ARG_TGT_OUT             1

// Robust I/O helpers for pipe-based communication.
ssize_t read_full(int fd, void *buf, size_t n);
ssize_t write_full(int fd, const void *buf, size_t n);

#endif
