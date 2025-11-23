/*
    IPC identifiers for the shared WorldState stuct defined in sim_types.h.

    We use a single shared memory segment (SIM_SHM_WORLD) that stores the entire
    WorldState struct, and a single named semaphore (SIM_SEM_WORLD) as a global
    mutex to protect access to it.

    Design choice: one semaphore instead of multiple fine-grained ones keeps the
    concurrency model simple and safe (no lock ordering, no deadlocks), and the
    cost is negligible for our small world state and update rates.

    Making it more complex is always an option, but we opted for a KISS approach. 
*/

#ifndef SIM_IPC_H
#define SIM_IPC_H

#define SIM_SHM_WORLD   "/sim_world_shm"
#define SIM_SEM_WORLD   "/sim_world_sem"


/*
    Phase_Migration: pipe-based IPC support

    We are gradually moving from shared memory + semaphore to a pipe + select()
    architecture. To keep the transition smooth, we add pipe-related types and
    helper functions here, alongside the existing SHM identifiers.

    These helpers do not create or open pipes; they just operate on file
    descriptors that are created in master.c and passed to
    children.
*/

#include <sys/types.h>  
#include <stddef.h>  

#define SIM_FIFO_DRONE_CMD    "/tmp/sim_fifo_drone_cmd" // bb_server -> drone (CommandState)
#define SIM_FIFO_DRONE_STATE  "/tmp/sim_fifo_drone_state" // drone -> bb_server (DroneState)
#define SIM_FIFO_INPUT_CMD    "/tmp/sim_fifo_input_cmd" // input -> bb_server (CommandState)

// Robust I/O helpers for pipe-based communication.
// Implemented in src/sim_ipc.c.
ssize_t read_full(int fd, void *buf, size_t n);
ssize_t write_full(int fd, const void *buf, size_t n);

#endif
