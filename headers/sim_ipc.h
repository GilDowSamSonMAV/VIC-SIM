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

#endif
