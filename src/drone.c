#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>     
#include <fcntl.h>      
#include <sys/mman.h>   
#include <semaphore.h>  
#include <signal.h>    
#include <sys/types.h>

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_const.h"
#include "sim_log.h"

// Flag set by the SIGINT handler to request a clean shutdown
static volatile sig_atomic_t running = 1;

// Async-signal-safe SIGINT handler: just flip the running flag
static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

// Keep drone inside [0, SIM_WORLD_WIDTH] × [0, SIM_WORLD_HEIGHT]
static void apply_world_bounds(DroneState *d)
{
    if (d->x < 0.0) {
        d->x = 0.0;
        if (d->vx < 0.0) d->vx = 0.0;
    } else if (d->x > SIM_WORLD_WIDTH) {
        d->x = SIM_WORLD_WIDTH;
        if (d->vx > 0.0) d->vx = 0.0;
    }

    if (d->y < 0.0) {
        d->y = 0.0;
        if (d->vy < 0.0) d->vy = 0.0;
    } else if (d->y > SIM_WORLD_HEIGHT) {
        d->y = SIM_WORLD_HEIGHT;
        if (d->vy > 0.0) d->vy = 0.0;
    }
}

int main(void)
{
    sim_log_init("drone"); // Not used yet
    signal(SIGINT, handle_sigint);

    // Open shared memory created by bb_server
    int fd = shm_open(SIM_SHM_WORLD, O_RDWR, 0666);
    if (fd == -1) {
        perror("drone: shm_open");
        return EXIT_FAILURE;
    }

    // Map the shared WorldState into this process's address space
    WorldState *world = mmap(NULL, sizeof(WorldState),
                             PROT_READ | PROT_WRITE,
                             MAP_SHARED, fd, 0);
    if (world == MAP_FAILED) {
        perror("drone: mmap");
        close(fd);
        return EXIT_FAILURE;
    }

    // Open existing global semaphore for synchronized world access
    sem_t *sem = sem_open(SIM_SEM_WORLD, 0);
    if (sem == SEM_FAILED) {
        perror("drone: sem_open");
        munmap(world, sizeof(WorldState));
        close(fd);
        return EXIT_FAILURE;
    }

    const double dt      = SIM_DEFAULT_DT;       
    const double mass    = SIM_DEFAULT_MASS;     
    const double damping = SIM_DEFAULT_DAMPING;

    // Precompute sleep interval matching the simulation timestep
    unsigned int sleep_us = (unsigned int)(dt * 1e6);

    sim_log_info("drone: started (dt=%.3f, M=%.3f, K=%.3f)\n",
                 dt, mass, damping);

    // Main simulation loop: read commands, integrate dynamics, write back state
    while (running) {
        DroneState   d;
        CommandState c;
        int quit_flag  = 0;
        int reset_flag = 0;

        // Read a consistent snapshot of drone and command state from shared memory
        if (sem_wait(sem) == -1) {
            perror("drone: sem_wait (read)");
            break;
        }
        d          = world->drone;
        c          = world->cmd;
        quit_flag  = world->cmd.quit;
        reset_flag = world->cmd.reset;
        if (sem_post(sem) == -1) {
            perror("drone: sem_post (read)");
            break;
        }

        // Respect global quit flag set by the input or bb_server
        if (quit_flag) {
            sim_log_info("drone: quit flag set, exiting\n");
            break;
        }

        // If reset requested, zero position and velocity once and clear the flag
        if (reset_flag) {
            d.x  = 0.0;
            d.y  = 0.0;
            d.vx = 0.0;
            d.vy = 0.0;
            c.reset = 0;
        }
        else{

        // Compute acceleration from command forces and viscous damping
        double fx = c.fx;
        double fy = c.fy;

        double ax = (fx - damping * d.vx) / mass;
        double ay = (fy - damping * d.vy) / mass;

        // Integrate velocity and position with an explicit Euler step
        d.vx += ax * dt;
        d.vy += ay * dt;

        d.x  += d.vx * dt;
        d.y  += d.vy * dt;
        }

        apply_world_bounds(&d);

        // Write updated drone state (and cleared reset flag) back into shared memory
        if (sem_wait(sem) == -1) {
            perror("drone: sem_wait (write)");
            break;
        }
        world->drone     = d;
        world->cmd.reset = c.reset;
        if (sem_post(sem) == -1) {
            perror("drone: sem_post (write)");
            break;
        }
        usleep(sleep_us);
    }

    sim_log_info("drone: exiting\n");

    // Clean up local handles to the semaphore and shared memory
    sem_close(sem);
    munmap(world, sizeof(WorldState));
    close(fd);

    return EXIT_SUCCESS;
}
