#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <signal.h>

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_const.h"
#include "sim_log.h"
#include "sim_ui.h"

// Crucial integer type used for providing variables that can be 
// read and written by both the main prog and sign handler
// without introducing race conditions. 
// Atomic -> Entire opetation completes before any interruption by sign handler. 
// Volative -> Ensures no optimization to avoid caching. 
static volatile sig_atomic_t running = 1; 

// Async-signal-safe SIGINT handler: just flip the running flag
static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

int main(void)
{
    sim_log_init("bb_server"); //Not used for now
    signal(SIGINT, handle_sigint);

    // Create/open shared memory
    int fd = shm_open(SIM_SHM_WORLD, O_CREAT | O_RDWR, 0666);
    if (fd == -1) {
        perror("bb_server: shm_open");
        return EXIT_FAILURE;
    }

    // Setting size of fd to match the WorldState struct
    if (ftruncate(fd, sizeof(WorldState)) == -1) {
        perror("bb_server: ftruncate");
        close(fd);
        return EXIT_FAILURE;
    }

    // Map the shared memory object into our address space
    WorldState *world = mmap(NULL, sizeof(WorldState),
                             PROT_READ | PROT_WRITE,
                             MAP_SHARED, fd, 0);
    if (world == MAP_FAILED) {
        perror("bb_server: mmap");
        close(fd);
        return EXIT_FAILURE;
    }

    //  Create/open semaphore
    sem_t *sem = sem_open(SIM_SEM_WORLD, O_CREAT, 0666, 1);
    if (sem == SEM_FAILED) {
        perror("bb_server: sem_open");
        munmap(world, sizeof(WorldState));
        close(fd);
        return EXIT_FAILURE;
    }

    // Initialize world state
    if (sem_wait(sem) == -1) {
        perror("bb_server: sem_wait (init)");
        return EXIT_FAILURE;
    } else {
        world->drone.x  = 0.0;
        world->drone.y  = 0.0;
        world->drone.vx = 0.0;
        world->drone.vy = 0.0;

        world->cmd.fx       = 0.0;
        world->cmd.fy       = 0.0;
        world->cmd.brake    = 0;
        world->cmd.reset    = 0;
        world->cmd.quit     = 0;
        world->cmd.last_key = 0;

        sem_post(sem);
    }

    // Init UI and show menu
    ui_init();

    int start_sim = 0;
    while (!start_sim && running) { // Handling choices of menu
        int choice = ui_show_start_menu();
        if (choice == UI_MENU_QUIT) {
            running = 0;
        } else if (choice == UI_MENU_INSTRUCTIONS) {
            ui_show_instructions();
        } else if (choice == UI_MENU_START) {
            start_sim = 1;
        }
    }

    // Shutting down properly
    if (!running) {
        ui_shutdown();
        sem_close(sem);
        munmap(world, sizeof(WorldState));
        close(fd);
        shm_unlink(SIM_SHM_WORLD);
        sem_unlink(SIM_SEM_WORLD);
        return 0;
    }

    sim_log_info("bb_server: entering main loop\n");

    // Main display loop
    while (running) {
        WorldState snapshot;

        if (sem_wait(sem) == -1) {
            perror("bb_server: sem_wait (loop)");
            break;
        }
        snapshot = *world;
        sem_post(sem);

        ui_draw(&snapshot);

        if (snapshot.cmd.quit) {
            sim_log_info("bb_server: quit flag set, exiting\n");
            break;
        }

        usleep(33333); // Approx 30 Hz
    }

    ui_shutdown();

    sem_close(sem);
    munmap(world, sizeof(WorldState));
    close(fd);

    // Cleanup shared memory & semaphore
    shm_unlink(SIM_SHM_WORLD);
    sem_unlink(SIM_SEM_WORLD);

    sim_log_info("bb_server: exited\n");
    return EXIT_SUCCESS;
}
