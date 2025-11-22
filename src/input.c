#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <signal.h>
#include <curses.h>
#include <math.h>

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

// Helper to clamp a value to [-max, +max]
static double clamp(double v, double max)
{
    if (v >  max) return  max;
    if (v < -max) return -max;
    return v;
}

// Render the input UI (direction pad + flags + current command state)
static void draw_ui(const CommandState *cmd)
{
    int H, W;
    getmaxyx(stdscr, H, W);

    erase();

    mvprintw(0, 0, "INPUT PROCESS (ncurses) - controls:");
    mvprintw(2, 0, "Direction pad:");

    // PAD: 3x3 box just under the label
    int pad_y = 3;          
    int pad_x = 2;        

    mvprintw(pad_y + 0, pad_x, "+---+---+---+");
    mvprintw(pad_y + 1, pad_x, "| q | w | e |");
    mvprintw(pad_y + 2, pad_x, "+---+---+---+");
    mvprintw(pad_y + 3, pad_x, "| a | s | d |");
    mvprintw(pad_y + 4, pad_x, "+---+---+---+");
    mvprintw(pad_y + 5, pad_x, "| z | x | c |");
    mvprintw(pad_y + 6, pad_x, "+---+---+---+");

    // Highlight positions of letters in that frame
    struct KeyCell { int ch; int y; int x; } cells[] = {
        { 'q', pad_y + 1, pad_x + 2 },
        { 'w', pad_y + 1, pad_x + 6 },
        { 'e', pad_y + 1, pad_x +10 },
        { 'a', pad_y + 3, pad_x + 2 },
        { 's', pad_y + 3, pad_x + 6 },
        { 'd', pad_y + 3, pad_x +10 },
        { 'z', pad_y + 5, pad_x + 2 },
        { 'x', pad_y + 5, pad_x + 6 },
        { 'c', pad_y + 5, pad_x +10 },
    };

    // Treat space the same as 's' for highlighting (brake in center)
    int lk = cmd->last_key;
    if (lk == ' ') lk = 's';

    for (size_t i = 0; i < sizeof(cells)/sizeof(cells[0]); ++i) {
        if (cells[i].ch == lk) {
            attron(A_REVERSE);
            mvaddch(cells[i].y, cells[i].x, cells[i].ch);
            attroff(A_REVERSE);
        }
    }

    // Special keys section under the pad
    int special_y = pad_y + 8;  
    mvprintw(special_y + 0, 0, "Special:");
    mvprintw(special_y + 1, 2, "s or SPACE = brake (zero force)");
    mvprintw(special_y + 2, 2, "r          = reset drone");
    mvprintw(special_y + 3, 2, "Q          = quit simulation & exit input");

    // Current command state (forces + flags + last key)
    int cmd_y = special_y + 5;
    mvprintw(cmd_y + 0, 0, "Current command:");
    mvprintw(cmd_y + 1, 2, "fx = %6.2f  fy = %6.2f", cmd->fx, cmd->fy);
    mvprintw(cmd_y + 2, 2, "brake = %d  reset = %d  quit = %d",
             cmd->brake, cmd->reset, cmd->quit);
    mvprintw(cmd_y + 3, 2, "last_key = %d (%c)",
             cmd->last_key,
             (cmd->last_key >= 32 && cmd->last_key <= 126)
                 ? cmd->last_key : '.');

    refresh();
}

int main(void)
{
    sim_log_init("input"); //Not used yet
    signal(SIGINT, handle_sigint);

    // Attach to shared memory created by bb_server
    int fd = shm_open(SIM_SHM_WORLD, O_RDWR, 0666);
    if (fd == -1) {
        perror("input: shm_open");
        return EXIT_FAILURE;
    }

    // Map the shared WorldState into this process's address space
    WorldState *world = mmap(NULL, sizeof(WorldState),
                             PROT_READ | PROT_WRITE,
                             MAP_SHARED, fd, 0);
    if (world == MAP_FAILED) {
        perror("input: mmap");
        close(fd);
        return EXIT_FAILURE;
    }

    // Open existing semaphore for synchronized access to WorldState
    sem_t *sem = sem_open(SIM_SEM_WORLD, 0);
    if (sem == SEM_FAILED) {
        perror("input: sem_open");
        munmap(world, sizeof(WorldState));
        close(fd);
        return EXIT_FAILURE;
    }

    // Local tuning constants for force increments (later move to sim_params)
    const double FORCE_STEP = 1.5;
    const double MAX_FORCE  = 15.0;
    const double INV_SQRT2  = 0.70710678118;

    // Initialize ncurses in this terminal for the control pad UI
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    timeout(100);

    sim_log_info("input: started (ncurses)\n");

    // Main input loop: read commands, update based on keys, write back to SHM
    while (running) {
        CommandState cmd;

        if (sem_wait(sem) == -1) {
            perror("input: sem_wait (read)");
            break;
        }
        cmd = world->cmd;
        sem_post(sem);

        if (cmd.quit) {
            sim_log_info("input: quit flag seen, exiting\n");
            break;
        }

        draw_ui(&cmd);

        // Read a key (non-blocking-ish because of timeout)
        int ch = getch();
        if (ch == ERR) {
            continue;
        }

        // Update command state according to key press
        cmd.last_key = ch;
        cmd.brake    = 0; 

        double fx = cmd.fx;
        double fy = cmd.fy;

        switch (ch) {
            case 'q':
                fx -= FORCE_STEP * INV_SQRT2;
                fy -= FORCE_STEP * INV_SQRT2;
                break;
            case 'e':
                fx += FORCE_STEP * INV_SQRT2;
                fy -= FORCE_STEP * INV_SQRT2;
                break;
            case 'z':
                fx -= FORCE_STEP * INV_SQRT2;
                fy += FORCE_STEP * INV_SQRT2;
                break;
            case 'c':
                fx += FORCE_STEP * INV_SQRT2;
                fy += FORCE_STEP * INV_SQRT2;
                break;

            case 'w':
                fy -= FORCE_STEP;
                break;
            case 'x':
                fy += FORCE_STEP;
                break;
            case 'a':
                fx -= FORCE_STEP;
                break;
            case 'd':
                fx += FORCE_STEP;
                break;

            case 's':
                fx = 0.0;
                fy = 0.0;
                cmd.brake = 1;
                break;

            case 'r':
                cmd.reset = 1;
                fx = 0.0;
                fy = 0.0;
                break;

            case 'Q':
                cmd.quit  = 1;
                running   = 0;
                break;

            default:
                break;
        }

        // Clamp command forces to the allowed range
        fx = clamp(fx, MAX_FORCE);
        fy = clamp(fy, MAX_FORCE);

        cmd.fx = fx;
        cmd.fy = fy;

        // Write back updated command into shared memory
        if (sem_wait(sem) == -1) {
            perror("input: sem_wait (write)");
            break;
        }
        world->cmd = cmd;
        sem_post(sem);
    }

    sim_log_info("input: exiting\n");

    // Restore terminal and release IPC resources
    endwin();
    sem_close(sem);
    munmap(world, sizeof(WorldState));
    close(fd);

    return EXIT_SUCCESS;
}
