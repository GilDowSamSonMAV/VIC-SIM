#define _XOPEN_SOURCE_EXTENDED
#include <locale.h>
#include <ncurses.h>
#include <string.h>

#include "sim_ui.h"
#include "sim_const.h"
#include "sim_types.h"

// Single map window centered in the terminal
static WINDOW *map_win = NULL;

// Track last screen size to avoid unnecessary resizes
static int last_screen_h = 0;
static int last_screen_w = 0;

// Compute map window size/position and create/resize it if needed
static void layout_map_window(void)
{
    int H, W;
    getmaxyx(stdscr, H, W);

    if (H == last_screen_h && W == last_screen_w && map_win != NULL) {
        return;
    }

    last_screen_h = H;
    last_screen_w = W;

    // Leave some margin around the map area
    int wh = (H > 6) ? H - 6 : H;
    int ww = (W > 10) ? W - 10 : W;
    if (wh < 3) wh = 3;
    if (ww < 3) ww = 3;

    if (map_win == NULL) {
        map_win = newwin(wh, ww, (H - wh) / 2, (W - ww) / 2);
    } else {
        wresize(map_win, wh, ww);
        mvwin(map_win, (H - wh) / 2, (W - ww) / 2);
    }
}

// Initialize color pairs used by menus and titles
static void ui_setup_colors(void)
{
    if (!has_colors())
        return;

    start_color();
    use_default_colors();

    init_pair(1, COLOR_BLACK, COLOR_CYAN);   
    init_pair(2, COLOR_YELLOW, -1);          
    init_pair(3, COLOR_MAGENTA, -1);        
}

// Render the full-screen main menu and return selected option index
static int show_menu_internal(void)
{
    const char *options[] = { "Start Simulation", "Instructions", "Quit" };
    int n_options = 3;
    int choice = 0;
    int ch;

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    WINDOW *menu_win = newwin(rows, cols, 0, 0);
    keypad(menu_win, TRUE);

    while (1) {
        werase(menu_win);
        wborder(menu_win, '|', '|', '-', '-', '+', '+', '+', '+');

        const char *title = "=== DRONE SIMULATOR ===";
        wattron(menu_win, COLOR_PAIR(3) | A_BOLD);
        mvwprintw(menu_win, 2, (cols - (int)strlen(title)) / 2, "%s", title);
        wattroff(menu_win, COLOR_PAIR(3) | A_BOLD);

        int start_y = rows / 2 - n_options;
        for (int i = 0; i < n_options; i++) {
            int x = (cols - (int)strlen(options[i])) / 2;
            int y = start_y + i * 2;
            if (i == choice) {
                wattron(menu_win, COLOR_PAIR(1) | A_BOLD);
                mvwprintw(menu_win, y, x, "%s", options[i]);
                wattroff(menu_win, COLOR_PAIR(1) | A_BOLD);
            } else {
                wattron(menu_win, COLOR_PAIR(2));
                mvwprintw(menu_win, y, x, "%s", options[i]);
                wattroff(menu_win, COLOR_PAIR(2));
            }
        }

        wrefresh(menu_win);

        ch = wgetch(menu_win);
        switch (ch) {
            case KEY_UP:    if (choice > 0) choice--; break;
            case KEY_DOWN:  if (choice < n_options - 1) choice++; break;
            case '\n':
                delwin(menu_win);
                return choice;
        }
    }
}

// Show a modal instructions/help window centered in the screen
static void show_instructions_internal(void)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    int win_rows = rows - 4;
    int win_cols = cols - 10;
    int start_y = 2;
    int start_x = 5;

    WINDOW *instr_win = newwin(win_rows, win_cols, start_y, start_x);
    keypad(instr_win, TRUE);

    werase(instr_win);
    box(instr_win, 0, 0);

    const char *title = "=== INSTRUCTIONS ===";
    int title_x = (win_cols - (int)strlen(title)) / 2;
    wattron(instr_win, A_BOLD | COLOR_PAIR(3));
    mvwprintw(instr_win, 1, title_x, "%s", title);
    wattroff(instr_win, A_BOLD | COLOR_PAIR(3));

    const char *lines[] = {
        "Controls are in the INPUT window:",
        "q w e / a s d / z x c = direction of force",
        "s or SPACE = brake (zero force)",
        "r = reset drone position",
        "Q = quit simulation",
        "",
        "This window shows the map & state only.",
        "Resize the terminal to see the window adjust.",
        "",
        "Press any key to return to menu..."
    };
    int n_lines = (int)(sizeof(lines) / sizeof(lines[0]));
    int start_y_text = 3;
    for (int i = 0; i < n_lines; i++) {
        int x = (win_cols - (int)strlen(lines[i])) / 2;
        mvwprintw(instr_win, start_y_text + i, x, "%s", lines[i]);
    }

    wrefresh(instr_win);
    wgetch(instr_win);
    delwin(instr_win);
}

// Initialize ncurses and the main map window
void ui_init(void)
{
    setlocale(LC_ALL, "");
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);

    ui_setup_colors();
    layout_map_window();
    werase(stdscr);
    refresh();
}
// For defs check sim_ui.h 
void ui_shutdown(void)
{
    if (map_win) {
        delwin(map_win);
        map_win = NULL;
    }
    endwin();
}

int ui_show_start_menu(void)
{
    return show_menu_internal();
}

void ui_show_instructions(void)
{
    show_instructions_internal();
}

void ui_draw(const WorldState *world)
{
    if (!world) return;

    layout_map_window();

    int H, W;
    getmaxyx(map_win, H, W);

    werase(stdscr);
    werase(map_win);
    box(map_win, 0, 0);

    double wx = world->drone.x;
    double wy = world->drone.y;

    if (wx < 0.0) wx = 0.0;
    if (wx > SIM_WORLD_WIDTH)  wx = SIM_WORLD_WIDTH;
    if (wy < 0.0) wy = 0.0;
    if (wy > SIM_WORLD_HEIGHT) wy = SIM_WORLD_HEIGHT;

    // Map world coords to interior of map window (1..W-2, 1..H-2)
    int px = 1 + (int)((wx / SIM_WORLD_WIDTH)  * (W - 2));
    int py = 1 + (int)((wy / SIM_WORLD_HEIGHT) * (H - 2));

    if (px < 1) px = 1;
    if (px > W - 2) px = W - 2;
    if (py < 1) py = 1;
    if (py > H - 2) py = H - 2;

    // Status lines at the top of the main screen
    mvprintw(0, 0, "x=%6.2f y=%6.2f  vx=%6.2f vy=%6.2f",
             world->drone.x, world->drone.y,
             world->drone.vx, world->drone.vy);

    mvprintw(1, 0, "fx=%6.2f fy=%6.2f  brake=%d reset=%d quit=%d last_key=%d",
             world->cmd.fx, world->cmd.fy,
             world->cmd.brake, world->cmd.reset,
             world->cmd.quit, world->cmd.last_key);

    mvprintw(2, 0, "Press 'Q' in INPUT window to quit");

    // Draw drone symbol at mapped position inside the map window
    mvwaddch(map_win, py, px, '@'); 

    refresh();
    wrefresh(map_win);
}
