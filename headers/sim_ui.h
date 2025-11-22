/*
    Ncurses-based UI for the bb_server.

    This module owns all ncurses setup/teardown and drawing logic. bb_server only
    calls the high-level interface below:

    - ui_init() / ui_shutdown(): initialize and clean up ncurses + windows.
    - ui_show_start_menu(): full-screen start menu, returns a UiMenuChoice.
    - ui_show_instructions(): modal help/instructions screen.
    - ui_draw(): render the current world snapshot (map + inspection panel).

    NB: The UI is read-only: it only gets a WorldState instant and never
    modifies shared memory directly.
*/

#ifndef SIM_UI_H
#define SIM_UI_H

#include "sim_types.h"

typedef enum {
    UI_MENU_START = 0,
    UI_MENU_INSTRUCTIONS = 1,
    UI_MENU_QUIT = 2
} UiMenuChoice;

void ui_init(void);

void ui_shutdown(void);

int ui_show_start_menu(void);

void ui_show_instructions(void);

void ui_draw(const WorldState *world);

#endif
