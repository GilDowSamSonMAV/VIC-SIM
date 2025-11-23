# POSIX Drone Simulator – Assignment 1

This repository contains the implementation for the first ARP course assignment: a `multi-process` drone game simulator utilizing `POSIX IPC` and the `ncurses` library.

### IPC Design: From Shared Memory to Named Pipes

In the **Phase 1** commits we implemented the simulator using **POSIX shared memory + a named semaphore**:

- `bb_server` created a shared memory segment that contained a single `WorldState` struct.
- All other processes attached to that segment and used the same `WorldState`.
- A single named semaphore was used as a global mutex to protect reads/writes.
- This worked well and was simple: one global struct, one lock, everyone reads/writes it.

However, the assignment explicitly asked us to use pipes and `select()` for inter-process communication, and shared memory alone was considered too simple.

---

In the **Phase_Migration** commits we kept the same logical architecture, but we **replaced shared memory with named pipes (FIFOs)**:

- We defined three FIFOs in `sim_ipc.h`:
  - `SIM_FIFO_DRONE_CMD`   commands from `bb_server` → `drone` (`CommandState`)
  - `SIM_FIFO_DRONE_STATE` drone state from `drone` → `bb_server` (`DroneState`)
  - `SIM_FIFO_INPUT_CMD`   user commands from `input` → `bb_server` (`CommandState`)
- `master` creates these FIFOs with `mkfifo()` at startup and removes them on exit.
- Each process opens the FIFOs by name, not by inherited file descriptors:
  - `bb_server` does `open()` on all three FIFOs and uses `select()` to multiplex:
    - drone state FIFO + input command FIFO.
  - `drone` opens its command FIFO (read) and state FIFO (write) and runs the physics loop.
  - `input` opens its command FIFO (write) and sends a new `CommandState` whenever a key is pressed.

We initially tried using anonymous pipes with file descriptors passed via `argv`, but that approach broke when launching processes inside terminal emulators (Terminator/Konsole): those programs create a new pseudo-terminal and close most inherited FDs, so the pipe ends passed via `argv` were no longer valid.  

Named pipes (FIFOs) solve this cleanly:

- The IPC channel lives in the filesystem (e.g. `/tmp/sim_fifo_*`), not in inherited FDs.
- Each process can be started in its own terminal window and still connect correctly by calling `open()` on the same FIFO path.
- We still satisfy the requirement to use **pipes + `select()`**, but with a setup that is robust to how terminals spawn processes.

To run 

```
./run.sh
```

Architecture

```text
posix-drone-sim/
├── bin/
│   ├── conf/
│   │   └── drone_parameters.conf
│   └── log/
│       └── .gitkeep
│
├── build/
│   └── (generated)
│
├── files/
│   ├── assignmentsv4.1.pdf
│   └── architecture.png
│
├── headers/
│   ├── CMakeLists.txt
│   ├── sim_const.h      # constants (sizes, defaults)
│   ├── sim_types.h      # shared structs: DroneState, WorldState, CommandMsg, ...
│   ├── sim_ipc.h        # shm/sem/FIFO names, IPC enums
│   ├── sim_log.h        # logging interface
│   └── sim_params.h     # config/parameters interface
│
├── src/
│   ├── CMakeLists.txt
│   ├── master.c         # spawns everything, sets up IPC
│   ├── bb_server.c      # blackboard + ncurses UI
│   ├── drone.c          # dynamics (Euler)
│   ├── input.c          # keyboard → command forces
│   ├── obstacles.c      # obstacles generator
│   ├── targets.c        # targets generator
│   ├── sim_log.c        # implementation of sim_log.h
│   └── sim_params.c     # implementation of sim_params.h
│
├── .github/
│   └── workflows/
│       └── ci.yml
│
├── CMakeLists.txt
├── run.sh
├── .gitignore
├── README.md
└── LICENSE

