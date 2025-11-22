# POSIX Drone Simulator – Assignment 1

This repository contains the implementation for the first ARP course assignment: a `multi-process` drone game simulator utilizing `POSIX IPC` and the `ncurses` library.

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

