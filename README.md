# POSIX Drone Simulator - Assignment 1

This repo implements the Assignment 1 architecture of a multi-process drone simulator using POSIX IPC and `ncurses`.

```text
posix-drone-sim/
├── bin/
│   ├── conf/
│   │   └── drone_parameters.conf    # parameters for drone + server
│   └── log/
│       └── .gitkeep                 # runtime logs (per process later)
│
├── build/                           # out-of-source build (cmake)
│   └── (generated)
│
├── docs/
│   ├── assignmentsv4.1.pdf          # assignment spec
│   └── architecture_a1.md           # Image/Flow showcasing architecture
│
├── headers/
│   ├── CMakeLists.txt
│   ├── constants.h                  # paths, sizes, default M,K,DT
│   ├── dataStructs.h                # DroneState, CommandMsg, etc.
│   ├── ipc.h                        # FIFO paths + send/recv prototypes
│   ├── log.h                        # logging interface
│   ├── params.h                     # config loading
│   ├── utils/
│   │   ├── utils.c
│   │   └── utils.h
│   └── wrapFuncs/
│       ├── wrapFunc.c               # wrappers with error checking
│       └── wrapFunc.h
│
├── src/
│   ├── CMakeLists.txt
│   ├── bb_server.c                  # A1: blackboard + ncurses + select()
│   ├── drone.c                      # A1: dynamics + Euler + pipe I/O
│   ├── input.c                      # A1: reads keys, sends commands
│   └── watchdog.c                   # A1: monitors heartbeats
│
├── .github/
│   └── workflows/
│       └── ci.yml                   # simple build check
│
├── CMakeLists.txt                   # root
├── run.sh                           # build
├── .gitignore
├── README.md
└── LICENSE
