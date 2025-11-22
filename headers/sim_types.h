/*
    Shared data model for the simulator.

    DroneState  = physical state of the drone (position and velocity).
    CommandState = user command state (forces and control flags), written by the
                input process and consumed by the drone + UI.
    WorldState  = the full "blackboard" snapshot stored in shared memory and
                protected by a single global semaphore. All processes attach
                to the same WorldState instance and only access it under the
                SIM_SEM_WORLD mutex (check sim_ipc.h for more details).
*/

#ifndef SIM_TYPES_H
#define SIM_TYPES_H

typedef struct {
    double x;
    double y;
    double vx;
    double vy;
} DroneState;

typedef struct {
    double fx;       
    double fy;       
    int    brake;    
    int    reset;    
    int    quit;     
    int    last_key; 
} CommandState;

typedef struct {
    DroneState   drone; 
    CommandState cmd;   
    // Add target and obstacles here
} WorldState;

#endif
