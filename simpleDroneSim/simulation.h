#ifndef SIMULATION_H
#define SIMULATION_H

#include "world.h"
#include "drone.h"
#include "config.h"

class Simulation {
public:
    World world;
    Drone drone;
    int   max_steps;
    bool  running = true;

    Simulation(int max_steps = 2000);

    void setup();
    bool step();
    void run_auto(int delay_ms = STEP_DELAY_MS);
    void run_manual();
    void print_summary();
};

#endif // SIMULATION_H