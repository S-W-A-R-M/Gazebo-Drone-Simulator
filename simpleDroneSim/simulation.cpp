#include "simulation.h"
#include "renderer.h"
#include "algorithms.h"
#include <iostream>
#include <thread>
#include <chrono>

Simulation::Simulation(int max_steps)
    : world(GRID_W, GRID_H),
      drone(0, 0),
      max_steps(max_steps)
{}

void Simulation::setup() {

    //PASS
    // world.add_fire(8.0f, 8.0f, 18.0f);
    // world.add_fire(9.0f, 6.0f, 10.0f);
    // world.add_fire(10.0f, 10.0f, 30.0f, 90.0f);
    // world.add_fire(10.0f, 7.0f, 10.0f, 90.0f);

    //test another fire pattern PASS
    // world.add_fire(15.0f, 10.0f, 40.0f, 90.0f);
    // world.add_fire(18.0f, 12.0f, 20.0f, 80.0f);
    // world.add_fire(12.0f, 8.0f, 20.0f, 80.0f);

    // L-Shape Fire PASS
    // world.add_fire(12.0f, 6.0f, 20.0f);
    // world.add_fire(12.0f, 10.0f, 20.0f);
    // world.add_fire(12.0f, 14.0f, 20.0f);
    // world.add_fire(16.0f, 14.0f, 20.0f);
    // world.add_fire(20.0f, 14.0f, 20.0f);

    // // Dumbbell Fire PASS 
    world.add_fire(8.0f, 10.0f, 35.0f);  // Left lobe
    world.add_fire(22.0f, 10.0f, 35.0f); // Right lobe
    world.add_fire(15.0f, 10.0f, 15.0f); // Narrow connecting bridge


    // // C-Shape Fire  PASS 
    // world.add_fire(10.0f, 6.0f,  20.0f); // Top lip
    // world.add_fire(15.0f, 6.0f,  20.0f);
    // world.add_fire(18.0f, 8.0f,  20.0f); // Back wall
    // world.add_fire(18.0f, 12.0f, 20.0f);
    // world.add_fire(15.0f, 14.0f, 20.0f); // Bottom lip
    // world.add_fire(10.0f, 14.0f, 20.0f);

    // Irregular Bumpy Wall  PASS
    // world.add_fire(8.0f,  12.0f, 18.0f);
    // world.add_fire(11.0f, 10.0f, 22.0f);
    // world.add_fire(14.0f, 13.0f, 15.0f);
    // world.add_fire(17.0f, 9.0f,  25.0f);
    // world.add_fire(21.0f, 11.0f, 20.0f);



    drone.build_lawnmower(0, 0, 24, 18, 2);
    drone.sense(world);

    Renderer::draw(world, drone);
}

bool Simulation::step() {
    if (drone.state == DroneState::DONE || drone.steps >= max_steps) {
        running = false;
        return false;
    }

    drone.sense(world);

    switch (drone.state) {
        case DroneState::SEARCH:
            algo_search(drone, world);
            break;
        case DroneState::PERIMETER:
            bug1_perimeter(drone, world);
            break;
        case DroneState::RETURN_HOME:
            algo_return_home(drone, world);
            break;
        case DroneState::DONE:
            running = false;
            return false;
    }

    drone.sense(world);
    return true;
}

void Simulation::run_auto(int delay_ms) {
    setup();
    while (running) {
        bool ok = step();
        Renderer::draw(world, drone);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        if (!ok) break;
    }
    print_summary();
}

void Simulation::run_manual() {
    setup();
    std::cout << "\nPress ENTER to step, 'q' + ENTER to quit, 'a' + ENTER for auto...\n";
    std::string input;
    while (running) {
        std::getline(std::cin, input);
        if (input == "q" || input == "Q") break;
        if (input == "a" || input == "A") {
            while (running) {
                bool ok = step();
                Renderer::draw(world, drone);
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(STEP_DELAY_MS));
                if (!ok) break;
            }
            break;
        }

        if(input == "b" || input == "B"){
            if(drone.steps > 0){
                drone.steps -= 2; 
                drone.pos = drone.last_pos; 
                drone.sense(world); 
            }
            continue;
        }

        bool ok = step();
        Renderer::draw(world, drone);
        if (!ok) break;
    }
    print_summary();
}

void Simulation::print_summary() {
    std::cout << "\n" << BOLD << CYAN;
    std::cout << "══════════════ MISSION COMPLETE ══════════════\n" << RESET;
    std::cout << "  Total steps    : " << drone.steps << "\n";
    std::cout << "  Cells visited  : ";
    int count = 0;
    for (auto& row : world.visited)
        for (bool v : row) if (v) count++;
    std::cout << count << " / " << (GRID_W * GRID_H) << "\n";
    std::cout << "  Final state    : " << state_name(drone.state) << "\n";
    std::cout << "  Final position : (" 
              << drone.pos.x << ", " << drone.pos.y << ")\n";
    std::cout << BOLD << CYAN;
    std::cout << "══════════════════════════════════════════════\n" << RESET;
}