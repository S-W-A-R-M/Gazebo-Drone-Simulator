#include <iostream>
#include <string>
#include "simulation.h"
#include "config.h"

int main(int argc, char* argv[]) {
    std::cout << BOLD << "\nSWARM Search Algorithm Simulator\n" << RESET;
    std::cout << "Compile Example: g++ -std=c++17 -o swarm_sim driver.cpp simulation.cpp algorithms.cpp drone.cpp world.cpp renderer.cpp\n\n";
    std::cout << "Mode? [a]uto / [m]anual : ";

    std::string mode;
    std::getline(std::cin, mode);

    Simulation sim(2000);

    if (mode == "m" || mode == "M" || mode == "manual") {
        sim.run_manual();
    } else {
        sim.run_auto(STEP_DELAY_MS);
    }

    return 0;
}