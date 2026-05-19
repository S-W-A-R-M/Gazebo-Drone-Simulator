#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "drone.h"
#include "world.h"

bool algo_search(Drone& drone, World& world);
bool boundaryTrace(Drone& drone, World& world);
bool bug1_perimeter(Drone& drone, World& world);
bool algo_return_home(Drone& drone, World& world);

#endif // ALGORITHMS_H