#include "algorithms.h"
#include <cmath>
#include <string>

/**
 * ALGORITHM: Search
 * Moves drone to next search waypoint, or transitions to perimeter state if temperature threshold is crossed.
 */
bool algo_search(Drone& drone, World& world) {
    if (drone.search_waypoints.empty()) return false;

    if (drone.current_temp >= SEARCH_THRESH) {
        drone.state = DroneState::PERIMETER;
        drone.perimeter_start_set = false;
        drone.perimeter_steps = 0;
        drone.orbit_dir = 1.0f;
        drone.last_action = "SEARCH → PERIMETER (temp threshold crossed)";
        return true;
    }

    Vec2 wp = drone.current_waypoint();

    if (drone.at_target(wp)) {
        if (drone.waypoint_idx >= drone.search_waypoints.size() - 1) {
            drone.state = DroneState::RETURN_HOME;
            drone.last_action = "search complete → RETURN HOME";
            return true;
        }
        drone.advance_waypoint();
        wp = drone.current_waypoint();
        drone.last_action = "waypoint reached, advancing";
    } 

    drone.move_toward(wp, world);
    drone.last_action = "moving to waypoint (" + 
                        std::to_string(wp.x) + "," + 
                        std::to_string(wp.y) + ")";
    return true;
}

/// ALGORITHM: Perimeter Tracing
// Follows fire boundary by moving forward when on boundary, and correcting in/out based on temperature feedback when drifting off.

bool boundaryTrace(Drone& drone, World& world) {
    // Set starting position first time we enter this state
    if (!drone.perimeter_start_set) {
        drone.perimeter_start = drone.pos;
        drone.perimeter_start_set = true;
        drone.perimeter_steps = 0;

         // Decide initial orbit direction based on facing direction at perimeter entry
        if (drone.facing == 'N' || drone.facing == 'E') {
            drone.fire_on_right = true;   // fire on right
        } else {
            drone.fire_on_right = false; // fire on left
        }

        drone.last_action = "perimeter start recorded at (" +
                            std::to_string(drone.pos.x) + "," +
                            std::to_string(drone.pos.y) + ")" + "perimeter start, orbit=" + 
                            std::string(drone.orbit_dir > 0 ? "CW" : "CCW");
        return true;
    }

    drone.perimeter_steps++;
    
    // Check if we've come full circle (min 20 steps before checking)
    if (drone.perimeter_steps > 20) {
        if (std::abs(drone.pos.x - drone.perimeter_start.x) <= 1 && std::abs(drone.pos.y - drone.perimeter_start.y) <= 1) {
            drone.state = DroneState::RETURN_HOME;
            drone.last_action = "perimeter complete → RETURN HOME";
            return true;
        }
    }

    std::string correction_reason = "on boundary";
    float temp_delta = std::abs(drone.current_temp - drone.last_temp);
    char previous_facing = drone.facing;

     // Priority order:
    // 1. If on boundary (LOW <= temp <= HIGH) → always move forward
    // 2. If too hot → turn away ONE step, then resume forward
    // 3. If too cold → turn in ONE step, then resume forward
    // Never let correction dominate over forward progress


    //when HOT
    if (drone.current_temp > HIGH_THRESH) {
        /*  
        if small oscillation occurs, correct, then contunue in same direction as before correction
        dont let small correction dominate over forward progress, just use it to nudge in the right 
        direction when we detect we're drifting too far off the boundary
        */
        if(temp_delta < 5.0f){ // if temp just spiked suddenly, likely a small oscillation, so correct but dont flip direction
             if (drone.fire_on_right) drone.moveLeft(world);
             else                     drone.moveRight(world);
             drone.setFacing(previous_facing); // reset facing to what it was before correction, so we continue in same direction after correction
             correction_reason = "temp spike detected, small correction";
        }
        if(temp_delta >= 5.0f){  // if temp increased significantly, we're potentially hitting a corner or drifting far off boundary, so flip orbit direction to try to get back to boundary faster
             if (drone.fire_on_right) drone.moveLeft(world);
             else                     drone.moveRight(world);
             correction_reason = "too hot, flipping orbit";
        }
    }

    //when COLD
    else if (drone.current_temp < LOW_THRESH) {
        if (drone.fire_on_right) drone.moveRight(world);
        else                     drone.moveLeft(world);
        correction_reason = "too cold, moving in";
    } 

    //when ON BOUNDARY
    else {
        drone.moveForward(world);
        correction_reason = "on boundary, following perimeter";
    }

    drone.last_action = "perimeter trace: " + correction_reason + " (" + std::to_string(drone.current_temp).substr(0,4) + "°C)";
    return true;
}

/// ALGORITHM: Bug1 Perimeter Tracing

bool bug1_perimeter(Drone& drone, World& world) {

    // ── Phase 0: approach fire until adjacent to hot cell ─────────
    if (!drone.perimeter_start_set) {
        
        // Check if any neighbor is hot
        bool hot_neighbor = 
            world.get_temp(drone.pos.x + 1, drone.pos.y) >= HIGH_THRESH ||
            world.get_temp(drone.pos.x - 1, drone.pos.y) >= HIGH_THRESH ||
            world.get_temp(drone.pos.x, drone.pos.y + 1) >= HIGH_THRESH ||
            world.get_temp(drone.pos.x, drone.pos.y - 1) >= HIGH_THRESH;

        if (!hot_neighbor) {
            // Keep moving toward fire — move toward hottest neighbor
            Vec2 directions[4] = {{1,0},{-1,0},{0,1},{0,-1}};
            Vec2 best_dir = {1, 0};
            float best_temp = -1.0f;
            
            for (auto& d : directions) {
                float t = world.get_temp(drone.pos.x + d.x, drone.pos.y + d.y);
                if (t > best_temp && t < HIGH_THRESH) {
                    best_temp = t;
                    best_dir = d;
                }
            }
            drone.move(best_dir, world);
            drone.last_action = "approaching fire boundary...";
            return true;
        }

        // Adjacent to hot cell — now set start and begin trace
        drone.perimeter_start = drone.pos;
        drone.perimeter_start_set = true;
        drone.perimeter_steps = 0;

        // Face toward the hottest neighbor so fire is on our right
        Vec2 dirs[4] = {{1,0},{-1,0},{0,1},{0,-1}};
        char dir_names[4] = {'E','W','S','N'};
        float best = -1.0f;
        for (int i = 0; i < 4; i++) {
            float t = world.get_temp(drone.pos.x + dirs[i].x, 
                                     drone.pos.y + dirs[i].y);
            if (t > best) { best = t; drone.facing = dir_names[i]; }
        }
        // Fire is now directly ahead — turn left so fire is on right
        drone.turnLeft();

        drone.last_action = "adjacent to fire, starting trace";
        return true;
    }

    drone.perimeter_steps++;

    // completion check
    if (drone.perimeter_steps > 20) {
        if (std::abs(drone.pos.x - drone.perimeter_start.x) <= 1 &&
            std::abs(drone.pos.y - drone.perimeter_start.y) <= 1) {
            drone.state = DroneState::RETURN_HOME;
            drone.last_action = "perimeter complete → RETURN HOME";
            return true;
        }
    }


    // ── Right-hand rule ───────────────────────────────────────────
    // Priority: turn right → go straight → turn left → turn around
    // "Too hot" counts as blocked (same as a physical wall)

    // Try turning right first (hug the fire)
    bool moved = false;

    // 1. Try right
    drone.turnRight();
    if (world.get_temp(drone.facing_cell().x, drone.facing_cell().y) <= HIGH_THRESH) {
        drone.moveForward(world);
        moved = true;
        drone.last_action = "right-hand: turned right";
    }

    // 2. Try straight
    if (!moved) {
        drone.turnLeft(); // undo the right turn
        if (world.get_temp(drone.facing_cell().x, drone.facing_cell().y) <= HIGH_THRESH) {
            drone.moveForward(world);
            moved = true;
            drone.last_action = "right-hand: went straight";
        }
    }

    // 3. Try left
    if (!moved) {
        drone.turnLeft();
        if (world.get_temp(drone.facing_cell().x, drone.facing_cell().y) <= HIGH_THRESH) {
            drone.moveForward(world);
            moved = true;
            drone.last_action = "right-hand: turned left";
        }
    }

    // 4. Turn around
    if (!moved) {
        drone.turnLeft(); // now facing backward
        drone.moveForward(world);
        drone.last_action = "right-hand: turned around";
    }

    return true;
}

/**
 * ALGORITHM: Return Home
 * Simply moves toward home position.
 */
bool algo_return_home(Drone& drone, World& world) {
    if (drone.at_target(drone.home)) {
        drone.state = DroneState::DONE;
        drone.last_action = "landed at home";
        return true;
    }
    drone.move_toward(drone.home, world);
    drone.last_action = "returning home";
    return true;
}