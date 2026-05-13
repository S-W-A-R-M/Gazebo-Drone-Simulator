#ifndef DRONE_H
#define DRONE_H

#include <vector>
#include <string>
#include "config.h"
#include "world.h"

class Drone {
public:
    Vec2        pos;
    Vec2        home;
    DroneState  state = DroneState::SEARCH;
    float       current_temp = AMBIENT_TEMP;
    float       last_temp    = AMBIENT_TEMP;
    Vec2        last_pos;
    int         steps = 0;
    int         perimeter_steps = 0;
    bool        perimeter_start_set = false;
    Vec2        perimeter_start;
    float       orbit_dir = 1.0f;   
    std::string last_action = "initialized";
    char        facing = 'S';  
    bool        fire_on_right; 

    std::vector<Vec2> search_waypoints;
    size_t waypoint_idx = 0;
    std::vector<Vec2> trail;

    Drone(int x, int y);

    bool move(Vec2 direction, World& world);
    bool move_north(World& w);
    bool move_south(World& w);
    bool move_east(World& w);  
    bool move_west(World& w);

    bool moveRight(World& w);
    bool moveLeft(World& w);
    bool moveForward(World& w);
    bool setFacing(char dir);

    bool move_toward(Vec2 target, World& world);
    bool at_target(Vec2 target) const;
    float sense(World& world);

    void build_lawnmower(int x0, int y0, int x1, int y1, int lane_spacing = 2);
    Vec2 current_waypoint() const;
    void advance_waypoint();
};

#endif // DRONE_H