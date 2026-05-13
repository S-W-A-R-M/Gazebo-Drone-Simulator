#include "drone.h"
#include <cmath>

Drone::Drone(int x, int y) : pos({x, y}), home({x, y}), last_pos({x, y}) {}

bool Drone::move(Vec2 direction, World& world) {
    Vec2 next = pos + direction;
    if(direction.x == 1) facing = 'E';
    else if(direction.x == -1) facing = 'W';
    else if(direction.y == 1) facing = 'S';
    else if(direction.y == -1) facing = 'N';
    if (!world.in_bounds(next.x, next.y)) return false;
    last_pos = pos;
    pos = next;
    trail.push_back(pos);
    steps++;
    return true;
}

bool Drone::move_north(World& w) { return move({0, -1}, w); }
bool Drone::move_south(World& w) { return move({0,  1}, w); }
bool Drone::move_east (World& w) { return move({ 1, 0}, w); }  
bool Drone::move_west (World& w) { return move({-1, 0}, w); }

bool Drone::moveRight(World& w){
    if(facing == 'N') return move_east(w);
    else if(facing == 'E') return move_south(w);
    else if(facing == 'S') return move_west(w);
    else if(facing == 'W') return move_north(w);
    return false;
}

bool Drone::moveLeft(World& w){
    if(facing == 'N') return move_west(w);
    else if(facing == 'E') return move_north(w);
    else if(facing == 'S') return move_east(w);
    else if(facing == 'W') return move_south(w);
    return false;
}

bool Drone::moveForward(World& w){
    if(facing == 'N') return move_north(w);
    else if(facing == 'E') return move_east(w);
    else if(facing == 'S') return move_south(w);
    else if(facing == 'W') return move_west(w);
    return false;
}

bool Drone::setFacing(char dir){
    if(dir == 'N' || dir == 'E' || dir == 'S' || dir == 'W'){
        facing = dir;
        return true;
    }
    return false;
}

bool Drone::move_toward(Vec2 target, World& world) {
    int dx = target.x - pos.x;
    int dy = target.y - pos.y;
    if (std::abs(dx) >= std::abs(dy)) {
        if (dx != 0) return move({(dx > 0 ? 1 : -1), 0}, world);
        if (dy != 0) return move({0, (dy > 0 ? 1 : -1)}, world);
    } else {
        if (dy != 0) return move({0, (dy > 0 ? 1 : -1)}, world);
        if (dx != 0) return move({(dx > 0 ? 1 : -1), 0}, world);
    }
    return false;
}

bool Drone::at_target(Vec2 target) const {
    return pos == target;
}

float Drone::sense(World& world) {
    last_temp    = current_temp;
    current_temp = world.sense_temp(pos.x, pos.y);
    return current_temp;
}

void Drone::build_lawnmower(int x0, int y0, int x1, int y1, int lane_spacing) {
    search_waypoints.clear();
    waypoint_idx = 0;
    bool forward = true;
    for (int x = x0; x <= x1; x += lane_spacing) {
        if (forward) {
            for (int y = y0; y <= y1; y++)
                search_waypoints.push_back({x, y});
        } else {
            for (int y = y1; y >= y0; y--)
                search_waypoints.push_back({x, y});
        }
        forward = !forward;
    }
    search_waypoints.push_back({home.x, home.y});
}

Vec2 Drone::current_waypoint() const {
    if (waypoint_idx >= search_waypoints.size())
        return search_waypoints.back();
    return search_waypoints[waypoint_idx];
}

void Drone::advance_waypoint() {
    if (waypoint_idx < search_waypoints.size() - 1)
        waypoint_idx++;
}