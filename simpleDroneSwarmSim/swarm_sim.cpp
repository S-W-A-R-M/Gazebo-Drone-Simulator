/**
 * SWARM Search Algorithm Simulator
 * ─────────────────────────────────
 * A lightweight 2D grid simulator for testing drone search and fire
 * perimeter tracing algorithms before deploying to ROS/Gazebo.
 *
 * Compile:  g++ -std=c++17 -o swarm_sim swarm_sim.cpp
 * Run:      ./swarm_sim
 */

#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <iomanip>
#include <algorithm>
#include <thread>
#include <chrono>
#include <functional>
#include <sstream>

// ═══════════════════════════════════════════════════════════════════
//  CONFIGURATION — tweak these to change the simulation
// ═══════════════════════════════════════════════════════════════════

constexpr int   GRID_W          = 30;       // grid width  (columns)
constexpr int   GRID_H          = 20;       // grid height (rows)
constexpr float AMBIENT_TEMP    = 20.0f;    // background temperature °C
constexpr float FIRE_PEAK       = 100.0f;   // temperature at fire center °C
constexpr float SEARCH_THRESH   = 50.0f;    // enter PERIMETER above this °C
constexpr float HIGH_THRESH     = 70.0f;    // too hot, back off
constexpr float LOW_THRESH      = 45.0f;    // too cold, move in
constexpr int   STEP_DELAY_MS   = 120;      // ms between auto-steps (speed)


// ═══════════════════════════════════════════════════════════════════
//  ANSI COLOR HELPERS
// ═══════════════════════════════════════════════════════════════════

#define RESET       "\033[0m"
#define BOLD        "\033[1m"
#define RED         "\033[31m"
#define YELLOW      "\033[33m"
#define GREEN       "\033[32m"
#define CYAN        "\033[36m"
#define BLUE        "\033[34m"
#define MAGENTA     "\033[35m"
#define WHITE       "\033[97m"
#define DIM         "\033[2m"
#define BG_RED      "\033[41m"
#define BG_YELLOW   "\033[43m"
#define BG_BLUE     "\033[44m"
#define BG_GREEN    "\033[42m"


// ═══════════════════════════════════════════════════════════════════
//  TYPES
// ═══════════════════════════════════════════════════════════════════

struct Vec2 {
    int x, y;
    bool operator==(const Vec2& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Vec2& o) const { return !(*this == o); }
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
};

enum class DroneState {
    SEARCH,
    PERIMETER,
    RETURN_HOME,
    DONE
};

std::string state_name(DroneState s) {
    switch(s) {
        case DroneState::SEARCH:      return "SEARCH";
        case DroneState::PERIMETER:   return "PERIMETER";
        case DroneState::RETURN_HOME: return "RETURN HOME";
        case DroneState::DONE:        return "DONE";
    }
    return "UNKNOWN";
}

struct FireSource {
    float cx, cy;   // center (float for sub-cell placement)
    float spread;   // controls radius of heat (higher = wider fire)
    float peak;     // peak temperature at center
};


// ═══════════════════════════════════════════════════════════════════
//  WORLD — the grid, fire sources, and temperature field
// ═══════════════════════════════════════════════════════════════════

class World {
public:
    int w, h;
    std::vector<FireSource> fires;
    std::vector<std::vector<float>> temp_field;     // ground truth temps
    std::vector<std::vector<float>> visited_temps;  // what drone has seen
    std::vector<std::vector<bool>>  visited;

    World(int w, int h) : w(w), h(h) {
        temp_field.assign(h, std::vector<float>(w, AMBIENT_TEMP));
        visited_temps.assign(h, std::vector<float>(w, -1.0f));
        visited.assign(h, std::vector<bool>(w, false));
    }

    // Add a fire source and bake its heat into the temperature field
    void add_fire(float cx, float cy, float spread = 20.0f, float peak = FIRE_PEAK) {
        fires.push_back({cx, cy, spread, peak});
        bake_temps();
    }


    bool in_bounds(int x, int y) const {
        return x >= 0 && x < w && y >= 0 && y < h;
    }

    float get_temp(int x, int y) const {
        if (!in_bounds(x, y)) return AMBIENT_TEMP;
        return temp_field[y][x];
    }

    // Drone reads temperature at its position (marks visited)
    float sense_temp(int x, int y) {
        if (!in_bounds(x, y)) return AMBIENT_TEMP;
        visited[y][x] = true;
        visited_temps[y][x] = temp_field[y][x];
        return temp_field[y][x];
    }

private:
    void bake_temps() {
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                float t = AMBIENT_TEMP;
                for (auto& f : fires) {
                    float dx = x - f.cx;
                    float dy = y - f.cy;
                    float contrib = (f.peak - AMBIENT_TEMP) * 
                                    std::exp(-(dx*dx + dy*dy) / f.spread);
                    t = std::max(t, AMBIENT_TEMP + contrib);
                }
                temp_field[y][x] = std::min(t, FIRE_PEAK);
            }
        }
    }
};


// ═══════════════════════════════════════════════════════════════════
//  DRONE — position, state, movement, and sensing
// ═══════════════════════════════════════════════════════════════════

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
    float       orbit_dir = 1.0f;   // 1.0 = one way, -1.0 = other
    std::string last_action = "initialized";
    char facing = 'S';  // Initial direction (North)
    bool fire_on_right; 

    // Lawnmower waypoints
    std::vector<Vec2> search_waypoints;
    size_t waypoint_idx = 0;

    // Log of positions for trail rendering
    std::vector<Vec2> trail;

    Drone(int x, int y) : pos({x, y}), home({x, y}), last_pos({x, y}) {}

    // ── Movement primitives ────────────────────────────────────────
    bool move(Vec2 direction, World& world) {
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

    bool move_north(World& w) { return move({0, -1}, w); }
    bool move_south(World& w) { return move({0,  1}, w); }
    bool move_east (World& w) { return move({ 1, 0}, w); }  
    bool move_west (World& w) { return move({-1, 0}, w); }

    // Relative movements based on current facing direction
    //move R, L, or forward based on current facing direction
    bool moveRight(World& w){
        if(facing == 'N') return move_east(w);
        else if(facing == 'E') return move_south(w);
        else if(facing == 'S') return move_west(w);
        else if(facing == 'W') return move_north(w);
        return false;
    }

    bool moveLeft(World& w){
        if(facing == 'N') return move_west(w);
        else if(facing == 'E') return move_north(w);
        else if(facing == 'S') return move_east(w);
        else if(facing == 'W') return move_south(w);
        return false;
    }

    bool moveForward(World& w){
        if(facing == 'N') return move_north(w);
        else if(facing == 'E') return move_east(w);
        else if(facing == 'S') return move_south(w);
        else if(facing == 'W') return move_west(w);
        return false;
    }

    //sets facing direction
    bool setFacing(char dir){
        if(dir == 'N' || dir == 'E' || dir == 'S' || dir == 'W'){
            facing = dir;
            return true;
        }
        return false;
    }


    // Move toward a target position (one step)
    bool move_toward(Vec2 target, World& world) {
        int dx = target.x - pos.x;
        int dy = target.y - pos.y;

        // Prefer the axis with larger delta
        if (std::abs(dx) >= std::abs(dy)) {
            if (dx != 0) return move({(dx > 0 ? 1 : -1), 0}, world);
            if (dy != 0) return move({0, (dy > 0 ? 1 : -1)}, world);
        } else {
            if (dy != 0) return move({0, (dy > 0 ? 1 : -1)}, world);
            if (dx != 0) return move({(dx > 0 ? 1 : -1), 0}, world);
        }
        return false;
    }

    
    
    bool at_target(Vec2 target) const {
        return pos == target;
    }

    // ── Sensing ───────────────────────────────────────────────────
    float sense(World& world) {
        last_temp    = current_temp;
        current_temp = world.sense_temp(pos.x, pos.y);
        return current_temp;
    }

    // ── Lawnmower pattern builder ─────────────────────────────────
    void build_lawnmower(int x0, int y0, int x1, int y1, int lane_spacing = 2) {
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
        // Return home at end
        search_waypoints.push_back({home.x, home.y});
    }

    Vec2 current_waypoint() const {
        if (waypoint_idx >= search_waypoints.size())
            return search_waypoints.back();
        return search_waypoints[waypoint_idx];
    }

    void advance_waypoint() {
        if (waypoint_idx < search_waypoints.size() - 1)
            waypoint_idx++;
    }
};


// ═══════════════════════════════════════════════════════════════════
//  RENDERER — draws the grid to the terminal
// ═══════════════════════════════════════════════════════════════════

class Renderer {
public:
    static void clear_screen() {
        std::cout << "\033[2J\033[H";
    }

    static std::string temp_color(float temp) {
        if (temp >= HIGH_THRESH)  return RED;
        if (temp >= SEARCH_THRESH) return YELLOW;
        if (temp >= 35.0f)         return "\033[33m";  // orange-ish
        return DIM;
    }

    static std::string temp_char(float temp) {
        if (temp >= HIGH_THRESH)   return "█";
        if (temp >= SEARCH_THRESH) return "▓";
        if (temp >= 35.0f)         return "░";
        return " ";
    }

    static void draw(const World& world, const Drone& drone) {
        clear_screen();

        // ── Title bar ─────────────────────────────────────────────
        std::cout << BOLD << CYAN;
        std::cout << "╔══════════════════════════════════════════════════════╗\n";
        std::cout << "║         SWARM Search Algorithm Simulator             ║\n";
        std::cout << "╚══════════════════════════════════════════════════════╝\n";
        std::cout << RESET;

        // ── Grid top border ───────────────────────────────────────
        std::cout << "  ┌";
        for (int x = 0; x < world.w; x++) std::cout << "─";
        std::cout << "┐\n";

        // ── Grid rows ─────────────────────────────────────────────
        for (int y = 0; y < world.h; y++) {
            std::cout << std::setw(2) << y << "│";
            for (int x = 0; x < world.w; x++) {
                Vec2 cell = {x, y};

                // Drone position
                // if (cell == drone.pos) {
                //     std::cout << BOLD << CYAN << "D" << RESET;
                //     continue;
                // }

                //Drone facing direction small arrow that represents drone and direction its facing
                if (cell == drone.pos) {
                    switch(drone.facing) {
                        case 'N': std::cout << BOLD << CYAN << "^" << RESET; break;
                        case 'E': std::cout << BOLD << CYAN << ">" << RESET; break;
                        case 'S': std::cout << BOLD << CYAN << "v" << RESET; break;
                        case 'W': std::cout << BOLD << CYAN << "<" << RESET; break;
                        default:  std::cout << BOLD << CYAN << " " << RESET; break;
                    }
                    continue;
                }
                

                // Home position
                if (cell == drone.home && cell != drone.pos) {
                    std::cout << BOLD << GREEN << "H" << RESET;
                    continue;
                }

                // Perimeter start marker
                if (drone.perimeter_start_set && cell == drone.perimeter_start) {
                    std::cout << BOLD << MAGENTA << "S" << RESET;
                    continue;
                }

                // Trail
                bool on_trail = false;
                for (auto& t : drone.trail) {
                    if (t == cell) { on_trail = true; break; }
                }

                // Visited cell — show what drone measured
                if (world.visited[y][x]) {
                    float t = world.visited_temps[y][x];
                    if (t >= HIGH_THRESH) {
                        std::cout << RED << "█" << RESET;
                    } else if (t >= SEARCH_THRESH) {
                        std::cout << YELLOW << "▓" << RESET;
                    } else if (t >= 35.0f) {
                        std::cout << "\033[33m" << "░" << RESET;
                    } else {
                        std::cout << DIM << (on_trail ? "·" : " ") << RESET;
                    }
                    continue;
                }

                // Unvisited — show dim trail or blank
                if (on_trail) {
                    std::cout << DIM << "·" << RESET;
                } else {
                    std::cout << " ";
                }
            }
            std::cout << "│\n";
        }

        // ── Grid bottom border ────────────────────────────────────
        std::cout << "  └";
        for (int x = 0; x < world.w; x++) std::cout << "─";
        std::cout << "┘\n";

        // ── X axis labels ─────────────────────────────────────────
        std::cout << "   ";
        for (int x = 0; x < world.w; x++) {
            if (x % 5 == 0) std::cout << "|";
            else std::cout << " ";
        }
        std::cout << "\n   ";
        for (int x = 0; x < world.w; x++) {
            if (x % 5 == 0) std::cout << x % 10;
            else std::cout << " ";
        }
        std::cout << "\n\n";

        // ── Status panel ──────────────────────────────────────────
        auto state_color = [](DroneState s) -> std::string {
            switch(s) {
                case DroneState::SEARCH:      return GREEN;
                case DroneState::PERIMETER:   return YELLOW;
                case DroneState::RETURN_HOME: return CYAN;
                case DroneState::DONE:        return DIM;
            }
            return WHITE;
        };

        std::cout << BOLD << "  STATUS\n" << RESET;
        std::cout << "  State    : " 
                  << state_color(drone.state) << BOLD 
                  << state_name(drone.state) << RESET << "\n";
        std::cout << "  Position : (" << drone.pos.x << ", " << drone.pos.y << ")\n";
        std::cout << "  Facing   : " << drone.facing << "\n";
        std::cout << "  Temp     : " << temp_color(drone.current_temp) << BOLD
                  << std::fixed << std::setprecision(1) 
                  << drone.current_temp << "°C" << RESET << "\n";
        std::cout << "  Steps    : " << drone.steps << "\n";
        std::cout << "  Last Action   : " << DIM << drone.last_action << RESET << "\n";
        

        if (drone.state == DroneState::SEARCH && !drone.search_waypoints.empty()) {
            auto wp = drone.current_waypoint();
            std::cout << "  Waypoint : (" << wp.x << ", " << wp.y << ") "
                      << "[" << drone.waypoint_idx << "/" 
                      << drone.search_waypoints.size()-1 << "]\n";
        }

        if (drone.state == DroneState::PERIMETER) {
            std::cout << "  Orbit    : " << (drone.orbit_dir > 0 ? "CW" : "CCW") << "\n";
            std::cout << "  P.Steps  : " << drone.perimeter_steps << "\n";
        }

        // ── Legend ────────────────────────────────────────────────
        std::cout << "\n  LEGEND\n";
        std::cout << "  " << BOLD << CYAN   << "D" << RESET << " Drone      ";
        std::cout <<        BOLD << GREEN   << "H" << RESET << " Home       ";
        std::cout <<        BOLD << MAGENTA << "S" << RESET << " Perim.start\n";
        std::cout << "  " << RED    << "█" << RESET << " Fire(>70°) ";
        std::cout <<        YELLOW  << "▓" << RESET << " Boundary   ";
        std::cout <<        "\033[33m" << "░" << RESET << " Warm\n";
        std::cout << "  " << DIM << "·" << RESET << " Trail      ";
        std::cout << "    (space) Unvisited\n";

        std::cout << std::flush;
    }
};


// ═══════════════════════════════════════════════════════════════════
//  ALGORITHMS — plug your search/tracing logic here
// ═══════════════════════════════════════════════════════════════════

/**
 * ALGORITHM: Lawnmower Search
 * Follows pre-built waypoints row by row.
 * Transitions to PERIMETER when temp >= SEARCH_THRESH.
 * Returns: true if a step was taken
 */
bool algo_search(Drone& drone, World& world) {
    if (drone.search_waypoints.empty()) return false;

    // Check temperature — enter perimeter mode if hot enough
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
            // Completed all waypoints
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

/**
 * ALGORITHM: Perimeter Tracing (Isotherm Following)
 * Estimates temperature gradient from movement and last reading.
 * Moves laterally along the fire boundary, correcting inward/outward
 * based on temperature feedback.
 *
 * This is where YOU can plug in alternative algorithms.
 */


 //this dont even work :(
bool algo_perimeter(Drone& drone, World& world) {
    // Set starting position first time we enter this state
    if (!drone.perimeter_start_set) {
        drone.perimeter_start = drone.pos;
        drone.perimeter_start_set = true;
        drone.perimeter_steps = 0;
        drone.last_action = "perimeter start recorded at (" +
                            std::to_string(drone.pos.x) + "," +
                            std::to_string(drone.pos.y) + ")";
        return true;
    }

    drone.perimeter_steps++;

    // Check if we've come full circle (min 20 steps before checking)
    if (drone.perimeter_steps > 20) {
        //if drone is within a radius of 1 cell from perimeter start, consider it a full loop (accounts for small drift)
        if (std::abs(drone.pos.x - drone.perimeter_start.x) <= 1 && std::abs(drone.pos.y - drone.perimeter_start.y) <= 1) {
            drone.state = DroneState::RETURN_HOME;
            drone.last_action = "perimeter complete → RETURN HOME";
            return true;
        }
    }

    // ── Gradient estimation ────────────────────────────────────────
    float grad_x = (float)(drone.pos.x - drone.last_pos.x);
    float grad_y = (float)(drone.pos.y - drone.last_pos.y);
    float mag = std::sqrt(grad_x*grad_x + grad_y*grad_y);

    if (mag < 0.5f) {
        // Haven't moved yet — pick an initial direction
        grad_x = 1.0f; grad_y = 0.0f; mag = 1.0f;
    }

    grad_x /= mag;
    grad_y /= mag;

    // Flip gradient if temperature decreased as we moved
    // (means we moved away from heat, so flip to point toward it)
    if (drone.current_temp < drone.last_temp - 0.1f) {
        grad_x = -grad_x;
        grad_y = -grad_y;
    }

    // ── Lateral direction (perpendicular to gradient) ─────────────
    float lat_x = -grad_y * drone.orbit_dir;
    float lat_y =  grad_x * drone.orbit_dir;

    // ── Radial correction based on temperature ────────────────────
    float correction = 0.0f;
    std::string correction_reason = "on boundary";

    if (drone.current_temp > HIGH_THRESH) {
        correction = -1.0f;   // too hot → move outward (away from fire)
        correction_reason = "too hot, backing off";
    } else if (drone.current_temp < LOW_THRESH) {
        correction = 1.0f;    // too cold → move inward (toward fire)
        correction_reason = "too cold, moving in";
    }

    // ── Compute next step direction ───────────────────────────────
    // Combine lateral movement with radial correction
    float next_fx = lat_x + grad_x * correction;
    float next_fy = lat_y + grad_y * correction;

    // Convert to discrete grid step (pick dominant axis)
    Vec2 step = {0, 0};
    if (std::abs(next_fx) >= std::abs(next_fy)) {
        step.x = (next_fx >= 0) ? 1 : -1;
    } else {
        step.y = (next_fy >= 0) ? 1 : -1;
    }

    bool moved = drone.move(step, world);
    if (!moved) {
        // Hit boundary — flip orbit direction
        drone.orbit_dir *= -1.0f;
        drone.last_action = "boundary hit, flipping orbit";
        return true;
    }

    drone.last_action = "perimeter trace: " + correction_reason +
                        " (" + std::to_string(drone.current_temp).substr(0,4) + "°C)";
    return true;
}


/**
 * ALGORITHM: Perimeter Tracing 
 * Uses simpler algorith that reacts simply to temperature thresholds to decide when to move inward/outward,
 * without trying to estimate the gradient direction.
 *
 * This is where YOU can plug in alternative algorithms.
 */
 
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
        //if drone is within a radius of 1 cell from perimeter start, consider it a full loop (accounts for small drift)
        if (std::abs(drone.pos.x - drone.perimeter_start.x) <= 1 && std::abs(drone.pos.y - drone.perimeter_start.y) <= 1) {
            drone.state = DroneState::RETURN_HOME;
            drone.last_action = "perimeter complete → RETURN HOME";
            return true;
        }
    }

    std::string correction_reason = "on boundary";

    //temperature delta 
    float temp_delta = std::abs(drone.current_temp - drone.last_temp);
    char previous_facing = drone.facing;

    // Priority order:
    // 1. If on boundary (LOW <= temp <= HIGH) → always move forward
    // 2. If too hot → turn away ONE step, then resume forward
    // 3. If too cold → turn in ONE step, then resume forward
    // Never let correction dominate over forward progress

    //when HOT
    if (drone.current_temp > HIGH_THRESH) {

        //if small oscillation occurs, correct, then contunue in same direction as before correction
        //dont let small correction dominate over forward progress, just use it to nudge in the right direction when we detect we're drifting too far off the boundary

        if(temp_delta < 5.0f){ // if temp just spiked suddenly, likely a small oscillation, so correct but dont flip direction
             if (drone.fire_on_right)                drone.moveLeft(world);
             else                                    drone.moveRight(world);
             drone.setFacing(previous_facing); // reset facing to what it was before correction, so we continue in same direction after correction
             correction_reason = "temp spike detected, small correction";
        }

        if(temp_delta >= 5.0f){ // if temp increased significantly, we're potentially hitting a corner or drifting far off boundary, so flip orbit direction to try to get back to boundary faster
             if (drone.fire_on_right)                drone.moveLeft(world);
             else                                    drone.moveRight(world);
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


// ═══════════════════════════════════════════════════════════════════
//  SIMULATION RUNNER
// ═══════════════════════════════════════════════════════════════════

class Simulation {
public:
    World world;
    Drone drone;
    int   max_steps;
    bool  running = true;

    Simulation(int max_steps = 2000)
        : world(GRID_W, GRID_H),
          drone(0, 0),
          max_steps(max_steps)
    {}

    void setup() {
        // ── Add fire sources ──────────────────────────────────────
        //Single fire blob in the middle of the search area
        world.add_fire(8.0f, 8.0f, 18.0f);

        world.add_fire(9.0f, 6.0f, 10.0f);

        // Uncomment to add a second fire source for testing irregular shapes:
        //world.add_fire(10.0f, 14.0f, 12.0f, 90.0f);

        //rectangular fire for testing perimeter following:
        world.add_fire(10.0f, 10.0f, 30.0f, 90.0f);

        world.add_fire(10.0f, 7.0f, 10.0f, 90.0f);
        

        // ── Build lawnmower pattern ───────────────────────────────
        // Search the area x: 0→24, y: 0→18, lanes every 2 cols
        drone.build_lawnmower(0, 0, 24, 18, 2);

        // ── Initial sense ─────────────────────────────────────────
        drone.sense(world);

        Renderer::draw(world, drone);
    }

    // Run one step of the simulation
    bool step() {
        if (drone.state == DroneState::DONE || drone.steps >= max_steps) {
            running = false;
            return false;
        }

        // Sense current temperature
        drone.sense(world);

        // Run the appropriate algorithm for current state
        switch (drone.state) {
            case DroneState::SEARCH:
                algo_search(drone, world);
                break;
            case DroneState::PERIMETER:
                boundaryTrace(drone, world);
                break;
            case DroneState::RETURN_HOME:
                algo_return_home(drone, world);
                break;
            case DroneState::DONE:
                running = false;
                return false;
        }

        // Sense again after moving
        drone.sense(world);

        return true;
    }

    // Run automatically with delay between steps
    void run_auto(int delay_ms = STEP_DELAY_MS) {
        setup();
        while (running) {
            bool ok = step();
            Renderer::draw(world, drone);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
            if (!ok) break;
        }
        print_summary();
    }

    // Run one step at a time (press Enter to advance)
    void run_manual() {
        setup();
        std::cout << "\nPress ENTER to step, 'q' + ENTER to quit, 'a' + ENTER for auto...\n";
        std::string input;
        while (running) {
            std::getline(std::cin, input);
            if (input == "q" || input == "Q") break;
            if (input == "a" || input == "A") {
                // Switch to auto mode
                while (running) {
                    bool ok = step();
                    Renderer::draw(world, drone);
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(STEP_DELAY_MS));
                    if (!ok) break;
                }
                break;
            }

            //feature not working yet but idea is
            //to go back a step press b + enter
            if(input == "b" || input == "B"){
                //go back one step
                if(drone.steps > 0){
                    drone.steps -= 2; // -1 to go back, -1 more to offset the upcoming +1 in step()
                    drone.pos = drone.last_pos; // move back to last position
                    drone.sense(world); // update temp readings for new position
                }
                continue;
            }

            bool ok = step();
            Renderer::draw(world, drone);
            if (!ok) break;
        }
        print_summary();
    }

    void print_summary() {
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
};


// ═══════════════════════════════════════════════════════════════════
//  MAIN — choose your run mode here
// ═══════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    std::cout << BOLD << "\nSWARM Search Algorithm Simulator\n" << RESET;
    std::cout << "Compile: g++ -std=c++17 -o swarm_sim swarm_sim.cpp\n\n";
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