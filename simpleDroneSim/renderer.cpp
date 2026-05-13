#include "renderer.h"

void Renderer::clear_screen() {
    std::cout << "\033[2J\033[H";
}

std::string Renderer::temp_color(float temp) {
    if (temp >= HIGH_THRESH)   return RED;
    if (temp >= SEARCH_THRESH) return YELLOW;
    if (temp >= 35.0f)         return "\033[33m";  
    return DIM;
}

std::string Renderer::temp_char(float temp) {
    if (temp >= HIGH_THRESH)   return "█";
    if (temp >= SEARCH_THRESH) return "▓";
    if (temp >= 35.0f)         return "░";
    return " ";
}

void Renderer::draw(const World& world, const Drone& drone) {
    clear_screen();

    std::cout << BOLD << CYAN;
    std::cout << "╔══════════════════════════════════════════════════════╗\n";
    std::cout << "║         SWARM Search Algorithm Simulator             ║\n";
    std::cout << "╚══════════════════════════════════════════════════════╝\n";
    std::cout << RESET;

    std::cout << "  ┌";
    for (int x = 0; x < world.w; x++) std::cout << "─";
    std::cout << "┐\n";

    for (int y = 0; y < world.h; y++) {
        std::cout << std::setw(2) << y << "│";
        for (int x = 0; x < world.w; x++) {
            Vec2 cell = {x, y};

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
            
            if (cell == drone.home && cell != drone.pos) {
                std::cout << BOLD << GREEN << "H" << RESET;
                continue;
            }

            if (drone.perimeter_start_set && cell == drone.perimeter_start) {
                std::cout << BOLD << MAGENTA << "S" << RESET;
                continue;
            }

            bool on_trail = false;
            for (auto& t : drone.trail) {
                if (t == cell) { on_trail = true; break; }
            }

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

            if (on_trail) {
                std::cout << DIM << "·" << RESET;
            } else {
                std::cout << " ";
            }
        }
        std::cout << "│\n";
    }

    std::cout << "  └";
    for (int x = 0; x < world.w; x++) std::cout << "─";
    std::cout << "┘\n";

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