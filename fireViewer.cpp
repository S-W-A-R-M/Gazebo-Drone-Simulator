#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <iomanip>
#include <algorithm>

// Configuration & ANSI Colors (Matched to your simulation)
constexpr int   GRID_W          = 30;
constexpr int   GRID_H          = 20;
constexpr float AMBIENT_TEMP    = 20.0f;
constexpr float FIRE_PEAK       = 100.0f;
constexpr float SEARCH_THRESH   = 50.0f;
constexpr float HIGH_THRESH     = 70.0f;

#define RESET       "\033[0m"
#define BOLD        "\033[1m"
#define RED         "\033[31m"
#define YELLOW      "\033[33m"
#define CYAN        "\033[36m"
#define MAGENTA     "\033[35m"
#define DIM         "\033[2m"

struct FireSource {
    float cx, cy, spread, peak;
};

class StaticWorld {
public:
    int w, h;
    std::vector<FireSource> fires;
    std::vector<std::vector<float>> temp_field;

    StaticWorld(int w, int h) : w(w), h(h) {
        temp_field.assign(h, std::vector<float>(w, AMBIENT_TEMP));
    }

    void add_fire(float cx, float cy, float spread = 20.0f, float peak = FIRE_PEAK) {
        fires.push_back({cx, cy, spread, peak});
    }

    void bake_temps() {
        // Reset field
        for(auto& row : temp_field) std::fill(row.begin(), row.end(), AMBIENT_TEMP);
        
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                float t = AMBIENT_TEMP;
                for (auto& f : fires) {
                    float dx = x - f.cx;
                    float dy = y - f.cy;
                    float contrib = (f.peak - AMBIENT_TEMP) * std::exp(-(dx*dx + dy*dy) / f.spread);
                    t = std::max(t, AMBIENT_TEMP + contrib);
                }
                temp_field[y][x] = std::min(t, FIRE_PEAK);
            }
        }
    }

    void clear_fires() {
        fires.clear();
    }
};

void draw_world(const StaticWorld& world, const std::string& title) {
    std::cout << "\033[2J\033[H"; // Clear screen
    std::cout << BOLD << CYAN;
    std::cout << "╔══════════════════════════════════════════════════════╗\n";
    std::cout << "║ FIRE VIEWER: " << std::setw(39) << std::left << title << " ║\n";
    std::cout << "╚══════════════════════════════════════════════════════╝\n";
    std::cout << RESET;

    std::cout << "  ┌";
    for (int x = 0; x < world.w; x++) std::cout << "─";
    std::cout << "┐\n";

    for (int y = 0; y < world.h; y++) {
        std::cout << std::setw(2) << y << "│";
        for (int x = 0; x < world.w; x++) {
            float t = world.temp_field[y][x];
            
            // Full visibility rendering
            if (t >= HIGH_THRESH) {
                std::cout << RED << "█" << RESET;
            } else if (t >= SEARCH_THRESH) {
                std::cout << YELLOW << "▓" << RESET;
            } else if (t >= 35.0f) {
                std::cout << "\033[33m" << "░" << RESET;
            } else {
                std::cout << DIM << " " << RESET;
            }
        }
        std::cout << "│\n";
    }

    std::cout << "  └";
    for (int x = 0; x < world.w; x++) std::cout << "─";
    std::cout << "┘\n";

    std::cout << "   ";
    for (int x = 0; x < world.w; x++) {
        if (x % 5 == 0) std::cout << "|"; else std::cout << " ";
    }
    std::cout << "\n   ";
    for (int x = 0; x < world.w; x++) {
        if (x % 5 == 0) std::cout << x % 10; else std::cout << " ";
    }
    std::cout << "\n\n";
    
    std::cout << "  " << RED << "█" << RESET << " Fire(>70°)   "
              << YELLOW << "▓" << RESET << " Boundary(>50°)   "
              << "\033[33m" << "░" << RESET << " Warm(>35°)\n\n";
              
    std::cout << "Press ENTER for next shape...";
    std::cin.ignore();
}

int main() {
    StaticWorld world(GRID_W, GRID_H);

    // 1. L-Shape
    world.add_fire(12.0f, 6.0f, 20.0f);
    world.add_fire(12.0f, 10.0f, 20.0f);
    world.add_fire(12.0f, 14.0f, 20.0f);
    world.add_fire(16.0f, 14.0f, 20.0f);
    world.add_fire(20.0f, 14.0f, 20.0f);
    world.bake_temps();
    draw_world(world, "The L-Shape (Sharp Corners)");

    // 2. Dumbbell
    world.clear_fires();
    world.add_fire(8.0f, 10.0f, 35.0f); 
    world.add_fire(22.0f, 10.0f, 35.0f);
    world.add_fire(15.0f, 10.0f, 15.0f);
    world.bake_temps();
    draw_world(world, "The Dumbbell (Narrow Corridor)");

    // 3. C-Shape
    world.clear_fires();
    world.add_fire(10.0f, 6.0f,  20.0f);
    world.add_fire(15.0f, 6.0f,  20.0f);
    world.add_fire(18.0f, 8.0f,  20.0f);
    world.add_fire(18.0f, 12.0f, 20.0f);
    world.add_fire(15.0f, 14.0f, 20.0f);
    world.add_fire(10.0f, 14.0f, 20.0f);
    world.bake_temps();
    draw_world(world, "The C-Shape (Concave Trap)");

    // 4. Irregular Front
    world.clear_fires();
    world.add_fire(8.0f,  12.0f, 18.0f);
    world.add_fire(11.0f, 10.0f, 22.0f);
    world.add_fire(14.0f, 13.0f, 15.0f);
    world.add_fire(17.0f, 9.0f,  25.0f);
    world.add_fire(21.0f, 11.0f, 20.0f);
    world.bake_temps();
    draw_world(world, "Irregular Bumpy Wall");

    std::cout << "\nDone previewing shapes.\n";
    return 0;
}