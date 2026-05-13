#ifndef CONFIG_H
#define CONFIG_H

#include <string>

// ═══════════════════════════════════════════════════════════════════
//  CONFIGURATION & ANSI COLORS
// ═══════════════════════════════════════════════════════════════════

constexpr int   GRID_W          = 30;       // grid width  (columns)
constexpr int   GRID_H          = 20;       // grid height (rows)
constexpr float AMBIENT_TEMP    = 20.0f;    // background temperature °C
constexpr float FIRE_PEAK       = 100.0f;   // temperature at fire center °C
constexpr float SEARCH_THRESH   = 50.0f;    // enter PERIMETER above this °C
constexpr float HIGH_THRESH     = 70.0f;    // too hot, back off
constexpr float LOW_THRESH      = 45.0f;    // too cold, move in
constexpr int   STEP_DELAY_MS   = 120;      // ms between auto-steps (speed)

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

// ═══════════════════════════════════════════════════════════════════
//  SHARED TYPES
// ═══════════════════════════════════════════════════════════════════

struct Vec2 {
    int x, y;
    bool operator==(const Vec2& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Vec2& o) const { return !(*this == o); }
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
};

struct FireSource {
    float cx, cy;   
    float spread;   
    float peak;     
};

enum class DroneState {
    SEARCH,
    PERIMETER,
    RETURN_HOME,
    DONE
};

inline std::string state_name(DroneState s) {
    switch(s) {
        case DroneState::SEARCH:      return "SEARCH";
        case DroneState::PERIMETER:   return "PERIMETER";
        case DroneState::RETURN_HOME: return "RETURN HOME";
        case DroneState::DONE:        return "DONE";
    }
    return "UNKNOWN";
}

#endif // CONFIG_H