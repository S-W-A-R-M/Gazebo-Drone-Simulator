#include "world.h"
#include <cmath>
#include <algorithm>

World::World(int w, int h) : w(w), h(h) {
    temp_field.assign(h, std::vector<float>(w, AMBIENT_TEMP));
    visited_temps.assign(h, std::vector<float>(w, -1.0f));
    visited.assign(h, std::vector<bool>(w, false));
}

void World::add_fire(float cx, float cy, float spread, float peak) {
    fires.push_back({cx, cy, spread, peak});
    bake_temps();
}

bool World::in_bounds(int x, int y) const {
    return x >= 0 && x < w && y >= 0 && y < h;
}

float World::get_temp(int x, int y) const {
    if (!in_bounds(x, y)) return AMBIENT_TEMP;
    return temp_field[y][x];
}

float World::sense_temp(int x, int y) {
    if (!in_bounds(x, y)) return AMBIENT_TEMP;
    visited[y][x] = true;
    visited_temps[y][x] = temp_field[y][x];
    return temp_field[y][x];
}

void World::bake_temps() {
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