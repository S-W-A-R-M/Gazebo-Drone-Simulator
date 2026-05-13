#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include "config.h"

class World {
public:
    int w, h;
    std::vector<FireSource> fires;
    std::vector<std::vector<float>> temp_field;     
    std::vector<std::vector<float>> visited_temps;  
    std::vector<std::vector<bool>>  visited;

    World(int w, int h);

    void add_fire(float cx, float cy, float spread = 20.0f, float peak = FIRE_PEAK);
    bool in_bounds(int x, int y) const;
    float get_temp(int x, int y) const;
    float sense_temp(int x, int y);

private:
    void bake_temps();
};

#endif // WORLD_H