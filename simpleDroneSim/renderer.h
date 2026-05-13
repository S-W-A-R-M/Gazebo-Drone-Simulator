#ifndef RENDERER_H
#define RENDERER_H

#include <iostream>
#include <string>
#include <iomanip>
#include "config.h"
#include "world.h"
#include "drone.h"

class Renderer {
public:
    static void clear_screen();
    static std::string temp_color(float temp); 
    static std::string temp_char(float temp);
    static void draw(const World& world, const Drone& drone);
};

#endif // RENDERER_H