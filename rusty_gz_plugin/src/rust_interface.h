
#pragma once

#include <cstdint>

extern "C" void create_crowd_agents(
    double, double, double, double, double, double, double);

extern "C" void debug_config();

typedef void (*spawn_cb_t) (uint64_t id, double x, double y);

extern "C" void register_spawn_cb(spawn_cb_t cb);

extern "C"{
struct Position
{
    float x;
    float y;
    int visible;
};
}

extern "C" Position query_position(uint64_t id);

extern "C" void run(float timestep);
