
#pragma once

#include <cstdint>

extern "C" typedef struct SimulationBinding simulation_binding_t;

extern "C"{
struct Position
{
    float x;
    float y;
    int visible;
};
}

typedef void (*spawn_cb_t) (uint64_t id, double x, double y);

extern "C" simulation_binding_t * crowdsim_new(
    char* file_path, spawn_cb_t cb);

extern "C" void crowdsim_free(simulation_binding_t *);

extern "C" void crowdsim_add_source_sink(
    simulation_binding_t *crowdsim_instance,
    Position start,
    Position* waypoints,
    uint64_t num_waypoints,
    double rate
    );

extern "C" Position crowdsim_query_position(
    simulation_binding_t*, uint64_t id);

extern "C" void crowdsim_run(
    simulation_binding_t*,
    float timestep);
