use rmf_crowdsim::local_planners::no_local_plan::NoLocalPlan;
use rmf_crowdsim::source_sink::source_sink::{MonotonicCrowd, SourceSink};
use rmf_crowdsim::spatial_index::location_hash_2d::LocationHash2D;
use rmf_crowdsim::spatial_index::spatial_index::SpatialIndex;
use rmf_crowdsim::*;
use std::sync::{Arc, Mutex};

#[derive(Debug, Copy, Clone)]
pub struct CrowdSimConfig{
    source_x: f64,
    source_y: f64,
    sink_x: f64,
    sink_y: f64,
    radius: f64,
    rate: f64,
    lambda: f64
}
static mut SIM_CONFIG: Vec<CrowdSimConfig> = vec!();

pub struct SpawnCBIntegration {
    pub call_back: extern fn(u64, f64, f64) -> ()
}
static mut GZ_SPAWN_CB: Option<SpawnCBIntegration> = None;

struct SimulationModel {
    crowd_sim: Simulation<LocationHash2D>
}
static mut SIM_MODEL: Option<SimulationModel> = None;

#[no_mangle]
pub extern "C" fn register_spawn_cb(call_back: extern fn(u64, f64, f64) -> ()) {
    unsafe { GZ_SPAWN_CB = Some(SpawnCBIntegration {
        call_back: call_back});};
}

#[no_mangle]
pub extern "C" fn create_crowd_agents(
    source_x: f64,
    source_y: f64,
    sink_x: f64,
    sink_y: f64,
    radius: f64,
    rate: f64,
    lambda: f64) {

    let cfg = CrowdSimConfig{
        source_x: source_x,
        source_y: source_y,
        sink_x: sink_x,
        sink_y: sink_y,
        radius: radius,
        rate: rate,
        lambda: lambda
    };

    unsafe { SIM_CONFIG.push(cfg); }
}

#[no_mangle]
pub extern "C" fn debug_config()
{
    for x in unsafe{ &SIM_CONFIG } {
       println!("{:?}", x);
    }
}

#[no_mangle]
pub extern "C" fn run()
{
    match unsafe {&GZ_SPAWN_CB}
    {
        Some(res) => {
            ((*res).call_back)(0u64, 0.5f64, 0f64);
        },
        None => {
            panic!("Please register a spawn callback")
        }
    }
}