#![feature(core_ffi_c)]

use rmf_crowdsim::local_planners::no_local_plan::NoLocalPlan;
use rmf_crowdsim::source_sink::source_sink::{PoissonCrowd, SourceSink};
use rmf_crowdsim::spatial_index::location_hash_2d::LocationHash2D;
use rmf_crowdsim::spatial_index::spatial_index::SpatialIndex;
use rmf_crowdsim::rmf::RMFPlanner;
use rmf_crowdsim::*;
use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use std::ffi::{c_int, c_float, c_char, CStr};
use std::fs;

////////////////////////////////////////////////////////////////////////////////
/// Globals go here.
#[derive(Debug, Clone)]
pub struct CrowdSimConfig{
    source_x: f64,
    source_y: f64,
    waypoints: Vec<Vec2f>,
    radius: f64,
    rate: f64,
    lambda: f64
}

/// List of SourceSink components
static mut SIM_CONFIG: Vec<CrowdSimConfig> = vec!();

/// Spawn callback
pub struct SpawnCBIntegration {
    pub call_back: extern fn(u64, f64, f64) -> ()
}
static mut GZ_SPAWN_CB: Option<SpawnCBIntegration> = None;

struct SimulationModel {
    crowd_sim: Simulation<LocationHash2D>,
    //high_level_planner: Arc<StubHighLevelPlan>,
    //local_planner: Arc<NoLocalPlan>,
}
static mut SIM_MODEL: Option<Arc<Mutex<SimulationModel>>> = None;

static mut FILE_PATH: &str = "";

#[repr(C)]
pub struct Position
{
    x: c_float,
    y: c_float,
    visible: c_int
}

#[repr(C)]
pub struct SimulationBinding
{
    crowd_sim: Simulation<LocationHash2D>,
    rmf_file_path: String,
    spawn_callback: SpawnCBIntegration,
    rmf_planner: Arc<Mutex<RMFPlanner>>,
    local_planner: Arc<Mutex<NoLocalPlan>> //TODO(arjo): Switch to Zanlungo
}

#[no_mangle]
pub extern "C" fn crowdsim_new(
    file_path: *const c_char,
    spawn_cb: extern fn (u64, f64, f64) -> ()
) -> *mut SimulationBinding
{
    // TODO(arjo): Calculate size based on rmf_planner
    let stub_spatial = spatial_index::location_hash_2d::LocationHash2D::new(
        1000f64,
        1000f64,
        20f64,
        Point::new(-500f64, -500f64),
    );

    let mut crowd_sim = Simulation::new(stub_spatial);

    let c_str: &CStr = unsafe { CStr::from_ptr(file_path) };

    let yaml_body = fs::read_to_string(c_str.to_str().unwrap()).unwrap();
    let rmf_planner = RMFPlanner::from_yaml(&yaml_body, 3.0, 0.2, 0.3);
    let high_level_planner = Arc::new(Mutex::new(rmf_planner));

    let event_listener = Arc::new(Mutex::new(
        CrowdEventListener::new(SpawnCBIntegration {
            call_back: spawn_cb})));
    crowd_sim.add_event_listener(event_listener.clone());

    Box::into_raw(Box::new(SimulationBinding
    {
        crowd_sim,
        rmf_file_path: c_str.to_str().unwrap().to_owned(),
        spawn_callback: SpawnCBIntegration { call_back: spawn_cb },
        rmf_planner: high_level_planner,
        local_planner: Arc::new(Mutex::new(NoLocalPlan{}))
    }))
}

#[no_mangle]
pub extern "C" fn crowdsim_free(ptr: *mut SimulationBinding)
{
    if ptr.is_null() {
        return;
    }
    unsafe {
        Box::from_raw(ptr);
    }
}

#[no_mangle]
pub extern "C" fn crowdsim_add_source_sink(
    ptr: *mut SimulationBinding,
    start: Position,
    waypoints: *mut Position,
    num_waypoints: u64,
    rate: f64)
{
    let mut sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    let positions = unsafe {
        std::slice::from_raw_parts(
            waypoints as *const Position,
            num_waypoints as usize
        )
    };

    let mut waypoints = vec!();

    for position in positions {
        waypoints.push(Vec2f::new(position.x.into(), position.y.into()));
        println!("Setting target {:?}, {:?}", position.x, position.y);
    }


    let crowd_generator = Arc::new(PoissonCrowd::new(rate));

    let source_sink = Arc::new(SourceSink {
        source: Vec2f::new(start.x.into(), start.y.into()),
        waypoints: waypoints,
        radius_sink: 1f64,
        crowd_generator: crowd_generator,
        high_level_planner: sim_binding.rmf_planner.clone(),
        local_planner: sim_binding.local_planner.clone(),
        agent_eyesight_range: 5f64,
        loop_forever: false
    });

    sim_binding.crowd_sim.add_source_sink(source_sink);
}


#[no_mangle]
pub extern "C" fn crowdsim_query_position(
    ptr: *mut SimulationBinding,
    agent_id: u64) -> Position
{
    let mut sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    //println!("Looking up agent {:?}", agent_id);

    let agent = sim_binding.crowd_sim.agents.get(&(agent_id as usize));
    if let Some(agent) = agent {
        return Position{
            x: agent.position.x as f32,
            y: agent.position.y as f32,
            visible: 1
        };
    }
    println!("Agent not found {:?}", agent_id);
    return Position{x: 0.0, y: 0.0, visible: -1};
}


#[no_mangle]
pub extern "C" fn crowdsim_run(
    ptr: *mut SimulationBinding,
    dt: f32)
{
    let mut sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    if dt <= 0.0 {
        println!("Got negative dt");
        return;
    }
    let step_size = std::time::Duration::new(
        dt.floor() as u64, ((dt - dt.floor())*1e9) as u32);
    sim_binding.crowd_sim.step(step_size);
}
////////////////////////////////////////////////////////////////////////////////
struct CrowdEventListener {
    correspondence: HashMap<u64, u64>,
    spawn_callback: SpawnCBIntegration
}

impl CrowdEventListener {
    pub fn new(spawn_callback: SpawnCBIntegration) -> Self {
        Self{
            correspondence: HashMap::new(),
            spawn_callback
        }
    }
}

impl EventListener for CrowdEventListener {
    fn agent_spawned(&mut self, position: Vec2f, agent: AgentId) {
        println!("Spawning Agent");
        (self.spawn_callback.call_back)(agent as u64, position.x, position.y);
    }

    /// Called each time an agent is destroyed
    fn agent_destroyed(&mut self, agent: AgentId) {
        println!("Removed {}", agent);
    }
}
