#![feature(core_ffi_c)]

use rmf_crowdsim::local_planners::no_local_plan::NoLocalPlan;
use rmf_crowdsim::source_sink::source_sink::{PoissonCrowd, SourceSink};
use rmf_crowdsim::spatial_index::location_hash_2d::LocationHash2D;
use rmf_crowdsim::spatial_index::spatial_index::SpatialIndex;
use rmf_crowdsim::*;
use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use std::ffi::{c_int, c_float};

////////////////////////////////////////////////////////////////////////////////
struct StubHighLevelPlan {
    default_vel: Vec2f,
}

impl StubHighLevelPlan {
    fn new(default_vel: Vec2f) -> Self {
        StubHighLevelPlan {
            default_vel: default_vel,
        }
    }
}

impl HighLevelPlanner for StubHighLevelPlan {
    fn get_desired_velocity(
        &mut self,
        _agent: &Agent,
        _time: std::time::Duration,
    ) -> Option<Vec2f> {
        Some(self.default_vel)
    }

    /// Set the target position for a given agent
    fn set_target(&mut self, _agent: &Agent, _point: Point, _tolerance: Vec2f) {
        // For now do nothing
    }
    /// Remove an agent
    fn remove_agent_id(&mut self, _agent: AgentId) {
        // Do nothing
    }
}
////////////////////////////////////////////////////////////////////////////////
#[derive(Debug)]
struct CrowdEventListener {
    correspondence: HashMap<u64, u64>
}

impl CrowdEventListener {
    pub fn new() -> Self {
        Self{
            correspondence: HashMap::new()
        }
    }
}

impl EventListener for CrowdEventListener {
    fn agent_spawned(&mut self, position: Vec2f, agent: AgentId) {
        match unsafe {&GZ_SPAWN_CB}
        {
            Some(res) => {
                ((*res).call_back)(agent as u64, position.x, position.y);
            },
            None => {
                panic!("Please register a spawn callback");
            }
        }
    }

    /// Called each time an agent is destroyed
    fn agent_destroyed(&mut self, agent: AgentId) {
        println!("Removed {}", agent);
    }
}

////////////////////////////////////////////////////////////////////////////////
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
    crowd_sim: Simulation<LocationHash2D>,
    //high_level_planner: Arc<StubHighLevelPlan>,
    //local_planner: Arc<NoLocalPlan>,
}
static mut SIM_MODEL: Option<Arc<Mutex<SimulationModel>>> = None;

#[no_mangle]
pub extern "C" fn register_spawn_cb(call_back: extern fn(u64, f64, f64) -> ()) {
    unsafe { GZ_SPAWN_CB = Some(SpawnCBIntegration {
        call_back: call_back});};

    let stub_spatial = spatial_index::location_hash_2d::LocationHash2D::new(
        1000f64,
        1000f64,
        20f64,
        Point::new(-500f64, -500f64),
    );

    let velocity = Vec2f::new(1.0, 0.0);

    let high_level_planner = Arc::new(Mutex::new(StubHighLevelPlan::new(velocity)));
    let local_planner = Arc::new(Mutex::new(NoLocalPlan {}));

    let mut crowd_simulation = Simulation::new(stub_spatial);

    let crowd_generator = Arc::new(PoissonCrowd::new(1f64));

    let source_sink = Arc::new(SourceSink {
        source: Vec2f::new(0f64, 0f64),
        sink: Vec2f::new(20f64, 0f64),
        radius_sink: 1f64,
        crowd_generator: crowd_generator,
        high_level_planner: high_level_planner,
        local_planner: local_planner,
        agent_eyesight_range: 5f64,
    });

    let event_listener = Arc::new(Mutex::new(CrowdEventListener::new()));

    crowd_simulation.add_event_listener(event_listener.clone());
    crowd_simulation.add_source_sink(source_sink);
    let model = Arc::new(Mutex::new(SimulationModel {
        crowd_sim: crowd_simulation,
    }));

    unsafe {
        SIM_MODEL = Some(model.clone());
    }
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
pub extern "C" fn run(dt: f32)
{
    if dt <= 0.0 {
        return;
    }
    let step_size = std::time::Duration::new(dt.floor() as u64, ((dt - dt.floor())*1e9) as u32);
    // TODO(arjo): Configure step time.
    match unsafe{&SIM_MODEL} {
    Some(sim) => {
        sim.lock().unwrap().crowd_sim.step(step_size);
    }
    None => {
        println!("Error: Sim model was not initiallized");
    }
    }
}

#[repr(C)]
pub struct Position
{
    x: c_float,
    y: c_float,
    visible: c_int
}

#[no_mangle]
pub extern "C" fn query_position(agent_id: u64) -> Position {
    match unsafe{&SIM_MODEL} {
        Some(sim) => {
            //TODO(arjo): Remove clone
            let agent_id = agent_id as usize;
            let sim = sim.lock().unwrap();
            let agent = sim.crowd_sim.agents.get(&agent_id);
            if let Some(agent) = agent {
                return Position{
                    x: agent.position.x as f32,
                    y: agent.position.y as f32,
                    visible: 1
                };
            }
            return Position{x: 0.0, y: 0.0, visible: -1};
        }
        None => {
            println!("Error: Sim model was not initialized");
            return Position{x: 0.0, y: 0.0, visible: -1};
        }
    }
}