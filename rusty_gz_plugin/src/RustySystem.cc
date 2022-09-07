#include "rust_interface.h"

#include "RustySystem.hh"
#include <gz/plugin/Register.hh>

#include <gz/msgs/entity_factory.pb.h>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Static.hh>

#include <gz/transport/Node.hh>

using namespace rusty;

std::string world_name;

gz::transport::Node node;

//std::unordered_map<uint64_t, > ;

void createEntityFromStr(const uint64_t id, const std::string& modelStr)
{
//! [call service create sphere]
  bool result;
  gz::msgs::EntityFactory req;
  gz::msgs::Boolean res;
  req.set_sdf(modelStr);
  req.set_name("actor"+std::to_string(id));

  bool executed = node.Request("/world/crowd_world/create",
            req, 1000, res, result);
  if (executed)
  {
    if (result)
      igndbg << "Entity was created : [" << res.data() << "]" << std::endl;
    else
    {
      ignerr << "Service call failed" << std::endl;
      return;
    }
  }
  else
    ignerr << "Service call timed out" << std::endl;
//! [call service create sphere]
}


extern "C" void spawn_agent(uint64_t id, double x, double y)
{
    auto sphereStr = R"(
    <?xml version="1.0" ?>
    <sdf version='1.7'>
        <include>
            <uri>
            https://fuel.gazebosim.org/1.0/OpenRobotics/models/Male visitor/1
            </uri>
            <pose>)" + std::to_string(x) + " " + std::to_string(y) + " " +
            R"(0.5 0 0 0</pose>
        </include>
    </sdf>)";
    createEntityFromStr(id, sphereStr);
}

RustySystem::RustySystem()
{

}

RustySystem::~RustySystem()
{
}

void RustySystem::Configure(const gz::sim::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager &_eventMgr)
{
    create_crowd_agents(0, 0, 10, 10, 1, 0.1, 0.1);
    debug_config();
    register_spawn_cb(spawn_agent);
}

void RustySystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                             const gz::sim::EntityComponentManager &_ecm)
{
    static bool launch = false;
    if (!launch)
    {
        run();
        launch = true;
    }
}

GZ_ADD_PLUGIN(rusty::RustySystem,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPostUpdate)