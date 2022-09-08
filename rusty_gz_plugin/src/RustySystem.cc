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

using namespace gz;
using namespace gz::sim;

std::string world_name;

gz::transport::Node node;

std::unordered_map<uint64_t, uint64_t> correspondance;

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
  register_spawn_cb(spawn_agent);
}

void RustySystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                            gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
  {
    return;
  }
  run(std::chrono::duration<float>(_info.dt).count());
  _ecm.Each<components::Actor, components::Name>(
    [&](const Entity &_entity, const components::Actor *,
    const components::Name *_name)->bool
    {
      if (_name->Data().size() > 5 && _name->Data().substr(0,5) == "actor")
      {
        auto position = query_position(atoi(_name->Data().substr(5, _name->Data().size()).c_str()));
        _ecm.Component<components::Pose>(_entity)->Data() = gz::math::Pose3d(position.x, position.y, 0.5, 0, 0, 0);
        _ecm.SetChanged(_entity, components::Pose::typeId,
          ComponentState::OneTimeChange);

      }
      return true;
    });
}

GZ_ADD_PLUGIN(rusty::RustySystem,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)