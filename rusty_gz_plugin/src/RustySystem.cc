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

#include <gz/common/StringUtils.hh>

using namespace rusty;

using namespace gz;
using namespace gz::sim;

std::string worldName;

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

  bool executed = node.Request("/world/"+ worldName +"/create",
            req, 10000, res, result);
  if (executed)
  {
    if (result)
      gzdbg << "Entity was created : [" << res.data() << "]" << std::endl;
    else
    {
      gzerr << "Service call failed" << std::endl;
      return;
    }
  }
  else
    gzerr << "Service call timed out" << std::endl;
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
            R"(1.7 0 0 0</pose>
        </include>
    </sdf>)";
    createEntityFromStr(id, sphereStr);
}

RustySystem::RustySystem()
{

}

RustySystem::~RustySystem()
{
  // TODO(arjo): Add a check if the crowdsim is actually inited
  crowdsim_free(this->crowdsim);
}

void RustySystem::Configure(const gz::sim::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager &_eventMgr)
{

  std::string path;
  if (_sdf->HasElement("path"))
  {
    path = _sdf->Get<std::string>("path");
  }
  else
  {
    gzerr << "Please specify a path using the <path> tag! "
      <<"As is standard practice this plugin will proceed to crash gazebo\n";
    return;
  }
  // Creates a new crowdsim instance
  this->crowdsim = crowdsim_new(
    path.c_str(),
    spawn_agent
  );

  // Source sinks:
  // We have hardcoded it for this demo, but should be trivial to parse SDF.
  Position start {-23, -1, 0};
  Position waypoints[1] = {
    Position{9, -3, 0}
  };

  crowdsim_add_source_sink(
    this->crowdsim,
    start,
    waypoints,
    1,
    1
  );

  //crowdsim_register_spawn_cb(spawn_agent);
  worldName = _ecm.Component<components::Name>(_entity)->Data();
}

void RustySystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                            gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
  {
    return;
  }
  crowdsim_run(
    this->crowdsim, std::chrono::duration<float>(_info.dt).count());
  _ecm.Each<components::Actor, components::Name>(
    [&](const Entity &_entity, const components::Actor *,
    const components::Name *_name)->bool
    {
      if (_name->Data().size() > 5 && _name->Data().substr(0,5) == "actor")
      {
        auto position = crowdsim_query_position(
          this->crowdsim,
          atoi(_name->Data().substr(5, _name->Data().size()).c_str()));
        if (position.visible < 0)
        {
          _ecm.RequestRemoveEntity(_entity, true);
          return true;
        }
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
