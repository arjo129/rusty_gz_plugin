#include "RustySystem.hh"
#include <ignition/plugin/Register.hh>

using namespace rusty;

extern "C" void hello_rust();

RustySystem::RustySystem()
{
}

RustySystem::~RustySystem()
{
}

void RustySystem::Configure(const ignition::gazebo::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            ignition::gazebo::EntityComponentManager &_ecm,
                            ignition::gazebo::EventManager &_eventMgr)
{
  hello_rust();
}

void RustySystem::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                             const ignition::gazebo::EntityComponentManager &_ecm)
{
}

IGNITION_ADD_PLUGIN(rusty::RustySystem,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPostUpdate)