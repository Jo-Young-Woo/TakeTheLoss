// The MAICON arena has no transparent textures.  Some Fortress Ogre 1 builds
// leave textured materials in a transparent pass without depth writes.  The
// floor then hides cubes farther from the camera than its own origin.
// Repair only this package's textured materials, on the rendering thread.
#include <memory>
#include <string>
#include "opaque_material_repair.hh"

#ifdef MAICON_GZ_HARMONIC
#include <gz/common/Console.hh>
#include <gz/common/Event.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/rendering/Events.hh>
namespace sim = gz::sim;
namespace render = gz::rendering;
namespace common = gz::common;
#else
#include <ignition/common/Console.hh>
#include <ignition/common/Event.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/Geometry.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/rendering/Events.hh>
namespace sim = ignition::gazebo;
namespace render = ignition::rendering;
namespace common = ignition::common;
#endif

namespace maicon
{
class OpaqueMaterialSystem final : public sim::System,
                                   public sim::ISystemConfigure
{
  public: ~OpaqueMaterialSystem() override { this->connection.reset(); }

  public: void Configure(const sim::Entity &, const std::shared_ptr<const sdf::Element> &settings,
                        sim::EntityComponentManager &, sim::EventManager &events) override
  {
    // Ogre 2 is available only as an explicit diagnostic; generated default
    // worlds never add this plugin to Ogre 2.
    if (settings && settings->HasElement("render_engine"))
    {
      const auto requested = settings->Get<std::string>("render_engine");
      if (requested == "ogre" || requested == "ogre2")
        this->engineName = requested;
    }
    this->connection = events.Connect<sim::events::SceneUpdate>([this]() { this->Repair(); });
  }

  private: void Repair()
  {
    const auto repaired = this->repair.Update(this->engineName);
    if (repaired && !this->reported)
    {
#ifdef MAICON_GZ_HARMONIC
      gzmsg << "MAICON " << this->engineName << " opaque texture depth-write compatibility enabled.\n";
#else
      ignmsg << "MAICON " << this->engineName << " opaque texture depth-write compatibility enabled.\n";
#endif
      this->reported = true;
    }
  }

  private: common::ConnectionPtr connection;
  private: OpaqueMaterialRepair repair;
  private: bool reported = false;
  private: std::string engineName = "ogre";
};
}  // namespace maicon

#ifdef MAICON_GZ_HARMONIC
GZ_ADD_PLUGIN(maicon::OpaqueMaterialSystem, sim::System, sim::ISystemConfigure)
GZ_ADD_PLUGIN_ALIAS(maicon::OpaqueMaterialSystem, "maicon::OpaqueMaterialSystem")
#else
IGNITION_ADD_PLUGIN(maicon::OpaqueMaterialSystem, sim::System, sim::ISystemConfigure)
IGNITION_ADD_PLUGIN_ALIAS(maicon::OpaqueMaterialSystem, "maicon::OpaqueMaterialSystem")
#endif
