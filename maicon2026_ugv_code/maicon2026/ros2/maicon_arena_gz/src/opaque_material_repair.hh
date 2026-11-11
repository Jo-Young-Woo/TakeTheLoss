#pragma once
#include <memory>
#include <string>
#include <unordered_map>

#ifdef MAICON_GZ_HARMONIC
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
namespace maicon_render = gz::rendering;
#else
#include <ignition/rendering/Geometry.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>
namespace maicon_render = ignition::rendering;
#endif

namespace maicon
{
// Must be called only on the scene's rendering thread.  Server SceneUpdate
// and GUI PreRender provide that contract in their respective processes.
class OpaqueMaterialRepair
{
  public: unsigned int Update(const std::string &engineName)
  {
    if (!maicon_render::isEngineLoaded(engineName))
      return 0;
    auto *engine = maicon_render::engine(engineName);
    unsigned int repaired = 0;
    for (unsigned int s = 0; s < engine->SceneCount(); ++s)
    {
      auto scene = engine->SceneByIndex(s);
      for (unsigned int i = 0; i < scene->VisualCount(); ++i)
      {
        auto visual = scene->VisualByIndex(i);
        repaired += this->Apply(visual->Material());
        for (unsigned int g = 0; g < visual->GeometryCount(); ++g)
          repaired += this->Apply(visual->GeometryByIndex(g)->Material());
      }
    }
    return repaired;
  }

  private: bool Apply(const maicon_render::MaterialPtr &material)
  {
    if (!material || !material->HasTexture())
      return false;
    const std::string texture = material->Texture();
    if (texture.find("maicon_arena_gz/") == std::string::npos ||
        texture.find("media/textures/") == std::string::npos)
      return false;
    const auto known = this->processed.find(material.get());
    if (known != this->processed.end() && !known->second.expired())
      return false;
    // Blending/transparency setters may change depth state: restore it last.
    material->SetAlphaFromTexture(false, 0.5, false);
    material->SetTransparency(0.0);
    material->SetDepthCheckEnabled(true);
    material->SetDepthWriteEnabled(true);
    this->processed[material.get()] = material;
    return true;
  }

  // Never retain scene resources, and repair objects recreated at old addresses.
  private: std::unordered_map<const maicon_render::Material *, std::weak_ptr<maicon_render::Material>> processed;
};
}  // namespace maicon
