#include "opaque_material_repair.hh"
#include <array>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numeric>

#ifdef MAICON_GZ_HARMONIC
#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/plugin/Register.hh>
namespace arena_gui = gz::gui;
#else
#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/gui/Plugin.hh>
#include <ignition/plugin/Register.hh>
namespace arena_gui = ignition::gui;
#endif

namespace maicon
{
class MaiconOpaqueMaterialGui final : public arena_gui::Plugin
{
  Q_OBJECT

  protected: void LoadConfig(const tinyxml2::XMLElement *) override
  {
    this->title = "Opaque texture compatibility";
    if (const char *path = std::getenv("MAICON_GUI_METRICS_PATH"))
      this->metricsPath = path;
    auto *window = arena_gui::App()->findChild<arena_gui::MainWindow *>();
    if (window)
      window->installEventFilter(this);
  }

  protected: bool eventFilter(QObject *object, QEvent *event) override
  {
    if (event->type() == arena_gui::events::PreRender::kType)
    {
      this->MeasureFrame();
      const auto changed = this->repair.Update("ogre");
      if (changed && !this->reported)
      {
#ifdef MAICON_GZ_HARMONIC
        gzmsg << "MAICON GUI Ogre 1 opaque texture depth-write compatibility enabled.\n";
#else
        ignmsg << "MAICON GUI Ogre 1 opaque texture depth-write compatibility enabled.\n";
#endif
        this->reported = true;
      }
    }
    return QObject::eventFilter(object, event);
  }

  private: void MeasureFrame()
  {
    if (this->metricsPath.empty() || this->metricsFinished)
      return;
    const auto now = std::chrono::steady_clock::now();
    if (!this->metricsStarted)
    {
      this->metricsStart = now;
      this->metricsStarted = true;
      return;
    }
    const double sinceStart = std::chrono::duration<double>(now-this->metricsStart).count();
    if (sinceStart < 5.0)
      return;
    const double elapsed = sinceStart-5.0;
    if (elapsed < 60.0)
    {
      ++this->frameBuckets[static_cast<unsigned int>(elapsed)];
      return;
    }
    this->metricsFinished = true;
    const auto frames = std::accumulate(this->frameBuckets.begin(), this->frameBuckets.end(), 0ULL);
    const auto minimum = *std::min_element(this->frameBuckets.begin(), this->frameBuckets.end());
    try
    {
      const std::filesystem::path path(this->metricsPath);
      if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
      const auto temporary = path.string()+".tmp";
      std::ofstream output(temporary);
      output.exceptions(std::ios::failbit | std::ios::badbit);
      output << std::setprecision(12)
             << "{\n  \"schema\": \"maicon_gui_render_loop_v1\",\n"
             << "  \"event\": \"PreRender\",\n  \"measurement\": \"render-loop callbacks, not display presents\",\n"
             << "  \"warmup_s\": 5,\n  \"window_s\": 60,\n  \"observed_completion_s\": " << sinceStart << ",\n"
             << "  \"frame_events\": " << frames << ",\n  \"mean_render_loop_fps\": " << frames/60.0 << ",\n"
             << "  \"minimum_1s_render_loop_fps\": " << minimum << ",\n  \"frames_per_1s_bucket\": [";
      for (size_t i = 0; i < this->frameBuckets.size(); ++i)
        output << (i ? ", " : "") << this->frameBuckets[i];
      output << "]\n}\n";
      output.close();
      std::filesystem::rename(temporary, path);
    }
    catch (const std::exception &)
    {
#ifdef MAICON_GZ_HARMONIC
      gzerr << "MAICON GUI render-loop metrics could not be saved.\n";
#else
      ignerr << "MAICON GUI render-loop metrics could not be saved.\n";
#endif
    }
  }

  private: OpaqueMaterialRepair repair;
  private: bool reported = false;
  private: std::string metricsPath;
  private: bool metricsStarted = false;
  private: bool metricsFinished = false;
  private: std::chrono::steady_clock::time_point metricsStart;
  private: std::array<unsigned long long, 60> frameBuckets{};
};
}  // namespace maicon

#ifdef MAICON_GZ_HARMONIC
GZ_ADD_PLUGIN(maicon::MaiconOpaqueMaterialGui, arena_gui::Plugin)
#else
IGNITION_ADD_PLUGIN(maicon::MaiconOpaqueMaterialGui, arena_gui::Plugin)
#endif
#include "opaque_material_gui.moc"
