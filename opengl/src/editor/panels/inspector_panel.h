#pragma once

#include "editor/panel.h"

class GameObject;
class ImGuiManager;
class EngineState;
class Renderer;
class SceneBuilder;
class SkyboxManager;
class FrameTimers;
struct GpuTimers;

namespace Editor
{
    // Settings panel
    class InspectorPanel : public IPanel {
    public:
        virtual const char* GetName() const override { return "Settings"; }
        virtual void OnImGuiRender(const PanelContext& ctx) override;
    };
}
