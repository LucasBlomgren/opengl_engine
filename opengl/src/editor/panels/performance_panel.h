#pragma once
#include "editor/panel.h"

class GameObject;
class ImGuiManager;
class EngineState;
class Renderer;
class SceneBuilder;
class SkyboxManager;
class MeshManager;
class TextureManager;
class FrameTimers;
class GpuTimers;

namespace Editor 
{
class PerformancePanel : public Editor::IPanel {
public:
	virtual const char* GetName() const override { return "Performance"; }
	virtual void OnImGuiRender(const PanelContext& ctx) override;

private:
	float uiTimer = 0.0f;
};
}