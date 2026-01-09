#pragma once

class Renderer;
class SceneBuilder;
class ImGuiManager;
class FrameTimers;
class GpuTimers;

namespace Editor 
{
// Context which is passed to panels during rendering
struct PanelContext {
	float deltaTime = 0.0f;
	float fps = 0.0f;
	size_t amountObjects = 0;

	::Renderer* renderer = nullptr;
	::SceneBuilder* sceneBuilder = nullptr;
	::ImGuiManager* imguiManager = nullptr;
	::FrameTimers* frameTimers = nullptr;
	::GpuTimers* gpuTimers = nullptr;
};

// Interface for editor panels
class IPanel {
public:
	virtual ~IPanel() = default;
	virtual const char* GetName() const = 0;
	virtual void OnImGuiRender(const PanelContext& ctx) = 0;

protected:
};
}