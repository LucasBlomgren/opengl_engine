#pragma once
#include "editor/panel.h"
#include <memory>
#include <vector>

class ImGuiManager;
class EngineState;
class SceneBuilder;
class Renderer;
class ImGuiManager;
class SkyboxManager;
class MeshManager;
class TextureManager;
class FrameTimers;
class GpuTimers;

namespace Editor
{
class PanelManager {
public:
	PanelManager(
		::ImGuiManager* imguiManager,
		::EngineState* engineState,
		::Renderer* renderer,
		::SceneBuilder* sceneBuilder,
		::SkyboxManager* skyboxManager,
		::MeshManager* meshManager,
		::TextureManager* textureManager,
		::FrameTimers* frameTimers,
		::GpuTimers* gpuTimers)
		: imguiManager(imguiManager)
		, engineState(engineState)
		, renderer(renderer)
		, sceneBuilder(sceneBuilder)
		, skyboxManager(skyboxManager)
		, meshManager(meshManager)
		, textureManager(textureManager)
		, frameTimers(frameTimers)
		, gpuTimers(gpuTimers) {
		ctx.engineState = engineState;
		ctx.renderer = renderer;
		ctx.sceneBuilder = sceneBuilder;
		ctx.imguiManager = imguiManager;
		ctx.skyboxManager = skyboxManager;
		ctx.frameTimers = frameTimers;
		ctx.gpuTimers = gpuTimers;
	}

	// initialize panels
	void init();

	// render all panels
	void renderPanels();

	// context passed to panels
	Editor::PanelContext ctx;

private:
	// all panels
	std::vector<std::shared_ptr<Editor::IPanel>> panels;

	// dependencies
	::ImGuiManager* imguiManager;
	::EngineState* engineState;
	::Renderer* renderer;
	::SceneBuilder* sceneBuilder;
	::SkyboxManager* skyboxManager;
	::MeshManager* meshManager;
	::TextureManager* textureManager;
	::FrameTimers* frameTimers;
	::GpuTimers* gpuTimers;
};
}