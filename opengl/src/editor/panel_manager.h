#pragma once
#include "editor/panel.h"
#include <memory>
#include <vector>

class PhysicsEngine;
class ImGuiManager;
class EngineState;
class World;
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
		::PhysicsEngine* physicsEngine,
		::ImGuiManager* imguiManager,
		::EngineState* engineState,
		::Renderer* renderer,
		::World* world,
		::SceneBuilder* sceneBuilder,
		::SkyboxManager* skyboxManager,
		::MeshManager* meshManager,
		::TextureManager* textureManager,
		::FrameTimers* frameTimers,
		::GpuTimers* gpuTimers)
		: physicsEngine(physicsEngine)
		, imguiManager(imguiManager)
		, engineState(engineState)
		, renderer(renderer)
		, world(world)
		, sceneBuilder(sceneBuilder)
		, skyboxManager(skyboxManager)
		, meshManager(meshManager)
		, textureManager(textureManager)
		, frameTimers(frameTimers)
		, gpuTimers(gpuTimers) {
		ctx.engineState = engineState;
		ctx.renderer = renderer;
		ctx.world = world;
		ctx.sceneBuilder = sceneBuilder;
		ctx.imguiManager = imguiManager;
		ctx.skyboxManager = skyboxManager;
		ctx.frameTimers = frameTimers;
		ctx.gpuTimers = gpuTimers;
	}

	// initialize panels
	void init();

	// render all panels
	void renderPanels(float deltaTime);

	// context passed to panels
	Editor::PanelContext ctx;

private:
	// all panels
	std::vector<std::shared_ptr<Editor::IPanel>> panels;

	// dependencies
	::PhysicsEngine* physicsEngine;
	::ImGuiManager* imguiManager;
	::EngineState* engineState;
	::Renderer* renderer;
	::World* world;
	::SceneBuilder* sceneBuilder;
	::SkyboxManager* skyboxManager;
	::MeshManager* meshManager;
	::TextureManager* textureManager;
	::FrameTimers* frameTimers;
	::GpuTimers* gpuTimers;
};
}