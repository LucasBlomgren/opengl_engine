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
struct GpuTimers;

namespace Editor
{
class PanelManager {
public:
	PanelManager(
		::Editor::EditorMain* editorMain,
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
		: editorMain(editorMain)
		, physicsEngine(physicsEngine)
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
		ctx.editorMain = editorMain;
		ctx.physicsEngine = physicsEngine;
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
	::Editor::EditorMain* editorMain;
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