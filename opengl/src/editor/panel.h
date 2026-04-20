#pragma once

#include "core/slot_map.h"

class PhysicsEngine;
class GameObject;
class EngineState;
class Renderer;
class SceneBuilder;
class World;
class ImGuiManager;
class SkyboxManager;
class MeshManager;
class TextureManager;
class FrameTimers;
struct GpuTimers;

namespace Editor 
{
class EditorMain;

// Context which is passed to panels during rendering
struct PanelContext {
	float deltaTime = 0.0f;
	float fps = 0.0f;
	size_t amountObjects = 0;
	size_t amountAwakeObjects = 0;
	size_t amountAsleepObjects = 0;
	size_t amountStaticObjects = 0;
	size_t amountTerrainTris = 0;
	size_t amountContacts = 0;
	size_t amountColliders = 0;
	GameObjectHandle selectedObjectHandle;
	bool objectIsSelected = false;

	EditorMain* editorMain = nullptr;
	::PhysicsEngine* physicsEngine = nullptr;
	::EngineState* engineState = nullptr;
	::Renderer* renderer = nullptr;
	::SceneBuilder* sceneBuilder = nullptr;
	::World* world = nullptr;
	::ImGuiManager* imguiManager = nullptr;
	::SkyboxManager* skyboxManager = nullptr;
	::MeshManager* meshManager = nullptr;
	::TextureManager* textureManager = nullptr;
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