#pragma once

#include "core/slot_map.h"

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
class GpuTimers;

namespace Editor 
{
// Context which is passed to panels during rendering
struct PanelContext {
	float deltaTime = 0.0f;
	float fps = 0.0f;
	size_t amountObjects = 0;
	size_t amountAwakeObjects = 0;
	size_t amountAsleepObjects = 0;
	size_t amountStaticObjects = 0;
	size_t amountTerrainTris = 0;
	size_t amountCollisions = 0;
	GameObjectHandle selectedObjectHandle;
	bool objectIsSelected = false;

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