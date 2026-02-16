#pragma once

#include "input.h"
#include "aabb.h"
#include "physics/raycast.h"
#include "viewport_fbo.h"
#include "panel_manager.h"

struct GLFWwindow;
class InputManager;
class EngineState;
class SceneBuilder;
class PhysicsEngine;
class Camera;
class SkyboxManager;
class Shader;
class Renderer;
class ImGuiManager;
class MeshManager;
class TextureManager;

namespace Editor 
{
class EditorMain : public IInputReceiver {
public:
    ViewportFBO viewportFBO{ 1920, 1080 };
    std::unique_ptr<PanelManager> panelManager;

    void handleInput(const InputFrame& in, const InputContext& ctx, Consumed& consumed, FrameWants& wants);

    void init(
        float SCR_WIDTH,
        float SCR_HEIGHT,
        EngineState* engineState,
        World* world,
        SceneBuilder* sceneBuilder,
        PhysicsEngine* physicsEngine,
        InputManager* inputManager,
        Camera* camera,
        GLFWwindow* window,
        ImGuiManager* imguiManager,
        Renderer* renderer,
        SkyboxManager* skyboxManager,
        MeshManager* meshManager,
        TextureManager* textureManager, 
        FrameTimers* frameTimers,
        GpuTimers* gpuTimers
    );

    bool flag_drawUI = true;
    void drawUI(InputContext& ctx, float deltaTime);

    // activate/deactivate editor mode
    void activate();
    void deactivate();

    // update
    void fixedUpdate(float fixedTimeStep);
    void update(Shader& shader);

    // select object
    bool objectIsSelected = false;
    bool objectIsHovered = false;
    GameObjectHandle selectedObjectHandle;
    GameObjectHandle hoveredObjectHandle;

    glm::vec3 selectionOffsetLocal{ 0.0f, 0.0f, 0.0f };
    void syncSelectionOffset();
    void selectObject(const InputContext& ctx);
    void dropObject();
    void updateSelectedObject(float fixedTimeStep);

    // raycast for placement/selection/etc
    RaycastHit rayCast(float length);
    RaycastHit& getLastRayHit();
    AABB aabbToPlace;
    bool placementObstructed = true;
    void placeObject();
    void createPlaceObjectAABB(Shader& shader);
    void drawAABB(const AABB& aabb, Shader& shader, glm::vec3 color = { 0.9f,0.7f,0.2f });

    glm::vec3 objectRainPos = { 0.0f, 100.0f, 0.0f };
    bool objectRainBlocks = false;
    bool objectRainSpheres = false;

private:
    float SCR_WIDTH;
    float SCR_HEIGHT;
    EngineState* engineState = nullptr;
    World* world = nullptr;
    SceneBuilder* sceneBuilder = nullptr;
    PhysicsEngine* physicsEngine = nullptr;
    InputManager* inputManager = nullptr;
    Camera* camera = nullptr;
    GLFWwindow* window = nullptr;
    ImGuiManager* imguiManager = nullptr;
    Renderer* renderer = nullptr;
    SkyboxManager* skyboxManager = nullptr;
    MeshManager* meshManager = nullptr;
    TextureManager* textureManager = nullptr;
    FrameTimers* frameTimers = nullptr;
    GpuTimers* gpuTimers = nullptr;

    bool viewportCapturedRMB = false;
    bool viewportCapturedLMB = false;
    float viewportMouseX = 0.0f;
    float viewportMouseY = 0.0f;
    float viewportDisplayW = 0.0f;
    float viewportDisplayH = 0.0f;

    float mouseXLastFrame = 0.0f;
    float mouseYLastFrame = 0.0f;

    // selection and placement
    RaycastHit lastHitData;
    bool drawPlacementAABB = false;
    constexpr static float SELECT_RANGE = 5000.0f;
    constexpr static float OBJ_PLACE_DISTANCE = 150.0f;
    constexpr static glm::vec3 OBJ_PLACE_SIZE{ 1.0f, 1.0f, 1.0f };
    constexpr static float SHOOT_VELOCITY = 100.0f;
};
}