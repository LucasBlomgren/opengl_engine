#pragma once

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include "engine_state.h"
#include "graphics/mesh/mesh_manager.h"
#include "scene_builder.h"
#include "graphics/renderer/renderer.h"
#include "graphics/textures/texture_manager.h"
#include "timer.h"

class ImGuiManager {
public:
    void init(GLFWwindow* window, EngineState& es, SceneBuilder& sb, MeshManager& mm, Renderer& r, TextureManager& tm);
    void newFrame();
    void setInputMode(bool cameraMode);
    void render();
    void shutdown();
    
    void mainUI(float deltaTime, FrameTimers& frameTimers, GpuTimers& gpu, size_t amountObjects);
    void settingsUI();
    void performanceUI(float deltaTime, FrameTimers& frameTimers, GpuTimers& gpu, size_t amountObjects);

    void selectedObjectUI(GameObject* obj);

private:
    EngineState* engineState = nullptr;
    SceneBuilder* sceneBuilder = nullptr;
    MeshManager* meshManager = nullptr;
    Renderer* renderer = nullptr;   
    TextureManager* textureManager = nullptr;
    ImGuiIO* io = nullptr;

    float uiTimer = 0.0f;
};