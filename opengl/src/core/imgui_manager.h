#pragma once

#include "imgui-docking/imgui.h"
#include "imgui-docking/imgui_impl_glfw.h"
#include "imgui-docking/imgui_impl_opengl3.h"

#include "engine_state.h"
#include "graphics/mesh/mesh_manager.h"
#include "scene_builder.h"
#include "graphics/renderer/renderer.h"
#include "graphics/textures/texture_manager.h"
#include "graphics/skybox/skybox_manager.h"
#include "timer.h"

class ImGuiManager : IInputReceiver {
public:
    void addInputRouter(InputRouter& router);
    void handleInput(const InputFrame& in, const InputContext& ctx, Consumed& consumed, FrameWants& wants);

    void init(GLFWwindow* windo);
    void newFrame();
    void render();
    void shutdown();
};