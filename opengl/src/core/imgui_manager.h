#pragma once

#include "imgui-docking/imgui.h"
#include "imgui-docking/imgui_impl_glfw.h"
#include "imgui-docking/imgui_impl_opengl3.h"

class ImGuiManager {
public:
    void init(GLFWwindow* windo);
    void newFrame();
    void render();
    void shutdown();
};