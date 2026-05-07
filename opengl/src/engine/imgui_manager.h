#pragma once

class ImGuiManager {
public:
    void init(GLFWwindow* windo);
    void newFrame();
    void render();
    void shutdown();
};