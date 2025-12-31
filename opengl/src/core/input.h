#pragma once
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <vector>

// Requests for the current frame
struct FrameWants { 
    bool cameraLook = false;
    bool captureMouse = false; 
};

// Context about the current input state
struct InputContext {
    bool uiWantsMouse = false;
    bool uiWantsKeyboard = false;
    bool isPlayerMode = false;
};

// Input state for the current frame
struct InputFrame {
    bool keyDown[GLFW_KEY_LAST + 1]{};
    bool keyPressed[GLFW_KEY_LAST + 1]{};
    bool keyReleased[GLFW_KEY_LAST + 1]{};

    bool mouseDown[GLFW_MOUSE_BUTTON_LAST + 1]{};
    bool mousePressed[GLFW_MOUSE_BUTTON_LAST + 1]{};
    bool mouseReleased[GLFW_MOUSE_BUTTON_LAST + 1]{};

    glm::vec2 mousePos{};
    glm::vec2 mouseDelta{};
    float scrollDelta{};
};

// Tracks which input types have been consumed
struct Consumed {
    bool mouse = false;
    bool keyboard = false;
    bool all() const { return mouse && keyboard; }
};

// Interface for input receivers
class IInputReceiver {
public:
    virtual void handleInput(const InputFrame& in, const InputContext& ctx, Consumed& consumed, FrameWants& wants) = 0;
    virtual ~IInputReceiver() = default;
};

// Routes input to registered receivers
class InputRouter {
public:
    void add(IInputReceiver* r) { receivers.push_back(r); }

    void route(const InputFrame& in, const InputContext& ctx, Consumed& c, FrameWants& w) {
        for (auto* r : receivers) {
            r->handleInput(in, ctx, c, w);
            if (c.all()) break;
        }
    }
private:
    std::vector<IInputReceiver*> receivers;
};