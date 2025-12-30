#pragma once
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <vector>

struct InputContext {
    bool uiWantsMouse = false;
    bool uiWantsKeyboard = false;
    bool isPlayerMode = false;
};

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

struct Consumed {
    bool mouse = false;
    bool keyboard = false;
    bool all() const { return mouse && keyboard; }
};

class IInputReceiver {
public:
    virtual void handleInput(const InputFrame& in, const InputContext& ctx, Consumed& consumed) = 0;
    virtual ~IInputReceiver() = default;
};

class InputRouter {
public:
    void add(IInputReceiver* r) { receivers.push_back(r); }

    void route(const InputFrame& in, const InputContext& ctx, Consumed& c) {
        for (auto* r : receivers) {
            r->handleInput(in, ctx, c);
            if (c.all()) break;
        }
    }
private:
    std::vector<IInputReceiver*> receivers;
};