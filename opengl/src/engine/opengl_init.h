#pragma once

#include <GLFW/glfw3.h>

GLFWwindow* OpenGL_init(int width, int height, const std::string& title) {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // specify OpenGL version 3.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // use core profile for modern OpenGL features

    GLFWmonitor* primary = glfwGetPrimaryMonitor(); // get the primary monitor for fullscreen
    const GLFWvidmode* mode = glfwGetVideoMode(primary); // get the video mode of the primary monitor to determine its resolution

    GLFWwindow* window = glfwCreateWindow(width, height, title.c_str(), primary, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return nullptr;
    }

    glfwSetWindowAttrib(window, GLFW_DECORATED, GLFW_FALSE); // remove title bar and borders
    glfwSetWindowPos(window, 0, 0); // position in top-left corner of primary monitor

    glfwMakeContextCurrent(window); // set the OpenGL context to the window
    glfwSwapInterval(1); // enable vsync

    // Load GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        return nullptr;
    }

    // GL config
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    return window;
}