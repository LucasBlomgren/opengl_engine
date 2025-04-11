#define GLM_ENABLE_EXPERIMENTAL

#include <iostream>
#include <vector>
#include <random>
#include <chrono>

#include "initOpenGL.h"
#include "engineState.h"
#include "InputManager.h"
#include "camera.h"
#include "physics.h"
#include "sceneBuilder.h"
#include "renderer.h"
#include "shader.h"
#include "textureManager.h"
#include "GameObject.h"

#include "drawContactPoints.h"
#include "xyzObject.h"
#include "worldFrame.h"
#include "drawLine.h"
#include "cubeData.h"

#include "Light.h"
#include "LightManager.h"

// overload operator<< for glm::vec3 
std::ostream& operator<<(std::ostream& os, const glm::vec3& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

// window resize callback
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

// settings
const unsigned int SCR_WIDTH = 1920;
const unsigned int SCR_HEIGHT = 1080;

// timing
float deltaTime = 0.0f;	
float lastFrame = 0.0f;

// systems
EngineState engineState;
PhysicsEngine physicsEngine;
Renderer renderer;

// managers
InputManager inputManager;
TextureManager textureManager;
SceneBuilder sceneBuilder;
LightManager lightManager;

// view
Camera camera(glm::vec3(-50.0f, 200.0f, 3.0f));

int main()
{
    GLFWwindow* window = initOpenGL(SCR_WIDTH, SCR_HEIGHT, "OpenGL engine");

    // setup input
    inputManager.SetPointers(&engineState, &camera);
    inputManager.Init(window);

    // setup rendering
    Shader shader("object.vs", "object.fs");
    renderer.Init(SCR_WIDTH, SCR_HEIGHT, engineState, lightManager, shader);

    // load textures
    textureManager.LoadTexture("crate", "crate.jpg");
    textureManager.LoadTexture("uvmap", "UV0.png");

    // setup scene builder
    sceneBuilder.SetTextureManager(&textureManager);
    // create scene
    sceneBuilder.createScene(physicsEngine, cubeVertices, indices);

    // set camera starting angle
    camera.Yaw += 130;
    camera.Pitch -= 30;
    camera.ProcessMouseMovement(0,0);

    // setup rng
    std::random_device rd;
    std::mt19937 g(rd());

    // setup clock
    auto start_time = std::chrono::high_resolution_clock::now();
    int frames = 0;
    int last_second = 0;

    // setup help VAOs
    unsigned int VAO_line = setupLine();
    unsigned int VAO_xyz = setup_xyzObject();
    unsigned int VAO_worldFrame = setup_worldFrame();
    unsigned int VAO_contactPoint = setupContactPoint();

    Light light(glm::vec3(350, 50, 320), glm::vec3(20, 2, 20), glm::vec3(1.0, 1.0, 1.0), 5);
    light.scale = glm::vec3(5, 2, 5);
    lightManager.AddLight(light);

    Light light2(glm::vec3(150, 220, 100), glm::vec3(20, 2, 20), glm::vec3(1.0, 1.0, 1.0), 105);
    lightManager.AddLight(light2);

    //lightManager.setDirectionalLight(glm::vec3(-0.0f, -1.0f, -0.0f), glm::vec3(0.1), glm::vec3(1.0), glm::vec3(0.5));

    // main loop
    while (!glfwWindowShouldClose(window))
    {
        // update time
        auto current_time = std::chrono::high_resolution_clock::now();
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // "gameplay" functionality
        if (engineState.GetPressedKey() == "H") {
            sceneBuilder.createScene(physicsEngine, cubeVertices, indices);
        }
        if (engineState.GetPressedKey() == "Mouse1") {
            GameObject& newObject = sceneBuilder.createObject(physicsEngine, "crate", (camera.Position + camera.Front * 30.0f), glm::vec3(10, 10, 10), 1, 0, cubeVertices, indices);
            newObject.linearVelocity = camera.Front * 500.0f;
        }
        engineState.ClearPressedKey();

        // continous input 
        inputManager.ProcessInput(window, deltaTime);

        // physics step
        if (!engineState.IsPaused())
            physicsEngine.step(window, sceneBuilder.GetGameObjectList(), deltaTime, engineState.GetShowNormals(), g);
        
        // rendering
        renderer.BeginFrame();
        renderer.SetViewProjection(camera);
        renderer.UploadDirectionalLight();
        renderer.UploadLightsToShader();
        light.Draw(shader);
        light2.Draw(shader);
        renderer.DrawGameObjects(sceneBuilder.GetGameObjectList(), VAO_line);
        renderer.DrawDebug(physicsEngine, VAO_contactPoint, VAO_xyz, VAO_worldFrame);

        // print FPS and object count
        float seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        if (seconds > last_second)
        {
            last_second = seconds;
            std::cout << "FPS: " << frames << "\n";
            std::cout << "Objects: " << sceneBuilder.GetGameObjectList().size() << "\n";
            frames = 0;
        };
        frames++;

        // swap buffers and poll IO events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwTerminate();
    return 0;
}