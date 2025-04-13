#define GLM_ENABLE_EXPERIMENTAL

#include <iostream>
#include <vector>
#include <random>
#include <chrono>

#include "init_opengl.h"
#include "engine_state.h"
#include "input_manager.h"
#include "camera.h"
#include "physics.h"
#include "scene_builder.h"
#include "renderer.h"
#include "shader.h"
#include "texture_manager.h"

#include "game_object.h"
#include "cube_data.h"

#include "light.h"
#include "light_manager.h"

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
    inputManager.setPointers(&engineState, &camera);
    inputManager.init(window);

    // setup rendering
    Shader shader("src/shaders/object.vs", "src/shaders/object.fs");
    renderer.init(SCR_WIDTH, SCR_HEIGHT, engineState, lightManager, shader);

    // load textures
    textureManager.loadTexture("crate", "src/assets/crate.jpg");
    textureManager.loadTexture("uvmap", "src/assets/UV0.png");

    // setup scene builder
    sceneBuilder.setTextureManager(&textureManager);
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

    Light light(glm::vec3(350, 50, 320), glm::vec3(5, 2, 5), glm::vec3(1.0, 0.0, 0.0), 8);
    lightManager.addLight(light);

    Light light2(glm::vec3(150, 220, 100), glm::vec3(20, 2, 20), glm::vec3(0.0, 1.0, 0.0), 55);
    lightManager.addLight(light2);

    Light light3(glm::vec3(1050, 220, 1000), glm::vec3(20, 2, 20), glm::vec3(0.0, 0.0, 1.0), 80);
    lightManager.addLight(light3);

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
            newObject.linearVelocity = camera.Front * 0.0f;
        }
        if (engineState.GetPressedKey() == "Mouse2") {
            GameObject& newObject = sceneBuilder.createObject(physicsEngine, "crate", (camera.Position + camera.Front * 30.0f), glm::vec3(10, 10, 10), 1, 0, cubeVertices, indices);
            newObject.linearVelocity = camera.Front * 500.0f;
        }
        engineState.clearPressedKey();

        // continous input 
        inputManager.processInput(window, deltaTime);

        // physics step
        if (!engineState.isPaused())
            physicsEngine.step(sceneBuilder.getGameObjectList(), deltaTime, engineState.getShowNormals(), g);
        
        // rendering
        renderer.beginFrame();
        renderer.setViewProjection(camera);
        renderer.uploadDirectionalLight();
        renderer.uploadLightsToShader();
        light.draw(shader);
        light2.draw(shader);
        light3.draw(shader);
        renderer.drawGameObjects(sceneBuilder.getGameObjectList(), VAO_line);
        renderer.drawDebug(physicsEngine, VAO_contactPoint, VAO_xyz, VAO_worldFrame);

        // print FPS and object count
        float seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        if (seconds > last_second)
        {
            last_second = seconds;
            std::cout << "FPS: " << frames << "\n";
            std::cout << "Objects: " << sceneBuilder.getGameObjectList().size() << "\n";
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