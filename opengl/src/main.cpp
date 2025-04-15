#define GLM_FORCE_SIMD_AVX2 
#define GLM_ENABLE_EXPERIMENTAL

#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <iomanip>

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

#include "editor.h"

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

// editor
Editor editor(engineState, sceneBuilder, physicsEngine, cubeVertices, indices);

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
    sceneBuilder.setPointers(&textureManager, &lightManager);
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

    // fixed timestep
    const float fixedTimeStep = 1.0f / 360.0f;
    float accumulator = 0.0f;
    float lastFrame = static_cast<float>(glfwGetTime());

    // setup help VAOs
    unsigned int VAO_line = setupLine();
    unsigned int VAO_xyz = setup_xyzObject();
    unsigned int VAO_worldFrame = setup_worldFrame();
    unsigned int VAO_contactPoint = setupContactPoint();

    // main loop
    while (!glfwWindowShouldClose(window))
    {
        // update time
        auto current_time = std::chrono::high_resolution_clock::now();
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // editor functions
        editor.update(camera);

        // continous input 
        inputManager.processInput(window, deltaTime);

        // physics step
        if (!engineState.isPaused()) {
            accumulator += deltaTime;
            while (accumulator >= fixedTimeStep) {
                physicsEngine.step(sceneBuilder.getGameObjectList(), fixedTimeStep, engineState.getShowNormals(), g);
                accumulator -= fixedTimeStep;
            }
        }
        
        // rendering
        renderer.beginFrame();
        renderer.setViewProjection(camera);
        renderer.uploadDirectionalLight();
        renderer.uploadLightsToShader();
        renderer.drawLights();
        renderer.drawGameObjects(sceneBuilder.getGameObjectList(), VAO_line);
        renderer.drawDebug(physicsEngine, VAO_contactPoint, VAO_xyz, VAO_worldFrame);

        // print FPS and object count
        float seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        if (seconds > last_second)
        {
            last_second = seconds;
            std::cout << "FPS: " << frames << "\n";
            std::cout << "Step: " << std::fixed << std::setprecision(4) << deltaTime << std::endl;
            std::cout << "Objects: " << sceneBuilder.getGameObjectList().size() << "\n";

            if (deltaTime > 0.016f) {
                std::cout << "-- Warning: deltaTime is larger than 1/60 -- " << std::endl;
                std::cout << "Collision pairs: " << physicsEngine.amountCollisionPairs << std::endl;
            }
            std::cout << "--------------" << std::endl;
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