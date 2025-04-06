#define GLM_ENABLE_EXPERIMENTAL

#include <iostream>
#include <vector>
#include <random>
#include <chrono>

#include "initOpenGL.h"
#include "engineState.h"
#include "inputManager.h"
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

// view
Camera camera(glm::vec3(-50.0f, 200.0f, 3.0f));

// all game objects
std::vector<GameObject> GameObjectList;

int main()
{
    GLFWwindow* window = initOpenGL(SCR_WIDTH, SCR_HEIGHT, "OpenGL engine");

    // setup input
    inputManager.SetPointers(&engineState, &camera);
    inputManager.Init(window);

    // setup rendering
    Shader shader("object.vs", "object.fs");
    renderer.Init(SCR_WIDTH, SCR_HEIGHT, engineState, shader);

    // load textures
    unsigned int crateTexture = TextureManager::LoadTexture("crate", "crate.jpg");
    unsigned int UVmapTexture = TextureManager::LoadTexture("uvmap", "UV0.png");

    // create scene
    sceneBuilder.createScene(physicsEngine, GameObjectList, cubeVertices, indices);

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
            sceneBuilder.createScene(physicsEngine, GameObjectList, cubeVertices, indices);
        }
        if (engineState.GetPressedKey() == "Mouse1") {
            sceneBuilder.createObject(physicsEngine, GameObjectList, camera.Position + camera.Front * 30.0f, glm::vec3(10, 10, 10), 1, 0, 1, cubeVertices, indices);
            GameObjectList[sceneBuilder.objectId - 1].linearVelocity = camera.Front * 500.0f;
        }
        engineState.ClearPressedKey();

        // continous input 
        inputManager.ProcessInput(window, deltaTime);

        // physics step
        if (!engineState.IsPaused())
            physicsEngine.step(window, GameObjectList, deltaTime, engineState.GetShowNormals(), g);
        
        //shader.setVec3("lightColor", glm::vec3(lightStrength, lightStrength, lightStrength));
        //shader.setVec3("lightPos", lightPos);
        //shader.setVec3("viewPos", camera.Position);

        // rendering
        renderer.BeginFrame();
        renderer.SetViewProjection(camera);
        renderer.DrawGameObjects(GameObjectList, VAO_line);
        renderer.DrawDebug(physicsEngine, VAO_contactPoint, VAO_xyz, VAO_worldFrame);

        // render light source
        //light->setModelMatrix();
        //shader.setMat4("model", light->modelMatrix);
        //shader.setVec3("uColor", glm::vec3(255,255,255));
        //glBindVertexArray(light->VAO);
        //glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

        // print FPS and object count
        float seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        if (seconds > last_second)
        {
            last_second = seconds;
            std::cout << "FPS: " << frames << "\n";
            std::cout << "Objects: " << GameObjectList.size() << "\n";
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