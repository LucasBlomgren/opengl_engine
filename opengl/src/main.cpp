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
#include "light.h"
#include "light_manager.h"
#include "editor/editor.h"

#include "bvh.h"

void drawAABB(RaycastHit& hitData, Shader& shader, Camera& camera);

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

// editor
Editor editor;

int main()
{
    GLFWwindow* window = initOpenGL(SCR_WIDTH, SCR_HEIGHT, "OpenGL engine");

    // setup input
    inputManager.setPointers(&engineState, &camera);
    inputManager.init(window);

    // setup rendering
    Shader shader("src/shaders/object.vert", "src/shaders/object.frag");
    Shader debugShader("src/shaders/debug.vert", "src/shaders/debug.frag");
    renderer.init(SCR_WIDTH, SCR_HEIGHT, engineState, lightManager, shader, debugShader);

    // load textures
    textureManager.loadTexture("crate", "src/assets/crate.jpg");
    textureManager.loadTexture("uvmap", "src/assets/UV0.png");

    // setup scene 
    sceneBuilder.setPointers(&textureManager, &lightManager);
    sceneBuilder.createScene(physicsEngine);

    BVHTree bvhTree;
    bvhTree.build(sceneBuilder.getGameObjectList());

    // setup physics
    physicsEngine.setPointers(&sceneBuilder.getGameObjectList());

    // setup editor
    editor.setPointers(&engineState, &sceneBuilder, &physicsEngine, &camera, &cubeVertices, &indices);

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

    float bvhInterval = 1.0f / 180.0f;  // sekunder mellan BVH‑uppdateringar
    float bvhAccumulator = 0.0f;

    // setup help VAOs
    unsigned int VAO_line = setupLine();
    unsigned int VAO_xyz = setup_xyzObject();
    unsigned int VAO_worldFrame = setup_worldFrame();
    unsigned int VAO_contactPoint = setupContactPoint();

    // main loop
    while (true)
    {
        glfwPollEvents();
        if (glfwWindowShouldClose(window)) break;

        // update time
        auto current_time = std::chrono::high_resolution_clock::now();
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // continous input 
        inputManager.processInput(window, deltaTime);

        // physics step
        if (!engineState.isPaused()) {
            accumulator += deltaTime;
            while (accumulator >= fixedTimeStep) {
                physicsEngine.step(fixedTimeStep, engineState.getShowNormals(), g);
                accumulator -= fixedTimeStep;
            }
            bvhAccumulator += deltaTime;
            if (bvhAccumulator >= bvhInterval) {
                bvhTree.update(sceneBuilder.getGameObjectList());
                bvhAccumulator -= bvhInterval;
            }
        }

        // editor functions
        editor.update(deltaTime);

        // rendering
        renderer.beginFrame();
        renderer.setViewProjection(camera);
        renderer.uploadDirectionalLight();
        renderer.uploadLightsToShader();
        renderer.drawLights();
        renderer.drawGameObjects(sceneBuilder.getGameObjectList(), VAO_line);
        renderer.drawBVH(bvhTree, VAO_line);
        renderer.drawDebug(physicsEngine, VAO_contactPoint, VAO_xyz, VAO_worldFrame);

        //drawAABB(editor.getLastRayHit(), shader, camera);

        // print FPS and object count
        if (engineState.getShowFPS()) {
            float seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
            if (seconds > last_second) {
                last_second = seconds;
                std::cout << "FPS: " << frames << "\n";
                std::cout << "Step: " << std::fixed << std::setprecision(4) << deltaTime << std::endl;
                std::cout << "Objects: " << sceneBuilder.getGameObjectList().size() << "\n";
                std::cout << "BVH rebuilds: " << bvhTree.numRebuilds << std::endl;

                if (deltaTime > 0.016f) {
                    std::cout << "-- Warning: deltaTime is larger than 1/60 -- " << std::endl;
                    std::cout << "Collision pairs: " << physicsEngine.amountCollisionPairs << std::endl;
                }
                std::cout << "--------------" << std::endl;
                frames = 0;
                bvhTree.numRebuilds = 0;
            };
            frames++;
        }

        glfwSwapBuffers(window);
    }
    glfwTerminate();
}

void drawAABB(RaycastHit& hitData, Shader& shader, Camera& camera)
{
    glm::vec3 min = camera.Position + camera.Front * 100.0f - 5.0f;
    glm::vec3 max = camera.Position + camera.Front * 100.0f + 5.0f;

    std::array<float, 72> buf = {
        min.x,min.y,min.z,  max.x,min.y,min.z,
        min.x,min.y,min.z,  min.x,max.y,min.z,
        min.x,min.y,min.z,  min.x,min.y,max.z,
        max.x,max.y,min.z,  max.x,min.y,min.z,
        max.x,max.y,min.z,  min.x,max.y,min.z,
        max.x,max.y,min.z,  max.x,max.y,max.z,
        min.x,max.y,max.z,  min.x,min.y,max.z,
        min.x,max.y,max.z,  min.x,max.y,min.z,
        min.x,max.y,max.z,  max.x,max.y,max.z,
        max.x,min.y,max.z,  max.x,max.y,max.z,
        max.x,min.y,max.z,  min.x,min.y,max.z,
        max.x,min.y,max.z,  max.x,min.y,min.z
    };

    static GLuint vao = 0, vbo = 0; 
    if (vao == 0) {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);
    }

    shader.use();
    shader.setBool("useTexture", false);
    shader.setBool("useUniformColor", true);
    shader.setVec3("uColor", { 0.9f,0.7f,0.2f });
    shader.setMat4("model", glm::mat4(1.0f));

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(buf), buf.data(), GL_DYNAMIC_DRAW);

    glLineWidth(2.0f);
    glDrawArrays(GL_LINES, 0, 24);
}