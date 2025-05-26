#define GLM_FORCE_SIMD_AVX2 
#define GLM_ENABLE_EXPERIMENTAL

#include <windows.h>

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

// overload operator<< for glm::vec3 
std::ostream& operator<<(std::ostream& os, const glm::vec3& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

// settings
const unsigned int SCR_WIDTH = 1920;
const unsigned int SCR_HEIGHT = 1080;
const bool debug = 0;

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

   // setup rng
   std::mt19937 rng(std::random_device{}());

   // ---------- Clocks ----------
   auto start_time = std::chrono::high_resolution_clock::now();
   int frames = 0;
   int last_second = 0;
   // fixed timestep
   const float fixedTimeStep = 1.0f / 120.0f;
   float accumulator = 0.0f;
   float lastFrame = static_cast<float>(glfwGetTime());
   float bvhInterval = fixedTimeStep;  // sekunder mellan BVH‑uppdateringar
   float bvhAccumulator = 0.0f;

   // setup input
   inputManager.setPointers(&engineState, &camera);
   inputManager.init(window);

   // setup rendering
   Shader shader("src/shaders/object.vert", "src/shaders/object.frag");
   Shader debugShader("src/shaders/debug.vert", "src/shaders/debug.frag");
   renderer.init(SCR_WIDTH, SCR_HEIGHT, engineState, lightManager, shader, debugShader);

   // load textures
   textureManager.loadTexture("crate", "src/assets/crate.jpg");
   textureManager.loadTexture("uvmap", "src/assets/UV_8K.png");

   // setup scene 
   sceneBuilder.setPointers(&textureManager, &lightManager, rng);
   sceneBuilder.createScene(physicsEngine);

   // setup bvh
   BVHTree bvhTree;
   bvhTree.build(sceneBuilder.getGameObjectList());

   // setup physics
   physicsEngine.init(&sceneBuilder.getGameObjectList(), &engineState, &bvhTree);

   // setup editor
   editor.setPointers(&engineState, &sceneBuilder, &physicsEngine, &camera, &cubeVertices, &indices);

   // setup help VAOs
   unsigned int VAO_line = setupLine();
   unsigned int VAO_xyz = setup_xyzObject();
   unsigned int VAO_worldFrame = setup_worldFrame();
   unsigned int VAO_contactPoint = setupContactPoint();

   // main loop
   while (true)
   {
      float cpuClockStart = static_cast<float>(glfwGetTime());
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
      float stepClockStart = static_cast<float>(glfwGetTime());
      if (!engineState.isPaused() or engineState.getAdvanceStep()) {
         accumulator += deltaTime;

         if (engineState.getAdvanceStep())
            accumulator = fixedTimeStep;

         while (accumulator >= fixedTimeStep) {
            physicsEngine.step(fixedTimeStep, rng);
            accumulator -= fixedTimeStep;
         }
         bvhAccumulator += deltaTime;
         if (bvhAccumulator >= bvhInterval) {
            bvhTree.update(sceneBuilder.getGameObjectList());
            bvhAccumulator -= bvhInterval;
         }
      }

      if (engineState.getAdvanceStep()) {
            engineState.setAdvanceStep(false);
      }

      // calculate step time
      float stepClockStop = static_cast<float>(glfwGetTime());
      float stepClock = stepClockStop - stepClockStart;
      float stepClockMs = stepClock * 1000.0f;

      // rendering
      float renderClockStart = static_cast<float>(glfwGetTime());
      renderer.beginFrame();
      renderer.setViewProjection(camera);
      renderer.uploadDirectionalLight();
      renderer.uploadLightsToShader();
      renderer.drawLights();
      renderer.drawGameObjects(sceneBuilder.getGameObjectList(), VAO_line);
      renderer.drawBVH(bvhTree, VAO_line);
      renderer.drawDebug(physicsEngine, VAO_contactPoint, VAO_xyz, VAO_worldFrame);
      // calculate render time
      float renderClockEnd = static_cast<float>(glfwGetTime());
      float renderClock = renderClockEnd - renderClockStart;
      float renderClockMs = renderClock * 1000.0f;

      // editor functions
      editor.update(deltaTime, debugShader);

      if (!engineState.isPaused() and editor.objectRain)
        sceneBuilder.objectRain(currentFrame, rng);

      // swap buffers
      float t1 = static_cast<float>(glfwGetTime());
      glfwSwapBuffers(window);
      float t2 = static_cast<float>(glfwGetTime());
      float swapTime = (t2 - t1);

      // calculate CPU time
      float cpuClockStop = static_cast<float>(glfwGetTime());
      float cpuClock = (cpuClockStop - cpuClockStart) - swapTime;
      float cpuClockMs = cpuClock * 1000.0f;
      float frameTimeMs = (cpuClockStop - cpuClockStart) * 1000.0f;

      // print FPS and other stats
      if (engineState.getShowFPS()) {
         float seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
         if (seconds > last_second) {
            last_second = seconds;
            const int LABEL_W = 10;
            const int VALUE_W = 0;

            std::cout
               // FPS
               << std::setw(LABEL_W) << std::left
               << "FPS:"
               // värdefält, vänster-justerat
               << std::setw(VALUE_W) << std::left
               << frames << "\n"

               // CPU Time
               << std::setw(LABEL_W) << std::left
               << "CPU:"
               // värdefält, vänster-justerat
               << std::setw(VALUE_W) << std::left
               << std::fixed << std::setprecision(1) << frameTimeMs << " ms\n"

               // Physics Time
               << std::setw(LABEL_W) << std::left
               << "Physics:"
               << std::setw(VALUE_W) << std::left
               << std::setprecision(1) << stepClockMs << " ms\n"

               // Render Time
               << std::setw(LABEL_W) << std::left
               << "Render:"
               << std::setw(VALUE_W) << std::left
               << std::setprecision(1) << renderClockMs << " ms\n"

               // BVH rebuilds
               << std::setw(LABEL_W) << std::left
               << "BVH:"
               << std::setw(VALUE_W) << std::left
               << bvhTree.numRebuilds << "\n"

               // Objects
               << std::setw(LABEL_W) << std::left
               << "Objects:"
               << std::setw(VALUE_W) << std::left
               << sceneBuilder.getGameObjectList().size() 
               << std::defaultfloat << "\n";

            if (deltaTime > 0.016f) {
               std::cout << "-- Warning: deltaTime is larger than 1/60 -- " << "\n";
            }
            std::cout << "-----------------------" << "\n";
            frames = 0;
            bvhTree.numRebuilds = 0;
         }
         frames++;
      }

      if (debug)
        if (current_time - start_time > std::chrono::seconds(8))
            break;
   }
   glfwTerminate();

   return 0;
}