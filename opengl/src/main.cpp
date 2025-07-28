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
#include "skybox_manager.h"

#include "game_object.h"
#include "light.h"
#include "light_manager.h"
#include "shadow_manager.h"
#include "editor/editor.h"

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
const unsigned int SHADOW_WIDTH = 16384;
const unsigned int SHADOW_HEIGHT = 16384;
const bool debug = 0;

// timing
float deltaTime = 0.0f;	
float lastFrame = 0.0f;

int main()
{
   GLFWwindow* window = initOpenGL(SCR_WIDTH, SCR_HEIGHT, "OpenGL engine");

   // systems
   EngineState engineState;
   PhysicsEngine physicsEngine;
   Renderer renderer;

   // managers
   InputManager inputManager;
   TextureManager textureManager;
   SkyboxManager skyboxManager; 
   SceneBuilder sceneBuilder;
   LightManager lightManager;
   ShadowManager shadowManager(SHADOW_WIDTH, SHADOW_HEIGHT); 

   // view
   Camera camera(glm::vec3(-5.0f, 20.0f, 0.3f));

   // editor
   Editor editor;

   // setup rng
   std::mt19937 rng(std::random_device{}());

   // ---------- Clocks ----------
   auto start_time = std::chrono::high_resolution_clock::now();
   int frames = 0;
   float last_second = 0.0f;
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
   renderer.init(SCR_WIDTH, SCR_HEIGHT, engineState, lightManager, shadowManager, skyboxManager);

   // load textures
   textureManager.loadTexture("crate", "src/assets/crate.jpg");
   textureManager.loadTexture("uvmap", "src/assets/UV_8K.png");
   textureManager.loadTexture("terrain1", "src/assets/terrain_rock_8k.jpg");
   textureManager.loadTexture("terrain2", "src/assets/terrain_grass_8k.jpg");

   std::vector<std::string> skyBoxFaces {
        std::string("src/assets/skyboxes/learnopengl/right.jpg"),
        std::string("src/assets/skyboxes/learnopengl/left.jpg"),
        std::string("src/assets/skyboxes/learnopengl/top.jpg"),
        std::string("src/assets/skyboxes/learnopengl/bottom.jpg"),
        std::string("src/assets/skyboxes/learnopengl/front.jpg"),
        std::string("src/assets/skyboxes/learnopengl/back.jpg")
   };
   textureManager.loadCubemap("skybox_default", skyBoxFaces);

   skyBoxFaces = {
        std::string("src/assets/skyboxes/milkyway/right.png"),
        std::string("src/assets/skyboxes/milkyway/left.png"),
        std::string("src/assets/skyboxes/milkyway/top.png"),
        std::string("src/assets/skyboxes/milkyway/bottom.png"),
        std::string("src/assets/skyboxes/milkyway/front.png"),
        std::string("src/assets/skyboxes/milkyway/back.png")
   };
   textureManager.loadCubemap("skybox_night", skyBoxFaces);

   // setup skybox
   skyboxManager.init(); 

   // setup scene 
   sceneBuilder.setPointers(&textureManager, &lightManager, rng);
   sceneBuilder.createScene(physicsEngine, 0);

   // setup physics
   physicsEngine.init(&engineState);

   // setup editor
   editor.setPointers(&engineState, &sceneBuilder, &physicsEngine, &camera, &skyboxManager, &cubeVertices, &cubeIndices);

   // main loop
   while (true) {
      float cpuClockStart = static_cast<float>(glfwGetTime());

      glfwPollEvents();
      if (glfwWindowShouldClose(window)) 
          break;

      // update time
      auto current_time = std::chrono::high_resolution_clock::now();
      float currentFrame = static_cast<float>(glfwGetTime());
      deltaTime = currentFrame - lastFrame;
      lastFrame = currentFrame;

      // continous input 
      inputManager.processInput(window, deltaTime);

      // editor functions
      editor.update(deltaTime, renderer.debugShader);

      // physics step
      float stepClockStart = static_cast<float>(glfwGetTime());
      if (!engineState.isPaused() or engineState.getAdvanceStep()) {
         accumulator += deltaTime;

         if (engineState.getAdvanceStep())
            accumulator = fixedTimeStep;

         while (accumulator >= fixedTimeStep) {
            physicsEngine.step(fixedTimeStep, rng);

            // rotate halos
            for (SceneBuilder::Halo& halo : sceneBuilder.allHalos) {
                glm::quat perFrameRot = glm::angleAxis(glm::radians(halo.rotSpeed), halo.rotDir);
                for (int i = 0; i < halo.indices.size(); i++) {
                    // increase orientation in x-axis by a small amoount
                    GameObject& obj = sceneBuilder.getDynamicObjects()[halo.indices[i]];

                    glm::vec3 relativePos = obj.position - halo.center;
                    glm::vec3 rotatedRelPos = perFrameRot * relativePos;

                    obj.position = halo.center + rotatedRelPos;
                    obj.orientation = perFrameRot * obj.orientation;
                    obj.modelMatrixDirty = true;
                    obj.setModelMatrix();
                    obj.updateAABB();
                    obj.updateCollider();
                }
            }
            accumulator -= fixedTimeStep;
         }
         bvhAccumulator += deltaTime;
         if (bvhAccumulator >= bvhInterval) {
            physicsEngine.getDynamicBvh().update(sceneBuilder.getDynamicObjects());
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

      if (!engineState.isPaused()) {
          if (editor.objectRainBlocks) sceneBuilder.objectRain(currentFrame, rng, 0);
          else if (editor.objectRainSpheres) sceneBuilder.objectRain(currentFrame, rng, 1);
      }

      // rendering
      float renderClockStart = static_cast<float>(glfwGetTime());
      renderer.update(camera, sceneBuilder, physicsEngine, editor);

      // calculate render time
      float renderClockEnd = static_cast<float>(glfwGetTime());
      float renderClock = renderClockEnd - renderClockStart;
      float renderClockMs = renderClock * 1000.0f;

      sceneBuilder.sceneDirty = false; 

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
               << std::fixed << std::setprecision(1) << cpuClockMs << " ms\n"

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
               << physicsEngine.getDynamicBvh().numRebuilds << "\n"

               // Objects
               << std::setw(LABEL_W) << std::left
               << "Objects:"
               << std::setw(VALUE_W) << std::left
               << sceneBuilder.getDynamicObjects().size() 
               << std::defaultfloat << "\n";

            if (deltaTime > 0.016f) {
               std::cout << "-- Warning: deltaTime is larger than 1/60 -- " << "\n";
            }
            std::cout << "-----------------------" << "\n";
            frames = 0;
            physicsEngine.getDynamicBvh().numRebuilds = 0;

            std::cout << std::setprecision(99);
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