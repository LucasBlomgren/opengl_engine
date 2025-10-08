
#include "pch.h"

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
#include "geometry/geometry_loader.h"

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

// timing
float deltaTime = 0.0f;	
float lastFrame = 0.0f;

int main()
{
    // initialize and configure OpenGL
    GLFWwindow* window = initOpenGL(SCR_WIDTH, SCR_HEIGHT, "OpenGL engine");

    // GPU logging
    static constexpr int NQ = 4;          // ringbuffer (4–8 är lagom)
    GLuint qShadow[NQ]{}, qMain[NQ]{}, qDebug[NQ]{};
    int writeIdx = 0;                     // var vi börjar nya queries
    int readIdx = (writeIdx + 1) % NQ;    // var vi försöker läsa från
    glGenQueries(NQ, qShadow);
    glGenQueries(NQ, qMain);
    glGenQueries(NQ, qDebug);
    double gpuShadowMs = 0.0, gpuMainMs = 0.0, gpuDebugMs = 0.0;

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
    const float fixedTimeStep = 1.0f / 60.0f;
    float accumulator = 0.0f;
    lastFrame = static_cast<float>(glfwGetTime());

    // setup input
    inputManager.setPointers(&engineState, &camera);
    inputManager.init(window);

    // setup rendering
    renderer.init(SCR_WIDTH, SCR_HEIGHT, engineState, lightManager, shadowManager, skyboxManager);

    // load textures
    textureManager.loadTexture("crate", "src/assets/crate.jpg");
    textureManager.loadTexture("uvmap", "src/assets/UV_8K.png");
    //textureManager.loadTexture("terrain1", "src/assets/terrain_rock_8k.jpg");
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

    // load geometry data
    loadCubeData();
    loadSphereData();
    //loadIcoSphereData();

    // setup scene 
    sceneBuilder.setPointers(&physicsEngine, &textureManager, &lightManager, rng);
    sceneBuilder.createScene(physicsEngine, 0);

    // setup physics
    physicsEngine.init(&engineState);

    // setup editor
    editor.setPointers(&engineState, &sceneBuilder, &physicsEngine, &camera, &skyboxManager, &cubeVertices, &cubeIndices);

    // main loop
    while (true) {
        float cpuClockStart = static_cast<float>(glfwGetTime());
        // update time
        auto current_time = std::chrono::high_resolution_clock::now();
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        GLuint avail = 0;
        // SHADOW
        glGetQueryObjectuiv(qShadow[readIdx], GL_QUERY_RESULT_AVAILABLE, &avail);
        if (avail) {
            GLuint64 ns = 0;
            glGetQueryObjectui64v(qShadow[readIdx], GL_QUERY_RESULT, &ns); // blockar inte när avail==true
            gpuShadowMs = ns / 1e6;
        }

        // MAIN
        glGetQueryObjectuiv(qMain[readIdx], GL_QUERY_RESULT_AVAILABLE, &avail);
        if (avail) {
            GLuint64 ns = 0;
            glGetQueryObjectui64v(qMain[readIdx], GL_QUERY_RESULT, &ns);
            gpuMainMs = ns / 1e6;
        }

        // DEBUG
        glGetQueryObjectuiv(qDebug[readIdx], GL_QUERY_RESULT_AVAILABLE, &avail);
        if (avail) {
            GLuint64 ns = 0;
            glGetQueryObjectui64v(qDebug[readIdx], GL_QUERY_RESULT, &ns);
            gpuDebugMs = ns / 1e6;
        }

        // Endast om båda var tillgängliga (eller om du vill tillåta var för sig):
        // bumpa readIdx så vi läser nästa nästa gång
        if (avail) {
            readIdx = (readIdx + 1) % NQ;
        }

        glfwPollEvents();

        if (glfwWindowShouldClose(window)) {
            break;
        }

        // rendering
        float renderClockStart = static_cast<float>(glfwGetTime());
        renderer.update(camera, sceneBuilder, physicsEngine, editor, qShadow, qMain, qDebug, writeIdx);
        // calculate render time
        float renderClockEnd = static_cast<float>(glfwGetTime());
        float renderClockMs = (renderClockEnd - renderClockStart) * 1000.0f;
        sceneBuilder.sceneDirty = false;
        writeIdx = (writeIdx + 1) % NQ;

        // continous input 
        inputManager.processInput(window, deltaTime);

        // editor functions
        editor.update(deltaTime, renderer.debugShader);

        // physics step
        float stepClockStart = static_cast<float>(glfwGetTime());
        if (!engineState.isPaused() or engineState.getAdvanceStep()) 
        {
            const int   kMaxStepsPerFrame = 8;
            const float kMaxAccum = kMaxStepsPerFrame * fixedTimeStep;
            accumulator = std::min(accumulator + deltaTime, kMaxAccum);

            if (engineState.getAdvanceStep()) {
                accumulator = fixedTimeStep;
            }

            int steps = 0;
            while (accumulator >= fixedTimeStep and steps < kMaxStepsPerFrame) 
            {
                for (SceneBuilder::Halo& halo : sceneBuilder.allHalos)
                {
                    // per-frame rotation (du har redan denna)
                    glm::quat perFrameRot = glm::angleAxis(glm::radians(halo.rotSpeed) * fixedTimeStep, glm::normalize(halo.rotDir));

                    for (int i = 0; i < halo.indices.size(); i++) {
                        GameObject& obj = sceneBuilder.getDynamicObjects()[halo.indices[i]];

                        // --- 1) Spara föregående pose ---
                        glm::vec3 prevPos = obj.position;
                        glm::quat prevOri = obj.orientation;

                        // --- 2) Sätt ny pose kinematiskt (din kod) ---
                        glm::vec3 relativePos = obj.position - halo.center;
                        glm::vec3 rotatedRelPos = perFrameRot * relativePos;

                        obj.position = halo.center + rotatedRelPos;
                        obj.orientation = perFrameRot * obj.orientation;

                        // --- 3) Härled hastigheter från faktisk förflyttning denna frame ---
                        // Linjär hastighet: differens
                        obj.linearVelocity = (obj.position - prevPos) / fixedTimeStep;

                        // Vinkelhastighet: delta-quat → (axis, theta) → ω = axis * (theta/dt)
                        glm::quat dq = obj.orientation * glm::conjugate(prevOri);  // "rotationen som hände i frame:n"
                        dq = glm::normalize(dq);
                        float cosHalf = glm::clamp(dq.w, -1.0f, 1.0f);
                        float theta = 2.0f * std::acos(cosHalf);                  // rad per frame
                        float s = std::sqrt(std::max(0.0f, 1.0f - cosHalf * cosHalf));
                        glm::vec3 axis = (s > 1e-8f) ? glm::vec3(dq.x, dq.y, dq.z) / s
                            : glm::vec3(0, 0, 1);           // fallback
                        obj.angularVelocity = axis * (theta / fixedTimeStep);           // rad/s

                        // --- 4) Uppdatera render/collider ---
                        obj.modelMatrixDirty = true;
                        obj.setModelMatrix();
                        obj.setHelperMatrices();
                        obj.updateAABB();
                        obj.updateCollider();
                    }
                }
                physicsEngine.step(fixedTimeStep, rng);

                steps++;
                accumulator -= fixedTimeStep;
            }
        }

        if (engineState.getAdvanceStep()) {
            engineState.setAdvanceStep(false);
        }

        // calculate step time
        float stepClockStop = static_cast<float>(glfwGetTime());
        float stepClockMs = (stepClockStop - stepClockStart) * 1000.0f;

        if (!engineState.isPaused()) {
            if (editor.objectRainBlocks) sceneBuilder.objectRain(currentFrame, rng, 0);
            else if (editor.objectRainSpheres) sceneBuilder.objectRain(currentFrame, rng, 1);
        }

        // swap buffers
        float t1 = static_cast<float>(glfwGetTime());
        glfwSwapBuffers(window);
        float t2 = static_cast<float>(glfwGetTime());
        float swapTime = (t2 - t1);

        // calculate CPU time
        float cpuClockStop = static_cast<float>(glfwGetTime());
        float cpuClockMs = (cpuClockStop - cpuClockStart) * 1000.0f;

        // print FPS and other stats
        if (engineState.getShowFPS()) {
            float seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
            if (seconds > last_second) {
                last_second = seconds;
                const int LABEL_W = 11;
                const int VALUE_W = 0;

                std::cout
                    // FPS
                    << std::setw(LABEL_W) << std::left
                    << "FPS:"
                    // värdefält, vänster-justerat
                    << std::setw(VALUE_W) << std::left
                    << frames << "\n"

                    // Objects
                    << std::setw(LABEL_W) << std::left
                    << "Objects:"
                    << std::setw(VALUE_W) << std::left
                    << sceneBuilder.getDynamicObjects().size()
                    << std::defaultfloat << "\n"

                    << "------" << "\n"

                    << std::setw(LABEL_W) << std::left
                    << "GPU:"
                    << std::setw(VALUE_W) << std::left
                    << std::fixed << std::setprecision(1) << (gpuShadowMs + gpuMainMs + gpuDebugMs) << " ms\n"
                    // GPU Shadow
                    << std::setw(LABEL_W) << std::left
                    << "Shadow:"
                    << std::setw(VALUE_W) << std::left
                    << std::fixed << gpuShadowMs << " ms\n"
                    // GPU Main
                    << std::setw(LABEL_W) << std::left
                    << "Main:"
                    << std::setw(VALUE_W) << std::left
                    << std::fixed << gpuMainMs << " ms\n"
                    // GPU Debug
                    << std::setw(LABEL_W) << std::left
                    << "Debug:"
                    << std::setw(VALUE_W) << std::left
                    << std::fixed << gpuDebugMs << " ms\n"

                    << "------" << "\n"

                    // CPU
                    << std::setw(LABEL_W) << std::left
                    << "CPU:"
                    // värdefält, vänster-justerat
                    << std::setw(VALUE_W) << std::left
                    << std::fixed << cpuClockMs << " ms\n"

                    // Physics
                    << std::setw(LABEL_W) << std::left
                    << "Physics:"
                    << std::setw(VALUE_W) << std::left
                    << stepClockMs << " ms\n"

                    // Render
                    << std::setw(LABEL_W) << std::left
                    << "Render:"
                    << std::setw(VALUE_W) << std::left
                    << renderClockMs << " ms\n";

                    //// BVH rebuilds
                    //<< std::setw(LABEL_W) << std::left
                    //<< "BVH:"
                    //<< std::setw(VALUE_W) << std::left
                    //<< physicsEngine.getDynamicAwakeBvh().numRebuilds << "\n"

                    // opengl
                    //<< std::setw(LABEL_W) << std::left
                    //<< "Opengl:"
                    //<< std::setw(VALUE_W) << std::left;
                    //glcount::print();
                    //std::cout << std::defaultfloat << "\n";
           

                if (deltaTime > 0.016f) {
                    std::cout << "-- Warning: deltaTime is larger than 1/60 -- " << "\n";
                }
                std::cout << "===============================" << "\n";

                frames = 0;
                physicsEngine.getDynamicAwakeBvh().numRebuilds = 0;

                std::cout << std::setprecision(9);
            }
            frames++;
        }
   }
   glfwTerminate();

   return 0;
}