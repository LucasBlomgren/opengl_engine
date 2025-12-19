
#include "pch.h"

#include "init_opengl.h"
#include "timer.h"
#include "engine_state.h"
#include "input_manager.h"
#include "camera.h"
#include "physics.h"
#include "scene_builder.h"
#include "renderer/renderer.h"
#include "shaders/shader.h"
#include "textures/texture_manager.h"
#include "mesh/mesh_manager.h"
#include "shaders/shader_manager.h"
#include "skybox/skybox_manager.h"
#include "imgui_manager.h"

#include "game_object.h"
#include "lighting/light.h"
#include "lighting/light_manager.h"
#include "lighting/shadow_manager.h"
#include "editor.h"

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
const unsigned int SHADOW_WIDTH = 4096;
const unsigned int SHADOW_HEIGHT = 4096;

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
    GpuTimers gpu;

    // setup rng
    std::mt19937 rng(std::random_device{}());

    // systems
    EngineState engineState;
    FrameTimers frameTimers;
    PhysicsEngine physicsEngine(&frameTimers);
    Renderer renderer;

    // managers
    ImGuiManager imguiManager;
    InputManager inputManager;
    TextureManager textureManager;
    MeshManager meshManager;
    ShaderManager shaderManager;
    SkyboxManager skyboxManager; 
    LightManager lightManager;
    ShadowManager shadowManager(SHADOW_WIDTH, SHADOW_HEIGHT); 

    // view
    Camera camera(glm::vec3(-70.0f, 40.0f, -30.0f));
    // set initial rotation
    camera.yaw = 35.0f;
    camera.pitch = -20.0f;
    camera.ProcessMouseMovement(0.0f, 0.0f);

    // editor
    Editor editor;
    SceneBuilder sceneBuilder(physicsEngine, renderer, textureManager, meshManager, shaderManager, lightManager, rng);

    // fixed timestep
    const float fixedTimeStep = 1.0f / 60.0f;
    float accumulator = 0.0f;

    // setup input
    inputManager.setPointers(&engineState, &camera);
    inputManager.init(window);

    // setup rendering
    renderer.init(SCR_WIDTH, SCR_HEIGHT, editor, engineState, lightManager, shaderManager, shadowManager, skyboxManager);

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

    // setup scene 
    sceneBuilder.createScene(6);

    // setup editor
    editor.setPointers(window, &inputManager, &engineState, &sceneBuilder, &physicsEngine, &camera, &skyboxManager);

    // ImGui setup
    imguiManager.init(window, engineState, sceneBuilder, meshManager, renderer, textureManager);

    // main loop
    while (true) {

        // timing
        frameTimers.beginFrame();
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // events
        glfwPollEvents();

        // exit condition
        if (glfwWindowShouldClose(window)) {
            break;
        }

        // start ImGui frame
        imguiManager.newFrame();
        imguiManager.setInputMode(engineState.getCameraMode());

        // read GPU timers from previous frames
        GLuint avail = 0;
        // SHADOW
        glGetQueryObjectuiv(qShadow[readIdx], GL_QUERY_RESULT_AVAILABLE, &avail);
        if (avail) {
            GLuint64 ns = 0;
            glGetQueryObjectui64v(qShadow[readIdx], GL_QUERY_RESULT, &ns); // blockar inte när avail==true
            gpu.shadowMs = ns / 1e6;
        }
        // MAIN
        glGetQueryObjectuiv(qMain[readIdx], GL_QUERY_RESULT_AVAILABLE, &avail);
        if (avail) {
            GLuint64 ns = 0;
            glGetQueryObjectui64v(qMain[readIdx], GL_QUERY_RESULT, &ns);
            gpu.mainMs = ns / 1e6;
        }
        // DEBUG
        glGetQueryObjectuiv(qDebug[readIdx], GL_QUERY_RESULT_AVAILABLE, &avail);
        if (avail) {
            GLuint64 ns = 0;
            glGetQueryObjectui64v(qDebug[readIdx], GL_QUERY_RESULT, &ns);
            gpu.debugMs = ns / 1e6;
        }
        // Endast om båda var tillgängliga (eller om du vill tillåta var för sig):
        // bumpa readIdx så vi läser nästa nästa gång
        if (avail) {
            readIdx = (readIdx + 1) % NQ;
        }


        // rendering
        {
            ScopedTimer t(frameTimers, "Render");
            renderer.render(camera, sceneBuilder, physicsEngine, qShadow, qMain, qDebug, writeIdx);
            sceneBuilder.sceneDirty = false;
            writeIdx = (writeIdx + 1) % NQ;
        }

        editor.cameraMode();
        // input & editor update
        if (engineState.getCameraMode()) {
            inputManager.processInput(window, deltaTime);

            // editor update
            {
                ScopedTimer t(frameTimers, "Editor");
                editor.update(deltaTime, *renderer.debugShader);
            }
        }

        // physics step
        if (!engineState.isPaused() or engineState.getAdvanceStep())
        {
            const int   kMaxStepsPerFrame = 8;
            const float kMaxAccum = kMaxStepsPerFrame * fixedTimeStep;
            accumulator = std::min(accumulator + deltaTime, kMaxAccum);

            // single step advance
            if (engineState.getAdvanceStep()) {
                accumulator = fixedTimeStep;
            }

            // stepping loop
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

        // single step advance flag reset
        if (engineState.getAdvanceStep()) {
            engineState.setAdvanceStep(false);
        }

        // object rain
        if (!engineState.isPaused()) {
            if (editor.objectRainBlocks) sceneBuilder.objectRain(currentFrame, 0);
            else if (editor.objectRainSpheres) sceneBuilder.objectRain(currentFrame, 1);
        }

        // ImGui rendering
        {
            ScopedTimer a(frameTimers, "ImGui");
            imguiManager.mainUI(deltaTime, frameTimers, gpu, sceneBuilder.getDynamicObjects().size());
            imguiManager.selectedObjectUI(editor.selectedObject);
            imguiManager.render();
        }

        frameTimers.endFrame();

        // swap buffers
        glfwSwapBuffers(window);
   }

   imguiManager.shutdown();
   glfwTerminate();

   return 0;
}