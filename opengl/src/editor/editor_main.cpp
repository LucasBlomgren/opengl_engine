#include "pch.h"
#include "editor_main.h"

#include "imgui.h"
#include "input_manager.h"
#include "scene_builder.h"
#include "physics.h"
#include "game_object.h"
#include "aabb.h"
#include "shaders/shader.h"
#include "physics/raycast.h"

#include <glm/gtc/random.hpp>

// -----------------------------------
//      Main functions
// -----------------------------------
void Editor::EditorMain::init(
    float SCR_WIDTH,
    float SCR_HEIGHT,
    EngineState* engineState,
    SceneBuilder* sceneBuilder,
    PhysicsEngine* physicsEngine,
    InputManager* inputManager,
    Camera* camera,
    GLFWwindow* window,
    ImGuiManager* imguiManager,
    Renderer* renderer,
    SkyboxManager* skyboxManager,
    MeshManager* meshManager,
    TextureManager* textureManager,
    FrameTimers* frameTimers,
    GpuTimers* gpuTimers
) {
    this->SCR_WIDTH = SCR_WIDTH;
    this->SCR_HEIGHT = SCR_HEIGHT;
    this->engineState = engineState;
    this->sceneBuilder = sceneBuilder;
    this->physicsEngine = physicsEngine;
    this->inputManager = inputManager;
    this->skyboxManager = skyboxManager;
    this->imguiManager = imguiManager;
    this->meshManager = meshManager;
    this->textureManager = textureManager;
    this->camera = camera;
    this->window = window;

    panelManager = std::make_unique<Editor::PanelManager>(
        imguiManager,
        engineState,
        renderer,  
        sceneBuilder,
        skyboxManager,
        meshManager,
        textureManager,
        frameTimers,
        gpuTimers
    );
    panelManager->init();
}


// activate/deactivate editor mode
void Editor::EditorMain::activate() {

}
void Editor::EditorMain::deactivate() {
    dropObject();
    lastHitData = {};
}

void Editor::EditorMain::addInputRouter(InputRouter& router) {
    router.add(this);
}

// -----------------------------------
//        Input handling
// -----------------------------------
void Editor::EditorMain::handleInput(const InputFrame& in, const InputContext& ctx, Consumed& consumed, FrameWants& wants) {
    if (ctx.isPlayerMode) return;

    // LMB/RMB drag capture only if started in viewport
    if (!viewportCapturedLMB and ctx.viewportHovered and in.mousePressed[GLFW_MOUSE_BUTTON_1])
        viewportCapturedLMB = true;
    if (viewportCapturedLMB and !in.mouseDown[GLFW_MOUSE_BUTTON_1])
        viewportCapturedLMB = false;

    if (!viewportCapturedRMB and ctx.viewportHovered and in.mousePressed[GLFW_MOUSE_BUTTON_2])
        viewportCapturedRMB = true;
    if (viewportCapturedRMB and !in.mouseDown[GLFW_MOUSE_BUTTON_2])
        viewportCapturedRMB = false;

    // handle non-viewport input capture
    ImGuiIO* io = &ImGui::GetIO();
    if (io->WantCaptureMouse and !(viewportCapturedLMB or viewportCapturedRMB)) {
        wants.cameraLook = false;
        wants.captureMouse = false;
        consumed.mouse = true;
    }
    if (io->WantCaptureKeyboard and !ctx.viewportFocused) {
        consumed.keyboard = true;
    }

    // handle viewport RMB camera look
    if (viewportCapturedRMB) {
        wants.cameraLook = true;
        wants.captureMouse = true;
    }

    // handle editor input
    if (!consumed.mouse) {
        if (in.mousePressed[GLFW_MOUSE_BUTTON_1]) {
            if (selectedObject == nullptr) {
                rayCast(SELECT_RANGE);
                selectObject(ctx);
            }
            else {
                RaycastHit hitData = rayCast(SELECT_RANGE);
                if (hitData.object == nullptr) {
                    dropObject();
                }
                else if (hitData.object != selectedObject) {
                    dropObject();
                    rayCast(SELECT_RANGE);
                    selectObject(ctx);
                }
                else {
                    syncSelectionOffset(); // klick på samma objekt: resync offset så det inte hoppar
                }
            }
            consumed.mouse = true;
        }

        if (in.mouseDown[GLFW_MOUSE_BUTTON_1]) {
            updateSelectedObject(1.0f / 60.0f); // hardcoded timestep for smoother movement
            consumed.mouse = true;
        }

        if (in.mouseDown[GLFW_MOUSE_BUTTON_2]) {
            wants.cameraLook = true;
            wants.captureMouse = true;
            consumed.mouse = true;
        }


        static float shootCd = 0.0f;
        shootCd -= engineState->deltaTime;
        if (in.mouseDown[GLFW_MOUSE_BUTTON_3] and shootCd <= 0.0f) {
            shootCd = 0.001f;

            static int shot = 0;
            int i = shot++ % 10;
            int x = i % 5, y = i / 5;

            glm::vec3 right = glm::normalize(glm::cross(camera->front, camera->up));
            float spread = 2.0f;
            float jitter = 1.4f;
            glm::vec3 offset = right * ((x - 2) * spread) + camera->up * ((y - 0.5f) * spread)
                + right * (glm::linearRand(-jitter, jitter))
                + camera->up * (glm::linearRand(-jitter, jitter));

            auto& newObject = sceneBuilder->createObject(
                "crate", "cube", ColliderType::CUBOID,
                camera->position + camera->front * 5.0f + offset,
                glm::vec3(1), 100, 0
            );
            newObject.linearVelocity = camera->front * SHOOT_VELOCITY;
        }

    }

    if (!consumed.keyboard) {
        if (in.keyPressed[GLFW_KEY_1]) {
            drawPlacementAABB = !drawPlacementAABB;
            consumed.keyboard = true;
        }

        if (in.keyPressed[GLFW_KEY_2]) {
            this->objectRainBlocks = !this->objectRainBlocks;
            objectRainPos = camera->position + camera->front * OBJ_PLACE_DISTANCE;
            consumed.keyboard = true;
        }
        if (in.keyPressed[GLFW_KEY_3]) {
            this->objectRainSpheres = !this->objectRainSpheres;
            objectRainPos = camera->position + camera->front * OBJ_PLACE_DISTANCE;
            consumed.keyboard = true;
        }

        if (in.keyPressed[GLFW_KEY_4]) {
            physicsEngine->sleepAllObjects();
            consumed.keyboard = true;
        }
        if (in.keyPressed[GLFW_KEY_5]) {
            physicsEngine->awakenAllObjects();
            consumed.keyboard = true;
        }
    }
}

// -----------------------------------
//        UI rendering
// -----------------------------------
void Editor::EditorMain::drawUI(InputContext& ctx)
{
    panelManager->renderPanels();

    // top toolbar
    ImGui::Begin("Toolbar1");
    ImGui::End();

    // bottom toolbar
    ImGui::Begin("Toolbar2");
    ImGui::End();

    // Viewport panel
    ImGui::Begin("Scene", nullptr);

    ImGuiStyle& style = ImGui::GetStyle();
    ImVec4* c = style.Colors;
    c[ImGuiCol_Tab] = ImVec4(0.15f, 0.15f, 0.15f, 1.0f);
    c[ImGuiCol_TabHovered] = ImVec4(0.25f, 0.25f, 0.25f, 1.0f);
    c[ImGuiCol_TabActive] = ImVec4(0.20f, 0.20f, 0.20f, 1.0f);
    c[ImGuiCol_TabUnfocused] = ImVec4(0.12f, 0.12f, 0.12f, 1.0f);
    c[ImGuiCol_TabUnfocusedActive] = ImVec4(0.18f, 0.18f, 0.18f, 1.0f);
    c[ImGuiCol_TitleBg] = ImVec4(0.10f, 0.10f, 0.10f, 1.0f);
    c[ImGuiCol_TitleBgActive] = ImVec4(0.12f, 0.12f, 0.12f, 1.0f);
    c[ImGuiCol_TitleBgCollapsed] = ImVec4(0.10f, 0.10f, 0.10f, 1.0f);

    style.TabRounding = 0.0f; // tab corners
    style.WindowRounding = 0.0f; // fönster corners
    style.FrameRounding = 0.0f; // knappar/sliders etc
    style.ChildRounding = 0.0f; // child windows
    style.PopupRounding = 0.0f; // popups

    ImVec2 avail = ImGui::GetContentRegionAvail();
    int w = std::max(1, (int)std::round(avail.x));
    int h = std::max(1, (int)std::round(avail.y));
    viewportFBO.resizeIfNeeded(w, h);

    ImGui::Image((ImTextureID)(intptr_t)viewportFBO.colorTex, ImVec2((float)w, (float)h), ImVec2(0, 1), ImVec2(1, 0));

    // 1) Hämta position och storlek på bilden i fönstret
    ImVec2 image_pos = ImGui::GetItemRectMin(); // top-left på imagen
    ImVec2 image_max = ImGui::GetItemRectMax(); // bottom-right
    ImVec2 image_size = ImVec2(image_max.x - image_pos.x, image_max.y - image_pos.y);

    // 2) Hämta muspositionen i skärmens koordinater
    ImVec2 mouse = ImGui::GetMousePos();

    // 3) Mus relativt bilden
    ImVec2 mouse_in_image = ImVec2(mouse.x - image_pos.x, mouse.y - image_pos.y);

    // 4) insideImage = musen ligger inom bildens rektangel
    bool insideImage =
        mouse_in_image.x >= 0.0f and mouse_in_image.y >= 0.0f and
        mouse_in_image.x < image_size.x and mouse_in_image.y < image_size.y;

    ctx.viewportHovered = insideImage;
    ctx.viewportFocused = ImGui::IsWindowFocused();

    // spara lokala coords för raycast
    viewportMouseX = mouse_in_image.x;
    viewportMouseY = mouse_in_image.y;
    viewportDisplayW = std::max(image_size.x, 1.0f);
    viewportDisplayH = std::max(image_size.y, 1.0f);

    ImGui::End();
}

// -----------------------------------
//        Update 
// -----------------------------------
void Editor::EditorMain::fixedUpdate(float fixedTimeStep) {
    //updateSelectedObject(fixedTimeStep);
}

void Editor::EditorMain::update(Shader& shader) {
    // raycast and draw placement AABB
    createPlaceObjectAABB(shader);

    hoveredObject = nullptr;

    // clear previous hover state
    if (lastHitData.object != nullptr)
        lastHitData.object->hoveredByEditor = false;

    // raycast for hover
    rayCast(SELECT_RANGE);

    // set new hover state
    if (lastHitData.object != nullptr) {
        lastHitData.object->hoveredByEditor = true;
        hoveredObject = lastHitData.object;
    }
}

// -----------------------------------
//      Selection 
// -----------------------------------
void Editor::EditorMain::syncSelectionOffset() {
    if (!selectedObject) return;

    glm::vec3 worldOffset = selectedObject->position - camera->position;
    selectionOffsetLocal.x = glm::dot(worldOffset, camera->right);
    selectionOffsetLocal.y = glm::dot(worldOffset, camera->up);
    selectionOffsetLocal.z = glm::dot(worldOffset, camera->front);

    selectedObject->lastPosition = selectedObject->position;
}

// select object
void Editor::EditorMain::selectObject(const InputContext& ctx) {
    if (selectedObject) return;

    RaycastHit& hitData = lastHitData;

    // no hit return
    if (hitData.object == nullptr) {
        return;
    }
    selectedObject = hitData.object;

    selectedObject->selectedByEditor = true;
    selectedObject->hoveredByEditor = false;
    selectedObject->lastPosition = selectedObject->position;

    selectedObject->linearVelocity = glm::vec3(0.0f);
    selectedObject->angularVelocity = glm::vec3(0.0f);

    glm::vec3 worldOffset = selectedObject->position - camera->position;
    // Projicera worldOffset på kamerans lokala axlar:
    selectionOffsetLocal.x = glm::dot(worldOffset, camera->right);
    selectionOffsetLocal.y = glm::dot(worldOffset, camera->up);
    selectionOffsetLocal.z = glm::dot(worldOffset, camera->front);

    panelManager->ctx.selectedObject = selectedObject;
}

// update selected object position based on camera and offset
void Editor::EditorMain::updateSelectedObject(float fixedTimeStep) {
    if (!selectedObject or selectedObject->isStatic)
        return;

    glm::vec3 worldOffset = camera->right * selectionOffsetLocal.x + camera->up * selectionOffsetLocal.y + camera->front * selectionOffsetLocal.z;

    // position
    glm::vec3 newPos = camera->position + worldOffset;
    selectedObject->position = newPos;

    // velocity
    //selectedObject->linearVelocity = (newPos - selectedObject->lastPosition) / fixedTimeStep;

    selectedObject->lastPosition = newPos;

    selectedObject->modelMatrixDirty = true;
    selectedObject->aabbDirty = true;
    selectedObject->setModelMatrix();
    selectedObject->updateAABB();
    selectedObject->updateCollider();
}

// drop selected object
void Editor::EditorMain::dropObject() {
    if (selectedObject) {
        selectedObject->selectedByEditor = false;

        // avoid nullptr dereference in physics engine
        GameObject* obj = selectedObject;

        if (selectedObject->isStatic) {
            BroadphaseBucket target = BroadphaseBucket::Static;
            physicsEngine->queueMove(obj, target);
        }
        else {
            BroadphaseBucket target = BroadphaseBucket::Awake;
            physicsEngine->queueMove(obj, target);
        }

        selectedObject->sleepCounter = 0.0f;
        selectedObject->sleepCounterThreshold = 1.5f;
        //selectedObject->linearVelocity = glm::vec3(0.0f);
        //selectedObject->angularVelocity = glm::vec3(0.0f);
        selectedObject = nullptr;
        hoveredObject = nullptr;
    }

    selectedObject = nullptr;
    hoveredObject = nullptr;
    panelManager->ctx.selectedObject = nullptr;
}

// place object at placement AABB position
void Editor::EditorMain::placeObject() {
    if (placementObstructed)
        return;

    glm::vec3 size{ OBJ_PLACE_SIZE };
    glm::vec3 spawnPos = aabbToPlace.centroid;
    glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(0.0f, 0.0f, 0.0f));
    GameObject& newObject = sceneBuilder->createObject("crate", "cube", ColliderType::CUBOID, spawnPos, size, 1, 0, orientation, 1.5f, 0);
    newObject.linearVelocity = glm::vec3(0.0f);
}

// get raycast
RaycastHit& Editor::EditorMain::getLastRayHit() {
    return lastHitData;
}
// Raycast from camera through mouse cursor into world
RaycastHit Editor::EditorMain::rayCast(float length)
{
    // 0) Normalisera musen i viewport-bilden (display space)
    float u = viewportMouseX / viewportDisplayW;
    float v = viewportMouseY / viewportDisplayH;

    // clamp för säkerhet
    u = std::clamp(u, 0.0f, 1.0f);
    v = std::clamp(v, 0.0f, 1.0f);

    // 1) u,v -> NDC
    float x = u * 2.0f - 1.0f;
    float y = 1.0f - v * 2.0f; // invert Y (top-left -> bottom-left)
    glm::vec4 rayClip(x, y, -1.0f, 1.0f);

    // 2) Clip → Eye(inverse projection)
    float aspect = (float)viewportFBO.width / (float)viewportFBO.height;
    glm::mat4 projection = glm::perspective(glm::radians(camera->zoom), aspect, 0.1f, 10000.0f);
    glm::mat4 view = camera->GetViewMatrix();
    glm::vec4 rayEye = glm::inverse(projection) * rayClip;
    // Vi vill ha en riktning, inte en punkt:
    rayEye = glm::vec4(rayEye.x, rayEye.y, -1.0f, 0.0f);

    // 3) Eye → World(inverse view) och normalisera
    glm::vec3 rayWorld = glm::normalize(glm::vec3(glm::inverse(view) * rayEye));

    // 4) Bygg rayen
    glm::vec3 origin = camera->position;
    glm::vec3 dir = rayWorld;

    float rLength = length;
    Ray r(camera->position, rayWorld, SELECT_RANGE);
    RaycastHit hitData = physicsEngine->performRaycast(r);

    lastHitData = hitData;
    return hitData;
}

// create placement AABB in front of camera, adjusted for collisions
void Editor::EditorMain::createPlaceObjectAABB(Shader& shader) {
    glm::vec3 size{ OBJ_PLACE_SIZE };

    AABB aabb;
    aabb.centroid = camera->position + camera->front * OBJ_PLACE_DISTANCE;
    aabb.halfExtents = glm::vec3(size / 2.0f);

    RaycastHit hitData = rayCast(OBJ_PLACE_DISTANCE);
    glm::vec3 normal = hitData.normal;
    if (hitData.object != nullptr) {
        if (glm::dot(hitData.normal, camera->front) > 0.0f)
            normal = -normal;

        glm::vec3 absN = glm::abs(normal);
        float extentOnN = glm::dot(aabb.halfExtents, absN);

        float margin = 0.1f;
        aabb.centroid = hitData.point + normal * (extentOnN - margin);
    }

    aabb.wMin = aabb.centroid - aabb.halfExtents;
    aabb.wMax = aabb.centroid + aabb.halfExtents;

    const BVHTree<GameObject>& dynamicAwakeBvh = physicsEngine->getDynamicAsleepBvh();
    int maxIter = 8;
    int iter = 0;
    for (int i = 0; i < maxIter; i++) {
        std::vector<GameObject*> collisions;
        collisions.reserve(100);
        dynamicAwakeBvh.singleQuery(aabb, collisions);

        if (collisions.size() == 0) {
            break;
        }

        // min depth collision
        float min = std::numeric_limits<float>::max();
        GameObject* minDepthObj = nullptr;
        for (int j = 0; j < collisions.size(); j++) {
            GameObject& objB = *collisions[j];

            float depth = aabb.getMinOverlapDepth(objB.aabb);
            if (depth < min) {
                min = depth;
                minDepthObj = &objB;
            }
        }

        // move AABB away from collision
        glm::vec3 normal = aabb.getCollisionNormal(minDepthObj->aabb);
        aabb.centroid += normal * min * 1.2f;
        aabb.wMin = aabb.centroid - aabb.halfExtents;
        aabb.wMax = aabb.centroid + aabb.halfExtents;

        iter++;
    }

    glm::vec3 color;
    if (iter >= maxIter) {
        this->placementObstructed = true;
        color = glm::vec3{ 1,0,0 };
    }
    else {
        this->placementObstructed = false;
        aabbToPlace = aabb;
        color = glm::vec3{ 0.9f, 0.7f, 0.2f };
    }

    if (drawPlacementAABB)
        drawAABB(aabb, shader, color);
}

// draw selection/placement AABB
void Editor::EditorMain::drawAABB(const AABB& aabb, Shader& shader, glm::vec3 color) {
    glm::vec3 min = aabb.wMin;
    glm::vec3 max = aabb.wMax;

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
        glGenVertexArrays(1, &vao); glcount::incVAO();
        glGenBuffers(1, &vbo); glcount::incVBO();
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);
    }

    shader.use();
    shader.setMat4("model", glm::mat4(1.0f));
    shader.setBool("debug.useUniformColor", true);
    shader.setVec3("debug.uColor", color);
  
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(buf), buf.data(), GL_DYNAMIC_DRAW);

    glLineWidth(2.0f);
    glDrawArrays(GL_LINES, 0, 24);
}