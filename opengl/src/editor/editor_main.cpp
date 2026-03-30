#include "pch.h"
#include "editor_main.h"

#include "imgui.h"
#include "input_manager.h"
#include "scene_builder.h"
#include "physics.h"
#include "graphics/renderer/renderer.h"
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
    World* world,
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
    this->world = world;
    this->sceneBuilder = sceneBuilder;
    this->physicsEngine = physicsEngine;
    this->renderer = renderer;
    this->inputManager = inputManager;
    this->skyboxManager = skyboxManager;
    this->imguiManager = imguiManager;
    this->meshManager = meshManager;
    this->textureManager = textureManager;
    this->camera = camera;
    this->window = window;

    panelManager = std::make_unique<Editor::PanelManager>(
        physicsEngine,
        imguiManager,
        engineState,
        renderer,  
        world,
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

    // clear hover/selection states on objects to avoid stuck states
    if (objectIsSelected) {
        GameObject* obj = world->getGameObject(selectedObjectHandle);
        if (obj) {
            obj->selectedByEditor = false;
        }
        objectIsSelected = false;
        selectedObjectHandle = GameObjectHandle{};
    }

    if (objectIsHovered) {
        GameObject* obj = world->getGameObject(hoveredObjectHandle);
        if (obj) {
            obj->hoveredByEditor = false;
        }
        objectIsHovered = false;
        hoveredObjectHandle = GameObjectHandle{};
    }

    selectedObjectIsBeingMoved = false;
}

// -----------------------------------
//        Input handling
// -----------------------------------
void Editor::EditorMain::handleInput(const InputFrame& in, const InputContext& ctx, Consumed& consumed, FrameWants& wants) {
    if (ctx.isPlayerMode) return;

    if (in.keyPressed[GLFW_KEY_F1]) {
        flag_drawUI = !flag_drawUI;
    }

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
        mouseXLastFrame = in.mousePos.x;
        mouseYLastFrame = in.mousePos.y;

        if (in.mousePressed[GLFW_MOUSE_BUTTON_1]) {
            if (!objectIsSelected) {
                rayCast(SELECT_RANGE);
                selectObject(ctx);
            }
            else {
                RaycastHit hitData = rayCast(SELECT_RANGE);

                if (!hitData.hit) {
                    dropObject();
                }
                else {
                    Collider* hitCollider = world->getCollider(hitData.colliderHandle);
                    GameObjectHandle hitObjHandle = hitCollider->gameObjectHandle;
                    if (hitObjHandle.slot != selectedObjectHandle.slot) {
                        dropObject();
                        rayCast(SELECT_RANGE);
                        selectObject(ctx);
                    }
                    else {
                        syncSelectionOffset(); // klick på samma objekt: resync offset så det inte hoppar
                    }
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

        selectedObjectIsBeingMoved = false;
        if (in.mouseDown[GLFW_MOUSE_BUTTON_1] && in.mouseDown[GLFW_MOUSE_BUTTON_2] && objectIsSelected) {
            selectedObjectIsBeingMoved = true;
        }

        static bool wasDown = false;
        static float shootTimer = 0.0f;

        const float fireInterval = 0.001f; // 1000/s

        bool down = in.mouseDown[GLFW_MOUSE_BUTTON_3];

        if (down && !wasDown) {
            // Första pressen: starta “nu” utan att försöka hinna ikapp något gammalt
            shootTimer = fireInterval;
        }

        if (down) {
            shootTimer -= engineState->deltaTime;

            int spawnedThisFrame = 0;
            const int maxPerFrame = 50; // säkerhetscap

            while (shootTimer <= 0.0f && spawnedThisFrame < maxPerFrame) {
                shootTimer += fireInterval; // håll konstant takt
                spawnedThisFrame++;

                // spawn
                static int shot = 0;
                int i = shot++ % 10;
                int x = i % 5, y = i / 5;

                glm::vec3 right = glm::normalize(glm::cross(camera->front, camera->up));
                float spread = 2.0f;
                float jitter = 1.4f;
                glm::vec3 offset = right * ((x - 2) * spread) + camera->up * ((y - 0.5f) * spread)
                    + right * (glm::linearRand(-jitter, jitter))
                    + camera->up * (glm::linearRand(-jitter, jitter));

                GameObjectHandle newObject = world->createGameObject(
                    "uvmap", "sphere", ColliderType::SPHERE, BodyType::Dynamic,
                    camera->position + camera->front * 5.0f + offset,
                    glm::vec3(1), 1, {}, 1.5f, false, {}, false
                );

                GameObject* objPtr = world->getGameObject(newObject);
                RigidBody* rb = world->getRigidBody(newObject);
                rb->linearVelocity = camera->front * SHOOT_VELOCITY;
            }
        }
        else {
            shootTimer = 0.0f;
        }

        wasDown = down;
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


        if (in.keyDown[GLFW_KEY_C]) {
            static size_t nextIndex = 0;

            SlotMap<GameObject, GameObjectHandle>& map = world->getGameObjectsMap();
            auto& objects = map.dense();

            int removed = 0;
            size_t checked = 0;

            while (!objects.empty() && checked < objects.size() && removed < 10) {
                if (nextIndex >= objects.size()) {
                    nextIndex = 0;
                }

                GameObjectHandle handle = map.handle_from_dense_index((int)nextIndex);
                GameObject* obj = world->getGameObject(handle);

                nextIndex++;
                checked++;

                if (!obj) continue;

                physicsEngine->queueRemove(obj->colliderHandle);
                handlesToRemove.push_back(handle);

                std::cout << "Editor: Queued remove of handle slot "
                    << handle.slot << " gen " << handle.gen << std::endl;

                removed++;
            }
        }
    }
}

// -----------------------------------
//        UI rendering
// -----------------------------------
void Editor::EditorMain::drawUI(InputContext& ctx, float deltaTime)
{
    if (!flag_drawUI) return;

    panelManager->renderPanels(deltaTime);

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

    // clear previous hover state
    if (objectIsHovered) {
        GameObject* hoveredObj = world->getGameObject(hoveredObjectHandle);
        hoveredObj->hoveredByEditor = false;
        objectIsHovered = false;
    }

    // raycast for hover
    RaycastHit raycast = rayCast(SELECT_RANGE);

    // set new hover state
    if (raycast.hit && !selectedObjectIsBeingMoved) {
        GameObject* hoveredObj = world->getGameObject(raycast.colliderHandle);
        Collider* hoveredCollider = world->getCollider(raycast.colliderHandle);
        GameObjectHandle hoveredHandle = hoveredCollider->gameObjectHandle;
        hoveredObj->hoveredByEditor = true;
        hoveredObjectHandle = hoveredHandle;
        objectIsHovered = true;
    }
}

//------------------------------------------------------
// OBJECT SELECTION AND PLACEMENT
//------------------------------------------------------
void Editor::EditorMain::syncSelectionOffset() {
    if (!objectIsSelected) return;

    GameObject* selectedObject = world->getGameObject(selectedObjectHandle);
    glm::vec3 worldOffset = selectedObject->transform.position - camera->position;
    selectionOffsetLocal.x = glm::dot(worldOffset, camera->right);
    selectionOffsetLocal.y = glm::dot(worldOffset, camera->up);
    selectionOffsetLocal.z = glm::dot(worldOffset, camera->front);

    selectedObject->transform.lastPosition = selectedObject->transform.position;
}

// select object
void Editor::EditorMain::selectObject(const InputContext& ctx) {
    if (objectIsSelected) return;

    RaycastHit raycast = rayCast(SELECT_RANGE);

    // no hit return
    if (!raycast.hit) {
        return;
    }

    objectIsSelected = true;
    Collider* selectedCollider = world->getCollider(raycast.colliderHandle);
    GameObjectHandle selectedHandle = selectedCollider->gameObjectHandle;
    selectedObjectHandle = selectedHandle;

    // set selected and hovered states, and calculate selection offset
    GameObject* selectedObject = world->getGameObject(selectedObjectHandle);
    selectedObject->selectedByEditor = true;
    selectedObject->hoveredByEditor = false;
    selectedObject->transform.lastPosition = selectedObject->transform.position;

    // Project worldofset onto cameras local axes
    glm::vec3 worldOffset = selectedObject->transform.position - camera->position;
    selectionOffsetLocal.x = glm::dot(worldOffset, camera->right);
    selectionOffsetLocal.y = glm::dot(worldOffset, camera->up);
    selectionOffsetLocal.z = glm::dot(worldOffset, camera->front);

    // set body to kinematic so it can be moved without physics interference, and store original body type to restore later
    RigidBody* rb = world->getRigidBody(selectedObjectHandle);
    rb->setExternalControl(true);
    rb->linearVelocity = glm::vec3(0.0f);
    rb->angularVelocity = glm::vec3(0.0f);

    panelManager->ctx.selectedObjectHandle = selectedObjectHandle;
    panelManager->ctx.objectIsSelected = true;
}

// update selected object position based on camera and offset
void Editor::EditorMain::updateSelectedObject(float fixedTimeStep) {
    if (!objectIsSelected)
        return;

    GameObject* selectedObject = world->getGameObject(selectedObjectHandle);
    Collider* selectedCollider = world->getCollider(selectedObjectHandle);

    // position
    glm::vec3 worldOffset = camera->right * selectionOffsetLocal.x + camera->up * selectionOffsetLocal.y + camera->front * selectionOffsetLocal.z;
    glm::vec3 newPos = camera->position + worldOffset;
    selectedObject->transform.position = newPos;
    selectedObject->transform.lastPosition = newPos;

    // velocity
    //selectedObject->linearVelocity = (newPos - selectedObject->lastPosition) / fixedTimeStep;

    selectedObject->transform.updateCache();
    selectedCollider->updateAABB(selectedObject->transform);
    selectedCollider->updateCollider(selectedObject->transform);

    physicsEngine->setBVHDirty(selectedObject->colliderHandle);
}

// drop selected object
void Editor::EditorMain::dropObject() {
    if (objectIsSelected) {
        GameObject* selectedObject = world->getGameObject(selectedObjectHandle);
        selectedObject->selectedByEditor = false;

        RigidBody* rb = world->getRigidBody(selectedObjectHandle);
        rb->setExternalControl(false);

        objectIsSelected = false;
        objectIsHovered = false;
    }

    selectedObjectHandle = {};
    hoveredObjectHandle = {};
    panelManager->ctx.objectIsSelected = false;
    panelManager->ctx.selectedObjectHandle = {};
}

// place object at placement AABB position
void Editor::EditorMain::placeObject() {
    if (placementObstructed)
        return;

    glm::vec3 size{ OBJ_PLACE_SIZE };
    glm::vec3 spawnPos = aabbToPlace.centroid;
    glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(0.0f, 0.0f, 0.0f));
    GameObjectHandle handle = world->createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Dynamic, spawnPos, size, 1, orientation, 1.5f, false, {}, false);
    GameObject* newObject = world->getGameObject(handle);
    RigidBody* rb = world->getRigidBody(handle);
    rb->linearVelocity = glm::vec3(0.0f);
}

// Raycast from camera through mouse cursor into world
RaycastHit Editor::EditorMain::rayCast(float length)
{
    // 0) Normalisera musen i viewport-bilden (display space)
    float u = viewportMouseX / viewportDisplayW;
    float v = viewportMouseY / viewportDisplayH;

    if (!flag_drawUI) {
        // om UI inte ritas, använd global musposition och skärmstorlek (screen space)
        u = mouseXLastFrame / SCR_WIDTH;
        v = mouseYLastFrame / SCR_HEIGHT;
    }

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
    Ray r(camera->position, dir, SELECT_RANGE);
    RaycastHit hitData = physicsEngine->performRaycast(r);

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
    if (hitData.hit) {
        if (glm::dot(hitData.normal, camera->front) > 0.0f)
            normal = -normal;

        glm::vec3 absN = glm::abs(normal);
        float extentOnN = glm::dot(aabb.halfExtents, absN);

        float margin = 0.1f;
        aabb.centroid = hitData.point + normal * (extentOnN - margin);
    }

    aabb.wMin = aabb.centroid - aabb.halfExtents;
    aabb.wMax = aabb.centroid + aabb.halfExtents;

    const BVHTree& dynamicAwakeBvh = physicsEngine->getDynamicAsleepBvh();
    int maxIter = 8;
    int iter = 0;
    for (int i = 0; i < maxIter; i++) {
        std::vector<ColliderHandle> collisions;
        collisions.reserve(100);
        dynamicAwakeBvh.singleQuery(aabb, collisions);

        if (collisions.size() == 0) {
            break;
        }

        // min depth collision
        float min = std::numeric_limits<float>::max();
        Collider* minDepthCollider = nullptr;
        for (ColliderHandle& handle : collisions) {
            Collider* collider = world->getCollider(handle);

            float depth = aabb.getMinOverlapDepth(collider->aabb);
            if (depth < min) {
                min = depth;
                minDepthCollider = collider;
            }
        }

        // move AABB away from collision
        glm::vec3 normal = aabb.getCollisionNormal(minDepthCollider->aabb);
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