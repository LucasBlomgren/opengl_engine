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
    FrameTimers* frameTimers,
    GpuTimers* gpuTimers
) {
    this->SCR_WIDTH = SCR_WIDTH;
    this->SCR_HEIGHT = SCR_HEIGHT;
    this->engineState = engineState;
    this->sceneBuilder = sceneBuilder;
    this->physicsEngine = physicsEngine;
    this->inputManager = inputManager;
    this->camera = camera;
    this->window = window;

    panelManager = std::make_unique<Editor::PanelManager>(
        imguiManager,
        engineState,
        renderer,
        sceneBuilder,
        frameTimers,
        gpuTimers
    );
    panelManager->init();
}

void Editor::EditorMain::addInputRouter(InputRouter& router) {
    router.add(this);
}

void Editor::EditorMain::handleInput(const InputFrame& in, const InputContext& ctx, Consumed& c, FrameWants& wants) {
    if (ctx.isPlayerMode) return;

    if (!c.mouse) {
        if (in.mousePressed[GLFW_MOUSE_BUTTON_1]) {
            if (selectedObject == nullptr) {
                rayCast(SELECT_RANGE);
                selectObject();
            }
            else {
                RaycastHit hitData = rayCast(SELECT_RANGE);
                if (hitData.object == nullptr) {
                    dropObject();
                }
                else if (hitData.object != selectedObject) {
                    dropObject();
                    rayCast(SELECT_RANGE);
                    selectObject();
                }
                else {
                    syncSelectionOffset(); // klick på samma objekt: resync offset så det inte hoppar
                }
            }
            c.mouse = true;
        }

        if (in.mouseDown[GLFW_MOUSE_BUTTON_1]) {
            updateSelectedObject(1.0f / 60.0f); // hardcoded timestep for smoother movement
            c.mouse = true;
        }

        if (in.mouseDown[GLFW_MOUSE_BUTTON_2]) {
            wants.cameraLook = true;
            wants.captureMouse = true;
            c.mouse = true;
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
                glm::vec3(1), 1, 0
            );
            newObject.linearVelocity = camera->front * SHOOT_VELOCITY;
        }

    }

    if (!c.keyboard) {
        if (in.keyPressed[GLFW_KEY_1]) {
            drawPlacementAABB = !drawPlacementAABB;
            c.keyboard = true;
        }

        if (in.keyPressed[GLFW_KEY_2]) {
            this->objectRainBlocks = !this->objectRainBlocks;
            objectRainPos = camera->position + camera->front * OBJ_PLACE_DISTANCE;
            c.keyboard = true;
        }
        if (in.keyPressed[GLFW_KEY_3]) {
            this->objectRainSpheres = !this->objectRainSpheres;
            objectRainPos = camera->position + camera->front * OBJ_PLACE_DISTANCE;
            c.keyboard = true;
        }

        if (in.keyPressed[GLFW_KEY_4]) {
            physicsEngine->sleepAllObjects();
            c.keyboard = true;
        }
        if (in.keyPressed[GLFW_KEY_5]) {
            physicsEngine->awakenAllObjects();
            c.keyboard = true;
        }
    }
}

// -----------------------------------
//        UI rendering
// -----------------------------------
void Editor::EditorMain::drawUI()
{
    if (panelManager)
        panelManager->renderPanels();

    ImGui::Begin("Toolbar1");
    ImGui::End();

    ImGui::Begin("Toolbar2");
    ImGui::End();

    ImGui::Begin("Scene");

    ImVec2 size = ImGui::GetContentRegionAvail();

    if (size.x > 8 && size.y > 8) {
        viewportFBO.resizeIfNeeded((int)size.x, (int)size.y);

        // Visa texturen i ImGui
        ImGui::Image(
            (ImTextureID)(intptr_t)viewportFBO.colorTex,
            size,
            ImVec2(0, 1),
            ImVec2(1, 0)
        );
    }

    ImGui::End();

    // andra panels: Inspector, Hierarchy, osv
}

// activate/deactivate editor mode
void Editor::EditorMain::activate() {

}
void Editor::EditorMain::deactivate() {
    dropObject();
    lastHitData = {};
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
void Editor::EditorMain::selectObject() {
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

// get raycast
RaycastHit& Editor::EditorMain::getLastRayHit() {
    return lastHitData;
}
// Raycast from camera through mouse cursor into world
RaycastHit Editor::EditorMain::rayCast(float length)
{
    float mouseX = inputManager->lastX;
    float mouseY = inputManager->lastY;
    float x = (2.0f * mouseX) / SCR_WIDTH - 1.0f;
    float y = 1.0f - (2.0f * mouseY) / SCR_HEIGHT; // viktigt: invert Y
    // NDC: z = -1 för near plane i OpenGL clip-space
    glm::vec4 rayClip(x, y, -1.0f, 1.0f);

    // 2) Clip → Eye(inverse projection)
    glm::mat4 projection = glm::perspective(glm::radians(camera->zoom), SCR_WIDTH / SCR_HEIGHT, 0.1f, 10000.0f);
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
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
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