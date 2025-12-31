#include "pch.h"
#include "editor.h"

#include "scene_builder.h"
#include "physics.h"
#include "game_object.h"
#include "aabb.h"
#include "shaders/shader.h"
#include "physics/raycast.h"

void Editor::addInputRouter(InputRouter& router) {
    router.add(this);
}

void Editor::handleInput(const InputFrame& in, const InputContext& ctx, Consumed& c, FrameWants& wants) {
    if (ctx.isPlayerMode) return;

    if (!c.mouse) {
        if (in.mousePressed[GLFW_MOUSE_BUTTON_1]) {
            if (selectedObject == nullptr) {
                selectObject();
            }
            else {
                RaycastHit hitData = rayCast(SELECT_RANGE);
                if (hitData.object == nullptr) {
                    dropObject();
                } 
                else if (hitData.object != selectedObject) {
                    dropObject();
                    selectObject();
                }
                else {
                    syncSelectionOffset(); // klick på samma objekt: resync offset så det inte hoppar
                }
            }
            c.mouse = true;
        }

        if (in.mouseDown[GLFW_MOUSE_BUTTON_1] and !in.mousePressed[GLFW_MOUSE_BUTTON_1]) {
            updateSelectedObject(1.0f/60.0f); // hardcoded timestep for smoother movement
            c.mouse = true;
        }

        if (in.mouseDown[GLFW_MOUSE_BUTTON_2]) {
            wants.cameraLook = true;
            wants.captureMouse = true;
            c.mouse = true;
        }

        if (in.mouseDown[GLFW_MOUSE_BUTTON_3]) {
            placeObject();
            c.mouse = true;
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

void Editor::setPointers(SceneBuilder* sceneBuilder, PhysicsEngine* physicsEngine, Camera* camera) {
    this->sceneBuilder = sceneBuilder;
    this->physicsEngine = physicsEngine;
    this->camera = camera;
}

void Editor::activate() {

}

void Editor::deactivate() {
    dropObject();
    lastHitData = {};
}

void Editor::fixedUpdate(float fixedTimeStep) {
    // update selected object
    //updateSelectedObject(fixedTimeStep);
}

void Editor::update(Shader& shader) {
    // raycast and draw placement AABB
    if (selectedObject == nullptr) {
        createPlaceObjectAABB(shader);
        RaycastHit hitData = rayCast(SELECT_RANGE);
    }
}

void Editor::syncSelectionOffset() {
    if (!selectedObject) return;

    glm::vec3 worldOffset = selectedObject->position - camera->position;
    selectionOffsetLocal.x = glm::dot(worldOffset, camera->right);
    selectionOffsetLocal.y = glm::dot(worldOffset, camera->up);
    selectionOffsetLocal.z = glm::dot(worldOffset, camera->front);

    selectedObject->lastPosition = selectedObject->position;
}

// select object
void Editor::selectObject() {
    if (selectedObject)
        return;

    RaycastHit& hitData = lastHitData;

    // no hit return
    if (hitData.object == nullptr) {
        return;
    }
    selectedObject = hitData.object;

    selectedObject->selectedByEditor = true;
    selectedObject->asleep = false;

    // avoid nullptr dereference in physics engine
    GameObject* obj = selectedObject;
    physicsEngine->queueAdd(obj);

    selectedObject->sleepCounterThreshold = FLT_MAX; // avoid sleeping while being edited
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
void Editor::updateSelectedObject(float fixedTimeStep) {
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
void Editor::dropObject() {
    if (selectedObject) {
        selectedObject->selectedByEditor = false;
        selectedObject->asleep = false;

        // avoid nullptr dereference in physics engine
        GameObject* obj = selectedObject;
        physicsEngine->queueAdd(obj);

        if (selectedObject->sleepCounterThreshold > 1000.0f) // hack for static object made dynamic (because cant move from static bvh to dynamic/asleep bvh)
            selectedObject->sleepCounterThreshold = 1.5f;

        selectedObject->sleepCounter = 0.0f;
        selectedObject->linearVelocity = glm::vec3(0.0f);
        selectedObject->angularVelocity = glm::vec3(0.0f);
        selectedObject = nullptr;
    }
}

// place object at placement AABB position
void Editor::placeObject() {
    if (placementObstructed) 
        return;

    glm::vec3 size{ OBJ_PLACE_SIZE };
    glm::vec3 spawnPos = aabbToPlace.centroid;
    glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(0.0f, 0.0f, 0.0f));
    GameObject& newObject = sceneBuilder->createObject("crate", "cube", ColliderType::CUBOID, spawnPos, size, 1, 0, orientation, 1.5f, 0);
    newObject.linearVelocity = glm::vec3(0.0f);
}

// create placement AABB in front of camera, adjusted for collisions
void Editor::createPlaceObjectAABB(Shader& shader) {
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

    BVHTree<GameObject>& dynamicAwakeBvh = physicsEngine->getDynamicAsleepBvh();
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
    } else {
        this->placementObstructed = false;
        aabbToPlace = aabb;
        color = glm::vec3{ 0.9f, 0.7f, 0.2f };
    }

    if (drawPlacementAABB)
        drawAABB(aabb, shader, color);
}

RaycastHit Editor::rayCast(float length) {
    float rLength = length;
    Ray r(camera->position, camera->front, rLength);
    RaycastHit hitData = physicsEngine->performRaycast(r);

    lastHitData = hitData;
    return hitData;
}

RaycastHit& Editor::getLastRayHit() {
    return lastHitData;
}

void Editor::drawAABB(const AABB& aabb, Shader& shader, glm::vec3 color) {
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