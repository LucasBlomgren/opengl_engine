#include "editor.h"

void Editor::setPointers(
    EngineState* engineState,
    SceneBuilder* sceneBuilder,
    PhysicsEngine* physicsEngine,
    Camera* camera,
    std::vector<Vertex>* cubeVertices,
    std::vector<unsigned int>* indices) 
{
        this->engineState = engineState;
        this->sceneBuilder = sceneBuilder;
        this->physicsEngine = physicsEngine;
        this->camera = camera;
        this->cubeVertices = cubeVertices;
        this->indices = indices;
}

void Editor::update(float& deltaTime, Shader& shader) {
    updateSelectedObject(deltaTime);

    if (engineState->GetPressedKey() == "H") {
        sceneBuilder->createScene(*physicsEngine);
        physicsEngine->getDynamicBvh().build(sceneBuilder->getDynamicObjects());    
        selectedObject = nullptr;
    }
    if (engineState->GetPressedKey() == "6") {
        sceneBuilder->toggleLightsState();
    }
    if (engineState->GetPressedKey() == "7") {
        this->drawPlacementAABB = !this->drawPlacementAABB;
    }
    if (engineState->GetPressedKey() == "8") {
        this->objectRainBlocks = !this->objectRainBlocks;
    }
    if (engineState->GetPressedKey() == "9") {
        this->objectRainSpheres = !this->objectRainSpheres;
    }

    if (engineState->GetPressedKey() == "M1_PRESS") 
        placeObject();
    if (engineState->GetPressedKey() == "M2_RELEASE")
        dropObject();
    if (engineState->GetPressedKey() == "M2_PRESS") 
        selectObject();

    if (engineState->GetPressedKey() == "M3_PRESS") {
        GameObject& newObject = sceneBuilder->createObject("crate", ColliderType::CUBOID, (camera->position + camera->front * 3.0f), glm::vec3(1), 1, 0);
        newObject.linearVelocity = camera->front *  100.0f;
        newObject.asleep = false;
    }

    engineState->clearPressedKey();

    if (selectedObject == nullptr) {
        RaycastHit hitData = rayCast(5000);
        if (hitData.object != nullptr) {
            hitData.object->isRaycastHit = true;
        }

        createPlaceObjectAABB(shader);
    }
}

void Editor::placeObject() {
    if (placementObstructed) 
        return;

    glm::vec3 size{ objPlaceSize };
    glm::vec3 spawnPos = aabbToPlace.centroid;
    glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(0.0f, 0.0f, 0.0f));
    GameObject& newObject = sceneBuilder->createObject("crate", ColliderType::CUBOID, spawnPos, size, 1, 0, orientation, 0.5f, 1);
    newObject.linearVelocity = glm::vec3(0.0f);
}

void Editor::createPlaceObjectAABB(Shader& shader) {
    float placeDist = 150.0f;
    glm::vec3 size{ objPlaceSize };

    AABB aabb;
    aabb.centroid = camera->position + camera->front * placeDist;
    aabb.halfExtents = glm::vec3(size / 2.0f);

    RaycastHit hitData = rayCast(placeDist);
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

    BVHTree<GameObject>& dynamicBvh = physicsEngine->getDynamicBvh();
    int maxIter = 8;
    int iter = 0;
    for (int i = 0; i < maxIter; i++) {
        std::vector<GameObject*> collisions;
        dynamicBvh.singleQuery(aabb, collisions);

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

void Editor::dropObject() {
    if (selectedObject) {
        selectedObject->selectedByEditor = false;
        selectedObject->asleep = false;
        selectedObject->sleepCounterThreshold = 0.5f;
        selectedObject->sleepCounter = 0.0f;
        selectedObject->angularVelocity = glm::vec3(0.0f);
        selectedObject = nullptr;
        return;
    }
}

void Editor::selectObject() {
    if (selectedObject)
        return;

    RaycastHit hitData = rayCast(5000);

    // no hit return
    if (hitData.object == nullptr) {
        return;
    }
    selectedObject = hitData.object;
    if (selectedObject->isStatic) {
        selectedObject = nullptr;
        return;
    }

    selectedObject->selectedByEditor = true;
    selectedObject->asleep = false;
    selectedObject->sleepCounterThreshold = 1000000.0f;
    selectedObject->lastPosition = selectedObject->position;

    selectedObject->linearVelocity = glm::vec3(0.0f);
    selectedObject->angularVelocity = glm::vec3(0.0f);

    glm::vec3 worldOffset = selectedObject->position - camera->position;
    // Projicera worldOffset på kamerans lokala axlar:
    selectionOffsetLocal.x = glm::dot(worldOffset, camera->right);
    selectionOffsetLocal.y = glm::dot(worldOffset, camera->up);
    selectionOffsetLocal.z = glm::dot(worldOffset, camera->front);
}

void Editor::updateSelectedObject(float fixedTimeStep) {
    if (!selectedObject) 
        return; 

    glm::vec3 worldOffset = camera->right * selectionOffsetLocal.x + camera->up * selectionOffsetLocal.y + camera->front * selectionOffsetLocal.z;

    // position
    glm::vec3 newPos = camera->position + worldOffset;
    selectedObject->position = newPos;
    // velocity
    selectedObject->linearVelocity = (newPos - selectedObject->lastPosition) / fixedTimeStep;

    selectedObject->lastPosition = newPos;
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