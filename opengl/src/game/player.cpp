#include "pch.h"
#include "player.h"

void Player::addInputRouter(InputRouter& router) {
    router.add(this);
}

void Player::handleInput(const InputFrame& in, const InputContext& ctx, Consumed& c) {
    if (!ctx.isPlayerMode) return;

    moveInput = glm::vec3(0.0f);

    if (!c.mouse) {
        if (in.mousePressed[GLFW_MOUSE_BUTTON_1])  { selectObject(); c.mouse = true; }
        if (in.mouseReleased[GLFW_MOUSE_BUTTON_1]) { dropObject();   c.mouse = true; }
        if (in.mousePressed[GLFW_MOUSE_BUTTON_2])  { placeObject();  c.mouse = true; }

        if (in.mousePressed[GLFW_MOUSE_BUTTON_3]) {
            GameObject& newObject = sceneBuilder->createObject("crate", "cube", ColliderType::CUBOID, (camera->position + camera->front * 3.0f), glm::vec3(1), 1, 0);
            newObject.linearVelocity = camera->front * 100.0f;
            newObject.asleep = false;
            c.mouse = true;
        }
    }

    if (!c.keyboard) {
        if (in.keyDown[GLFW_KEY_W]) { moveInput += camera->front; c.keyboard = true; }
        if (in.keyDown[GLFW_KEY_S]) { moveInput -= camera->front; c.keyboard = true; }
        if (in.keyDown[GLFW_KEY_D]) { moveInput += camera->right; c.keyboard = true; }
        if (in.keyDown[GLFW_KEY_A]) { moveInput -= camera->right; c.keyboard = true; }
        moveInput.y = 0.0f;

        if (in.keyDown[GLFW_KEY_SPACE]) {
            if (playerObject->onGround && !playerObject->hasJumped) {
                playerObject->playerJumpImpulse += 10.5f;
                playerObject->onGround = false;
                playerObject->hasJumped = true;
                c.keyboard = true;
            }
        }
    }
}

void Player::setPointers(SceneBuilder* sceneBuilder, PhysicsEngine* physicsEngine, Camera* camera) {
    this->sceneBuilder = sceneBuilder;
    this->physicsEngine = physicsEngine;
    this->camera = camera;
}

void Player::activate() {
    createPlayerObject();
}

void Player::deactivate() {
    destroyPlayerObject();
    dropObject();
}

void Player::createPlayerObject() {
    sceneBuilder->createObject("crate", "cube", ColliderType::CUBOID, camera->position - glm::vec3(0, 0.76f, 0), glm::vec3(1.0f, 1.86f, 1.0f), 1, 0, {}, 1);
    GameObject& player = sceneBuilder->getDynamicObjects().back();
    sceneBuilder->playerObjectId = sceneBuilder->getDynamicObjects().size() - 1;

    player.player = true;
    player.allowSleep = false;
    player.seeThrough = true;

    playerObject = &player;
}

void Player::destroyPlayerObject() {
    GameObject& player = sceneBuilder->getDynamicObjects()[sceneBuilder->playerObjectId];
    physicsEngine->queueRemove(&player);
    //sceneBuilder->getDynamicObjects().erase(sceneBuilder->getDynamicObjects().begin() + sceneBuilder->playerObjectId);
    sceneBuilder->playerObjectId = -1;

    playerObject = nullptr;
}

void Player::fixedUpdate(float fixedTimeStep) {
    updateSelectedObject(fixedTimeStep);
}

void Player::update(Shader& shader) {
    // update player movement
    updatePlayerMovement();

    if (PLAYER_RAYCAST_ENABLED) {
        if (selectedObject == nullptr) {
            createPlaceObjectAABB(shader);
            RaycastHit hitData = rayCast(5000);
        }
    }
}

// Update player movement based on input
void Player::updatePlayerMovement() {
    //camera->position = player.position - camera->front * glm::vec3(12.0f) + glm::vec3(0, 0.76f, 0);
    camera->position = playerObject->position + glm::vec3(0, 0.76f, 0);

    // Undvik diag-fartbonus
    if (glm::length2(moveInput) > 0.0f) {
        moveInput = glm::normalize(moveInput);
    }

    const float moveSpeed = 8.f; // meter per sekund
    playerObject->playerMoveImpulse = moveInput * moveSpeed;
}

// Update position of selected object to follow camera + offset
void Player::updateSelectedObject(float dt) {
    if (!selectedObject or selectedObject->isStatic)
        return;

    glm::vec3 worldOffset = camera->right * selectionOffsetLocal.x + camera->up * selectionOffsetLocal.y + camera->front * selectionOffsetLocal.z;

    // position
    glm::vec3 newPos = camera->position + worldOffset;
    selectedObject->position = newPos;

    // velocity
    selectedObject->linearVelocity = (newPos - selectedObject->lastPosition) / dt;
    selectedObject->angularVelocity = glm::vec3(0.0f);
    selectedObject->lastPosition = newPos;

    selectedObject->modelMatrixDirty = true;
    selectedObject->aabbDirty = true;
    selectedObject->setModelMatrix();
    selectedObject->updateAABB();
    selectedObject->updateCollider();


}

// Select object under crosshair
void Player::selectObject() {
    if (selectedObject)
        return;

    RaycastHit& hitData = lastHitData;

    // no hit return
    if (hitData.object == nullptr) {
        return;
    }

    selectedObject = hitData.object;
    if (selectedObject->isStatic) {
        selectedObject = nullptr;
        return;
    }

    selectedObject->selectedByPlayer = true;
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

void Player::dropObject() {
    if (selectedObject) {
        selectedObject->selectedByPlayer = false;
        selectedObject->asleep = false;

        // avoid nullptr dereference in physics engine
        GameObject* obj = selectedObject;
        physicsEngine->queueAdd(obj);

        if (selectedObject->sleepCounterThreshold > 1000.0f) // hack for static object made dynamic (because cant move from static bvh to dynamic/asleep bvh)
            selectedObject->sleepCounterThreshold = 1.5f;

        selectedObject->sleepCounter = 0.0f;
        selectedObject->angularVelocity = glm::vec3(0.0f);

        selectedObject = nullptr;
    }
}

void Player::placeObject() {
    if (placementObstructed)
        return;

    glm::vec3 size{ OBJ_PLACE_SIZE };
    glm::vec3 spawnPos = aabbToPlace.centroid;
    glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(0.0f, 0.0f, 0.0f));
    GameObject& newObject = sceneBuilder->createObject("crate", "cube", ColliderType::CUBOID, spawnPos, size, 1, 0, orientation, 1.5f, 0);
    newObject.linearVelocity = glm::vec3(0.0f);
}

void Player::createPlaceObjectAABB(Shader& shader) {
    float placeDist = 150.0f;
    glm::vec3 size{ OBJ_PLACE_SIZE };

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
    }
    else {
        this->placementObstructed = false;
        aabbToPlace = aabb;
        color = glm::vec3{ 0.9f, 0.7f, 0.2f };
    }

    if (drawPlacementAABB)
        drawAABB(aabb, shader, color);
}

RaycastHit Player::rayCast(float length) {
    float rLength = length;
    Ray r(camera->position, camera->front, rLength);
    RaycastHit hitData = physicsEngine->performRaycast(r);

    lastHitData = hitData;
    return hitData;
}

RaycastHit& Player::getLastRayHit() {
    return lastHitData;
}


void Player::drawAABB(const AABB& aabb, Shader& shader, glm::vec3 color) {
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