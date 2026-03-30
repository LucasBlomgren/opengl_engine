#include "pch.h"
#include "player.h"

void Player::handleInput(const InputFrame& in, const InputContext& ctx, Consumed& c, FrameWants& wants) {
    if (!ctx.isPlayerMode) return;

    wants.cameraLook = true;
    wants.captureMouse = true;

    if (!c.mouse) {
        if (in.mousePressed[GLFW_MOUSE_BUTTON_1])  { selectObject(); }
        if (in.mouseReleased[GLFW_MOUSE_BUTTON_1]) { pendingDrop = true; c.mouse = true; }
        if (in.mousePressed[GLFW_MOUSE_BUTTON_2])  { placeObject();  c.mouse = true; }

        if (in.mousePressed[GLFW_MOUSE_BUTTON_3]) {
            GameObjectHandle handle = world->createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Dynamic, (camera->position + camera->front * 3.0f), glm::vec3(1), 1, {}, 1.5f, false, {}, false);
            GameObject* newObj = world->getGameObject(handle);
            RigidBody* rb = world->getRigidBody(handle);
            rb->linearVelocity = camera->front * SHOOT_VELOCITY;
            c.mouse = true;
        }

        if (in.mouseDown[GLFW_MOUSE_BUTTON_1]) {
            updateSelectedObject(1.0f / 60.0f); // hardcoded timestep for smoother movement
            c.mouse = true;
        }
    }

    moveInput = glm::vec3(0.0f);
    if (!c.keyboard) {
        if (in.keyDown[GLFW_KEY_W]) { moveInput += camera->front; c.keyboard = true; }
        if (in.keyDown[GLFW_KEY_S]) { moveInput -= camera->front; c.keyboard = true; }
        if (in.keyDown[GLFW_KEY_D]) { moveInput += camera->right; c.keyboard = true; }
        if (in.keyDown[GLFW_KEY_A]) { moveInput -= camera->right; c.keyboard = true; }
        moveInput.y = 0.0f;

        if (in.keyDown[GLFW_KEY_SPACE]) {
            GameObject* playerObject = world->getGameObject(playerHandle);
            if (playerObject->onGround and !playerObject->hasJumped) {
                playerObject->playerJumpImpulse += JUMP_HEIGHT;
                playerObject->onGround = false;
                playerObject->hasJumped = true;
                c.keyboard = true;
            }
        }
    }
}

void Player::setPointers(World* world, PhysicsEngine* physicsEngine, Camera* camera) {
    this->world = world;
    this->physicsEngine = physicsEngine;
    this->camera = camera;
}

void Player::activate() {
    createPlayerObject();
}

void Player::deactivate() {
    destroyPlayerObject();
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
}

void Player::createPlayerObject() {
    playerHandle = world->createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Dynamic, camera->position - glm::vec3(0, 0.76f, 0), glm::vec3(1.0f, 1.86f, 1.0f), 1, {}, 1, false, glm::vec3{255}, true);

    GameObject* player = world->getGameObject(playerHandle);
    RigidBody* rb = world->getRigidBody(playerHandle);
    player->player = true;
    rb->allowSleep = false;
}

void Player::destroyPlayerObject() {
    GameObject* player = world->getGameObject(playerHandle);
    physicsEngine->queueRemove(player->colliderHandle);
    world->deleteGameObject(playerHandle);
    playerHandle = GameObjectHandle{};
}

void Player::fixedUpdate(float fixedTimeStep) {
    //updateSelectedObject(fixedTimeStep);
}

void Player::update(Shader& shader) {
    // update player movement
    updatePlayerMovement();

    if (pendingDrop) {
        dropObject();
        pendingDrop = false;
    }

    if (!objectIsSelected) {
        createPlaceObjectAABB(shader);
    }

    // clear previous hover state
    if (objectIsHovered) {
        GameObject* hoveredObj = world->getGameObject(hoveredObjectHandle);
        hoveredObj->hoveredByEditor = false;
        objectIsHovered = false;
    }

    // raycast for hover
    RaycastHit raycast = rayCast(SELECT_RANGE);

    // set new hover state
    if (raycast.hit && !objectIsSelected) {
        Collider* collider = world->getCollider(raycast.colliderHandle);
        RigidBody* rb = world->getRigidBody(collider->rigidBodyHandle);
        GameObject* hoveredObj = world->getGameObject(collider->gameObjectHandle);
        hoveredObj->hoveredByEditor = true;
        hoveredObjectHandle = collider->gameObjectHandle;
        objectIsHovered = true;
        rb->type = BodyType::Kinematic; // make kinematic while hovered for better interaction (e.g. no gravity while hovering)
    }
}

// Update player movement based on input
void Player::updatePlayerMovement() {
    //camera->position = player.position - camera->front * glm::vec3(12.0f) + glm::vec3(0, 0.76f, 0);

    GameObject* playerObject = world->getGameObject(playerHandle);
    camera->position = playerObject->transform.position + glm::vec3(0, 0.76f, 0);

    // Undvik diag-fartbonus
    if (glm::length2(moveInput) > 0.0f) {
        moveInput = glm::normalize(moveInput);
    }

    const float moveSpeed = 8.f; // meter per sekund
    playerObject->playerMoveImpulse = moveInput * moveSpeed;

    updateSelectedObject(1.0f / 60.0f);
}

// Update position of selected object to follow camera + offset
void Player::updateSelectedObject(float dt) {
    // function uses selectedObjectHandle not pointer
    if (objectIsSelected == false)
        return;

    GameObject* playerObject = world->getGameObject(playerHandle);
    GameObject* selectedObject = world->getGameObject(selectedObjectHandle);
    RigidBody* selectedRb = world->getRigidBody(selectedObject->rigidBodyHandle);
    RigidBody* playerRb = world->getRigidBody(playerObject->rigidBodyHandle);
    Collider* selectedCollider = world->getCollider(selectedRb->colliderHandle);

    glm::vec3 worldOffset = camera->right * selectionOffsetLocal.x + camera->up * selectionOffsetLocal.y + camera->front * selectionOffsetLocal.z;

    // position
    glm::vec3 newPos = camera->position + worldOffset;
    selectedObject->transform.position = newPos;

    // velocity
    selectedRb->linearVelocity = (newPos - selectedObject->transform.lastPosition) / dt;
    selectedRb->linearVelocity += playerRb->linearVelocity;
    selectedRb->angularVelocity = glm::vec3(0.0f);
    selectedObject->transform.lastPosition = newPos;

    selectedObject->transform.updateCache();
    selectedCollider->updateAABB(selectedObject->transform);
    selectedCollider->updateCollider(selectedObject->transform);
}

// Select object under crosshair
void Player::selectObject() {
    if (objectIsSelected)
        return;

    RaycastHit raycast = rayCast(SELECT_RANGE);

    // no hit return
    if (raycast.hit == false) {
        return;
    }

    GameObject* selectedObject = world->getGameObject(raycast.colliderHandle);
    RigidBody* selectedRb = world->getRigidBody(selectedObject->rigidBodyHandle);

    if (selectedRb->type == BodyType::Static) {
        objectIsSelected = false;
        return;
    }

    objectIsSelected = true;
    selectedObject->selectedByPlayer = true;
    selectedRb->asleep = false;
    selectedRb->type = BodyType::Kinematic;

    // avoid nullptr dereference in physics engine
    GameObject* obj = selectedObject;
    BroadphaseBucket bpBucket;
    bpBucket = BroadphaseBucket::Awake;
    physicsEngine->queueMove(raycast.colliderHandle, bpBucket);

    selectedRb->sleepCounterThreshold = FLT_MAX; // avoid sleeping while being edited
    selectedRb->allowSleep = false;
    selectedObject->transform.lastPosition = selectedObject->transform.position;

    selectedRb->linearVelocity = glm::vec3(0.0f);
    selectedRb->angularVelocity = glm::vec3(0.0f);

    glm::vec3 worldOffset = selectedObject->transform.position - camera->position;
    // Projicera worldOffset på kamerans lokala axlar:
    selectionOffsetLocal.x = glm::dot(worldOffset, camera->right);
    selectionOffsetLocal.y = glm::dot(worldOffset, camera->up);
    selectionOffsetLocal.z = glm::dot(worldOffset, camera->front);
}

void Player::dropObject() {
    if (objectIsSelected) {
        objectIsSelected = false;

        GameObject* selectedObject = world->getGameObject(selectedObjectHandle);
        RigidBody* selectedRb = world->getRigidBody(selectedObject->rigidBodyHandle);
        selectedObject->selectedByPlayer = false;
        selectedRb->allowSleep = true;
        selectedRb->sleepCounter = 0.0f;
        selectedRb->sleepCounterThreshold = 1.5f;
        selectedRb->angularVelocity = glm::vec3(0.0f);

        BroadphaseBucket target;
        if (selectedRb->type == BodyType::Static) {
            target = BroadphaseBucket::Static;
        } else {
            target = BroadphaseBucket::Awake;
        }
        physicsEngine->queueMove(selectedObject->colliderHandle, target);
    }
}

void Player::placeObject() {
    if (placementObstructed)
        return;

    glm::vec3 size{ OBJ_PLACE_SIZE };
    glm::vec3 spawnPos = aabbToPlace.centroid;
    glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(0.0f, 0.0f, 0.0f));

    GameObjectHandle handle = world->createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Dynamic, spawnPos, size, 1, orientation, 1.5f, false, {}, false);
    GameObject* newObj = world->getGameObject(handle);
    RigidBody* rb = world->getRigidBody(newObj->rigidBodyHandle);
    rb->linearVelocity = glm::vec3(0.0f);
}

void Player::createPlaceObjectAABB(Shader& shader) {
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

RaycastHit Player::rayCast(float length) {
    float rLength = length;
    Ray r(camera->position, camera->front, rLength);
    RaycastHit hitData = physicsEngine->performRaycast(r);

    return hitData;
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