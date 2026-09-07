#include "pch.h"
#include "player.h"
#include "world.h"

void Player::setPointers(World* world, physics::Engine* physics, Renderer* renderer, Camera* camera) {
    this->world = world;
    this->physics = physics;
    this->renderer = renderer;
    this->camera = camera;
}

physics::BodyHandle Player::getPlayerRigidBodyHandle() {
    GameObject* player = world->getGameObject(playerHandle);
    if (player) {
        return player->rigidBodyHandle;
    }
    return physics::BodyHandle{};
}

//------------------------------------------------------
// INPUT HANDLING
//-------------------------------------------------------
void Player::handleInput(const InputFrame& in, const InputContext& ctx, Consumed& c, FrameWants& wants) {
    if (!ctx.isPlayerMode) return;

    wants.cameraLook = true;
    wants.captureMouse = true;

    if (!c.mouse) {
        if (in.mousePressed[GLFW_MOUSE_BUTTON_1]) { selectObject(); c.mouse = true;  }
        if (in.mouseReleased[GLFW_MOUSE_BUTTON_1]) { pendingDrop = true; c.mouse = true; }
        if (in.mousePressed[GLFW_MOUSE_BUTTON_2])  { placeObject();  c.mouse = true; }

        if (in.mousePressed[GLFW_MOUSE_BUTTON_3]) {
            c.mouse = true;

            // create new object description
            GameObjectDesc newObj;
            glm::vec3 position = { camera->position + camera->front * 5.0f };
            glm::vec3 size = glm::vec3{ SHOOT_SIZE };
            newObj.mass = SHOOT_MASS;
            newObj.rootTransformHandle = world->createTransform(position, glm::quat{ 1.0f, 0.0f, 0.0f, 0.0f }, size);

            SubPartDesc part;
            part.localTransformHandle = world->createTransform();
            part.colliderType = physics::ColliderType::SPHERE;
            part.textureName = "checker_gray";
            part.meshName = "sphere";
            newObj.parts.push_back(part);

            // create new object & apply shoot velocity
            GameObjectHandle newObject = world->createGameObject(newObj);
            GameObject* object = world->getGameObject(newObject);

            if (object) {
                physics->setLinearVelocity(
                    object->rigidBodyHandle,
                    camera->front * SHOOT_VELOCITY
                );
            }
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
            if (onGround and !hasJumped) {
                jumpImpulse += JUMP_HEIGHT;
                hasJumped = true;
                c.keyboard = true;
            }
        }
    }
}


//------------------------------------------------------
// SWITCHING PLAYER MODE
//-------------------------------------------------------
void Player::resetState() {
    onGround = false;
    hasJumped = false;
    moveInput = glm::vec3(0.0f);
    moveImpulse = glm::vec3(0.0f);
    jumpImpulse = 0.0f;

    selectedObjectHandle = {};
    hoveredObjectHandle = {};
}
void Player::activate() {
    resetState();
    createPlayerObject();
}

void Player::deactivate() {
    resetState();
    destroyPlayerObject();
    dropObject();
}

void Player::createPlayerObject() {
    GameObjectDesc playerDesc;
    glm::vec3 size = glm::vec3{ 1.0f, 1.86f, 1.0f };
    glm::vec3 position = camera->position - glm::vec3(0, 0.76f, 0);
    playerDesc.rootTransformHandle = world->createTransform(position, glm::quat{ 1.0f, 0.0f, 0.0f, 0.0f }, size);
    playerDesc.bodyType = physics::BodyType::Kinematic;
    playerDesc.reportContacts = true;
    SubPartDesc part;
    part.seeThrough = true;
    part.localTransformHandle = world->createTransform();
    part.colliderType = physics::ColliderType::CUBOID;
    playerDesc.parts.push_back(part);
    playerHandle = world->createGameObject(playerDesc);

    GameObject* player = world->getGameObject(playerHandle);
    player->player = true;
}

void Player::destroyPlayerObject() {
    world->deleteGameObject(playerHandle);
    playerHandle = GameObjectHandle{};
}


//----------------------------------------------------------
// MOVEMENT, CAMERA UPDATE and OBJECT SELECTION UPDATE
//----------------------------------------------------------
void Player::updateMovement(float dt) {
    TransformHandle& tHandle = world->getGameObject(playerHandle)->rootTransformHandle;
    Transform* t = world->getTransform(tHandle);

    // first person camera
    camera->position = t->position + glm::vec3(0, 0.76f, 0);
    //headBob(dt);

    // third person camera
    //camera->position = t->position - camera->front * glm::vec3(12.0f) + glm::vec3(0, 0.76f, 0);

    // avoid diagonal speed boost
    if (glm::length2(moveInput) > 0.0f) {
        moveInput = glm::normalize(moveInput);
    }
    moveImpulse = moveInput * MOVE_ACCELERATION;
}

void Player::headBob(float dt) {
    float bobY = 0.0f;
    float bobX = 0.0f;

    if (onGround && glm::length(moveImpulse) >= 0.001f) {
        bobTimer += dt * bobFrequency;
        bobY = std::sin(bobTimer) * bobAmount;
        bobX = std::sin(bobTimer * 0.5f) * sideBobAmount;
    }
    else {
        bobTimer = 0.0f;
        bobY = 0.0f;
        bobX = 0.0f;
    }

    GameObject* player = world->getGameObject(playerHandle);
    if (!player) return;

    Transform* t = world->getTransform(player->rootTransformHandle);
    if (!t) return;

    camera->position = t->position
        + glm::vec3(0.0f, eyeHeight, 0.0f)
        + camera->right * bobX
        + glm::vec3(0.0f, bobY, 0.0f);
}

void Player::updateObjectSelection(Shader& shader) {
    if (pendingDrop) {
        dropObject();
        pendingDrop = false;
    }

    if (!selectedObjectHandle.isValid()) {
        createPlaceObjectAABB(shader);
    }

    // raycast for hover
    physics::RaycastHit raycast = Player::raycast(SELECT_RANGE);

    // set new hover state
    if (raycast.hit && !selectedObjectHandle.isValid()) {
        hoveredObjectHandle =
            world->getGameObjectHandle(raycast.bodyHandle);
    }
    else {
        hoveredObjectHandle = {};
    }
}

//------------------------------------------------------
// PHYSICS
//-------------------------------------------------------
void Player::updateBody(float dt) {
    GameObject* player = world->getGameObject(playerHandle);
    if (!player) return;

    const std::optional<physics::BodyState> bodyState =
        physics->getRigidBodyState(player->rigidBodyHandle);
    if (!bodyState) return;

    Transform* t = world->getTransform(player->rootTransformHandle);
    if (!t) return;

    glm::vec3 v = bodyState->linearVelocity;

    // air friction
    if (!onGround) {
        moveImpulse.x *= AIR_FRICTION;
        moveImpulse.z *= AIR_FRICTION;
    }

    v.x += moveImpulse.x;
    v.z += moveImpulse.z;

    // apply stronger gravity when in air for better jump feel and faster fall speed
    // reset vertical velocity when landing to prevent small bounces from accumulated gravity
    if (!onGround) {
        v.y += GRAVITY * GRAVITY_MULTIPLIER * dt;
    }
    else if (v.y < 0.0f) {
        v.y = 0.0f;
    }

    if (jumpImpulse != 0.0f) {
        v.y += jumpImpulse;
        jumpImpulse = 0.0f;
    }

    glm::vec2 vxz(v.x, v.z);
    float spd = glm::length(vxz);
    if (spd > MAX_MOVE_SPEED) {
        vxz *= (MAX_MOVE_SPEED / spd); // scale both x and z to max speed while preserving direction
        v.x = vxz.x;
        v.z = vxz.y;
    }

    //if (onGround and glm::length2(playerMoveImpulse) < 0.01f) {
    //    v.x -= v.x * std::min(20.0f * dt, 3.0f);
    //    v.z -= v.z * std::min(20.0f * dt, 3.0f);
    //}


    // stop horizontal movement when on ground and no input to prevent
    // sliding on slopes and when landing from a jump
    if (onGround and glm::length2(moveImpulse) < 0.01f) {
        v.x = 0.0f;
        v.z = 0.0f;
    }

    physics->setLinearVelocity(
        player->rigidBodyHandle,
        v
    );
    t->lastPosition = t->position;
    t->position += v * dt;
    t->updateCache();
}

void Player::resolveExternalContact() {
    onGround = false;

    const auto& contacts = physics->getExternalMotionContacts();
    GameObject* playerObj = world->getGameObject(playerHandle);
    Transform* transform = world->getTransform(playerObj->rootTransformHandle);

    for (const physics::ExternalMotionContact& c : contacts) {
        bool playerIsA = false;
        bool playerInContact = false;

        if (c.bodyA == playerObj->rigidBodyHandle && c.bodyA.isValid()) {
            playerInContact = true;
            playerIsA = true;
        }
        else if (c.bodyB == playerObj->rigidBodyHandle && c.bodyB.isValid()) {
            playerIsA = false;
            playerInContact = true;
        }

        if (!playerInContact) continue;

        glm::vec3 normal = c.normal;
        if (playerIsA) {
            normal = -normal;
        }

        if (c.terrainContact) {
            if (std::abs(normal.x) <= 1e-1f) {
                normal.x = 0.0f;
            }

            if (std::abs(normal.z) <= 1e-1f) {
                normal.z = 0.0f;
            }
        }

        constexpr glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
        constexpr float penetrationSlop = 0.005f;
        float correction = std::max(c.penetration - penetrationSlop, 0.0f);

        if (glm::dot(up, normal) > ON_GROUND_ANGLE_THRESHOLD) {
            onGround = true;
        }

        transform->position += normal * correction;
    }

    if (onGround) {
        hasJumped = false;
    }
}

//---------------------------------------------------------------
// SELECTION AND PLACEMENT
// --------------------------------------------------------------

// Update position of selected object to follow camera + offset
void Player::moveSelectedObject(float dt) {
    if (!selectedObjectHandle.isValid()) return;

    GameObject* selectedObject = world->getGameObject(selectedObjectHandle);
    Transform* selectedTransform = world->getTransform(selectedObject->rootTransformHandle);

    glm::vec3 worldOffset = camera->right * selectionOffsetLocal.x + camera->up * selectionOffsetLocal.y + camera->front * selectionOffsetLocal.z;

    // position
    glm::vec3 newPos = camera->position + worldOffset;
    selectedTransform->position = newPos;

    // velocity
    physics->setLinearVelocity(
        selectedObject->rigidBodyHandle,
        (newPos - selectedTransform->lastPosition) / dt
    );
    physics->setAngularVelocity(
        selectedObject->rigidBodyHandle,
        glm::vec3(0.0f)
    );
    selectedTransform->lastPosition = newPos;

    selectedTransform->updateCache();

    world->syncGameObjectTransformToPhysics(selectedObjectHandle);
}

//------------------------------------------------------
// OBJECT SELECTION AND PLACEMENT
//------------------------------------------------------

// Select object under crosshair
void Player::selectObject() {
    if (selectedObjectHandle.isValid()) return;

    physics::RaycastHit raycast = Player::raycast(SELECT_RANGE);

    // no hit return
    if (!raycast.hit) return;

    const std::optional<physics::BodyState> bodyState =
        physics->getRigidBodyState(raycast.bodyHandle);

    if (!bodyState || bodyState->type == physics::BodyType::Static) return;

    GameObjectHandle handle =
        world->getGameObjectHandle(raycast.bodyHandle);
    if (!handle.isValid()) return;

    selectedObjectHandle = handle;

    GameObject* obj = world->getGameObject(handle);
    Transform* transform = world->getTransform(obj->rootTransformHandle);
    physics->setRigidBodyType(raycast.bodyHandle, physics::BodyType::Kinematic);
    physics->setRigidBodySleepState(raycast.bodyHandle, false);

    transform->lastPosition = transform->position;

    physics->setLinearVelocity(
        raycast.bodyHandle,
        glm::vec3(0.0f)
    );
    physics->setAngularVelocity(
        raycast.bodyHandle,
        glm::vec3(0.0f)
    );

    // Project worldofset onto cameras local axes
    glm::vec3 worldOffset = transform->position - camera->position;
    selectionOffsetLocal.x = glm::dot(worldOffset, camera->right);
    selectionOffsetLocal.y = glm::dot(worldOffset, camera->up);
    selectionOffsetLocal.z = glm::dot(worldOffset, camera->front);
}

void Player::dropObject() {
    if (selectedObjectHandle.isValid()) {
        GameObject* selectedObject = world->getGameObject(selectedObjectHandle);

        if (selectedObject) {
            physics->setRigidBodyType(
                selectedObject->rigidBodyHandle,
                physics::BodyType::Dynamic
            );
        }
    }

    selectedObjectHandle = {};
    hoveredObjectHandle = {};
}

void Player::placeObject() {
    if (placementObstructed)
        return;

    glm::vec3 size{ OBJ_PLACE_SIZE };
    glm::vec3 spawnPos = aabbToPlace.worldCenter;
    glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(0.0f, 0.0f, 0.0f));

    // create new object description
    GameObjectDesc newObj;
    newObj.mass = SHOOT_MASS;
    newObj.rootTransformHandle = world->createTransform(spawnPos, orientation, size);

    SubPartDesc part;
    part.localTransformHandle = world->createTransform();
    part.colliderType = physics::ColliderType::CUBOID;
    part.textureName = "checker_gray";
    part.meshName = "cube";
    newObj.parts.push_back(part);

    // create new object & apply shoot velocity
    world->createGameObject(newObj);
}

void Player::createPlaceObjectAABB(Shader& shader) {
    glm::vec3 size{ OBJ_PLACE_SIZE };

    physics::AABB aabb;
    aabb.worldCenter = camera->position + camera->front * OBJ_PLACE_DISTANCE;
    aabb.worldHalfExtents = glm::vec3(size / 2.0f);

    physics::RaycastHit hitData = raycast(OBJ_PLACE_DISTANCE);
    glm::vec3 normal = hitData.normal;
    if (hitData.hit) {
        if (glm::dot(hitData.normal, camera->front) > 0.0f)
            normal = -normal;

        glm::vec3 absN = glm::abs(normal);
        float extentOnN = glm::dot(aabb.worldHalfExtents, absN);

        float margin = 0.1f;
        aabb.worldCenter = hitData.point + normal * (extentOnN - margin);
    }

    aabb.worldMin = aabb.worldCenter - aabb.worldHalfExtents;
    aabb.worldMax = aabb.worldCenter + aabb.worldHalfExtents;
    int maxIter = 8;
    int iter = 0;
    for (int i = 0; i < maxIter; i++) {
        std::vector<physics::BodyHandle> collisions;
        collisions.reserve(100);
        collisions = physics->queryBodies(
            aabb,
            physics::BodySet::Asleep
        );

        if (collisions.size() == 0) {
            break;
        }

        // min depth collision
        float min = std::numeric_limits<float>::max();
        std::optional<physics::AABB> minDepthBounds;
        for (physics::BodyHandle& handle : collisions) {
            const std::optional<physics::BodyState> bodyState =
                physics->getRigidBodyState(handle);

            if (!bodyState || bodyState->colliders.empty()) {
                continue;
            }

            const std::optional<physics::ColliderState> colliderState =
                physics->getColliderState(
                    bodyState->colliders[0]
                );

            if (!colliderState) {
                continue;
            }

            float depth =
                aabb.getMinOverlapDepth(colliderState->bounds);
            if (depth < min) {
                min = depth;
                minDepthBounds = colliderState->bounds;
            }
        }

        if (!minDepthBounds) {
            break;
        }

        // move AABB away from collision
        glm::vec3 normal =
            aabb.getCollisionNormal(*minDepthBounds);

        aabb.worldCenter += normal * min * 1.2f;
        aabb.worldMin = aabb.worldCenter - aabb.worldHalfExtents;
        aabb.worldMax = aabb.worldCenter + aabb.worldHalfExtents;

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
}

physics::RaycastHit Player::raycast(float length) {
    float rLength = length;
    physics::Ray r(camera->position, camera->front, rLength);
    physics::RaycastHit hitData = physics->raycast(
        r,
        getPlayerRigidBodyHandle()
    );
    return hitData;
}
