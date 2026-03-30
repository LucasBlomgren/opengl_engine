#include "pch.h"
#include "player.h"

void Player::setPointers(World* world, PhysicsEngine* physicsEngine, Camera* camera) {
    this->world = world;
    this->physicsEngine = physicsEngine;
    this->camera = camera;
}


//------------------------------------------------------
// INPUT HANDLING
//-------------------------------------------------------
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
            moveSelectedObject(1.0f / 60.0f); // hardcoded timestep for smoother movement
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
            if (onGround and !hasJumped) {
                jumpImpulse += JUMP_HEIGHT;
                onGround = false;
                hasJumped = true;
                c.keyboard = true;
            }
        }
    }
}


//------------------------------------------------------
// SWITCHING PLAYER MODE
//-------------------------------------------------------
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
    player->player = true;

    RigidBody* rb = world->getRigidBody(playerHandle);
    rb->responseMode = ContactResponseMode::Character;
    rb->setExternalControl(true);
    rb->allowSleep = false;
}

void Player::destroyPlayerObject() {
    GameObject* player = world->getGameObject(playerHandle);
    physicsEngine->queueRemove(player->colliderHandle);
    world->deleteGameObject(playerHandle);
    playerHandle = GameObjectHandle{};
}


//----------------------------------------------------------
// MOVEMENT, CAMERA UPDATE and OBJECT SELECTION UPDATE
//----------------------------------------------------------
void Player::updateMovement() {
    Transform& t = world->getGameObject(playerHandle)->transform;

    // first person camera
    camera->position = t.position + glm::vec3(0, 0.76f, 0);

    // third person camera
    //camera->position = player.position - camera->front * glm::vec3(12.0f) + glm::vec3(0, 0.76f, 0);

    // avoid diagonal speed boost
    if (glm::length2(moveInput) > 0.0f) {
        moveInput = glm::normalize(moveInput);
    }
    moveImpulse = moveInput * MOVE_SPEED;

    moveSelectedObject(1.0f / 60.0f);
}

void Player::updateObjectSelection(Shader& shader) {
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
        if (!hoveredObj) {
            std::cout << "Warning: hovered object was nullptr" << std::endl;
            hoveredObjectHandle = GameObjectHandle{};
            objectIsHovered = false;
            return;
        }
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
    }
}

//------------------------------------------------------
// PHYSICS
//-------------------------------------------------------
void Player::updateBody(float dt) {
    GameObject* player = world->getGameObject(playerHandle);
    RigidBody* rb = world->getRigidBody(playerHandle);
    Transform& t = player->transform;

    glm::vec3 v = rb->linearVelocity;

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


    // stop horizontal movement when on ground and no input to prevent sliding on slopes and when landing from a jump
    if (onGround and glm::length2(moveImpulse) < 0.01f) {
        v.x = 0.0f;
        v.z = 0.0f;
    }

    // #TODO: should be using rigidbody functions instead
    rb->linearVelocity = v;
    t.lastPosition = t.position;
    t.position += rb->linearVelocity * dt;
}

void Player::resolveExternalContact() {
    onGround = false;

    const auto& contacts = physicsEngine->getExternalMotionContacts();
    GameObject* playerObj = world->getGameObject(playerHandle);

    for (const ExternalMotionContact& c : contacts) {
        bool playerIsA = false;
        bool playerInContact = false;

        if (c.bodyA == playerObj->rigidBodyHandle && c.bodyA.isValid()) {
            playerIsA = true;
            playerInContact = true;
        }
        else if (c.bodyB == playerObj->rigidBodyHandle && c.bodyB.isValid()) {
            playerIsA = false;
            playerInContact = true;
        }

        if (!playerInContact) continue;

        glm::vec3 up;
        if (playerIsA) {
            playerObj->transform.position -= c.normal * (c.penetration + 0.001f);
            up = glm::vec3(0.0f, -1.0f, 0.0f);
        }
        else {
            playerObj->transform.position += c.normal * (c.penetration + 0.001f);
            up = glm::vec3(0.0f, 1.0f, 0.0f);
        }

        if (glm::dot(up, c.normal) > ON_GROUND_ANGLE_THRESHOLD) {
            onGround = true;
        }
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
    if (!objectIsSelected) return;

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

//------------------------------------------------------
// OBJECT SELECTION AND PLACEMENT
//------------------------------------------------------
// 
// Select object under crosshair
void Player::selectObject() {
    if (objectIsSelected) return;

    RaycastHit raycast = rayCast(SELECT_RANGE);

    // no hit return
    if (!raycast.hit) return;

    Collider* selectedCollider = world->getCollider(raycast.colliderHandle);
    RigidBody* rb = world->getRigidBody(selectedCollider->rigidBodyHandle);

    if (rb->type == BodyType::Static) return;

    objectIsSelected = true;
    GameObjectHandle handle = selectedCollider->gameObjectHandle;
    selectedObjectHandle = handle;

    GameObject* obj = world->getGameObject(handle);
    obj->selectedByPlayer = true;
    rb->asleep = false;
    rb->setExternalControl(true);

    BroadphaseBucket bpBucket;
    bpBucket = BroadphaseBucket::Awake;
    physicsEngine->queueMove(raycast.colliderHandle, bpBucket);

    rb->allowSleep = false;
    obj->transform.lastPosition = obj->transform.position;

    rb->linearVelocity = glm::vec3(0.0f);
    rb->angularVelocity = glm::vec3(0.0f);

    // Project worldofset onto cameras local axes
    glm::vec3 worldOffset = obj->transform.position - camera->position;
    selectionOffsetLocal.x = glm::dot(worldOffset, camera->right);
    selectionOffsetLocal.y = glm::dot(worldOffset, camera->up);
    selectionOffsetLocal.z = glm::dot(worldOffset, camera->front);
}

void Player::dropObject() {
    if (objectIsSelected) {
        GameObject* selectedObject = world->getGameObject(selectedObjectHandle);
        selectedObject->selectedByPlayer = false;

        RigidBody* rb = world->getRigidBody(selectedObjectHandle);
        rb->setExternalControl(false);
    }

    objectIsSelected = false;
    objectIsHovered = false;

    selectedObjectHandle = {};
    hoveredObjectHandle = {};
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