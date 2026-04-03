#pragma once

#include <vector>

#include "scene_builder.h"
#include "physics.h"
#include "game_object.h"
#include "camera.h"

class Player : public IInputReceiver {
public:
    GameObjectHandle playerHandle;

    void handleInput(const InputFrame& in, const InputContext& ctx, Consumed& consumed, FrameWants& wants);
    void setPointers(World* world, PhysicsEngine* physicsEngine, Camera* camera);

    // activate/deactivate player mode
    void activate();
    void deactivate();
    void resetState();

    // create/destroy player object
    void createPlayerObject();
    void destroyPlayerObject();

    // update
    void updateObjectSelection(Shader& shader);
    void updateMovement();
    void updateBody(float fixedTimeStep);
    void resolveExternalContact();

    // select object
    bool objectIsSelected = false;
    bool objectIsHovered = false;
    GameObjectHandle selectedObjectHandle;
    GameObjectHandle hoveredObjectHandle;
    glm::vec3 selectionOffsetLocal{ 0.0f, 0.0f, 0.0f };
    void selectObject();
    void dropObject();
    void moveSelectedObject(float dt);

    // raycast for placement/selection/etc
    RaycastHit rayCast(float length);
    AABB aabbToPlace;
    bool placementObstructed = true;
    void placeObject();
    void createPlaceObjectAABB(Shader& shader);
    void drawAABB(const AABB& aabb, Shader& shader, glm::vec3 color = { 0.9f,0.7f,0.2f });

private:
    World* world = nullptr;
    PhysicsEngine* physicsEngine = nullptr;
    Camera* camera = nullptr;

    // movement
    bool onGround = false;
    bool hasJumped = false;
    glm::vec3 moveInput{ 0.0f, 0.0f, 0.0f };
    glm::vec3 moveImpulse{ 0.0f, 0.0f, 0.0f };
    float jumpImpulse = 0.0f;

    constexpr static float MOVE_ACCELERATION = 3.0f; // meters per second
    constexpr static float MAX_MOVE_SPEED = 15.0f;
    constexpr static float JUMP_HEIGHT = 10.5f;
    constexpr static float SHOOT_VELOCITY = 100.0f;
    constexpr static float ON_GROUND_ANGLE_THRESHOLD = 0.7f; // cos(45 degrees)
    constexpr static float GRAVITY = -9.81f;    
    constexpr static float GRAVITY_MULTIPLIER = 3.0f;
    constexpr static float AIR_FRICTION = 0.02f;

    // selection and placement
    bool dragging = false;
    bool pendingDrop = false;
    bool drawPlacementAABB = false;
    constexpr static float OBJ_PLACE_DISTANCE = 150.0f;
    constexpr static glm::vec3 OBJ_PLACE_SIZE{ 1.0f, 1.0f, 1.0f };
    constexpr static float SELECT_RANGE = 100.0f;
};