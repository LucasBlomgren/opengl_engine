#pragma once

#include "pch.h"

#define GLM_ENABLE_EXPERIMENTAL

#include "shaders/shader.h"
#include "mesh/mesh.h"
#include "colliders/collider.h"
#include "colliders/aabb.h"
#include "colliders/oobb.h"
#include "colliders/sphere.h"
#include "debug/oobb_renderer.h"
#include "debug/aabb_renderer.h"
#include "broadphase/broadphase_types.h"
#include "game/transform.h"

struct SubPart {
    TransformHandle localTransformHandle;
    ColliderHandle colliderHandle;

    // render
    Shader* shader = nullptr;
    Mesh* mesh = nullptr;
    GLuint textureId;
    glm::vec3 color;
    bool seeThrough = false;
    int batchIdx = -1;
    int batchInstanceIdx = -1;
};

// #TODO: decide what is private/public in GameObject
class GameObject {
public:
    int id;
    TransformHandle rootTransformHandle;
    RigidBodyHandle rigidBodyHandle;

    std::vector<SubPart> parts;

    // editor/player interaction
    bool player = false;
    bool hoveredByEditor = false;
    bool selectedByEditor = false;
    bool selectedByPlayer = false;

    // constructor
    GameObject(int id, TransformHandle rootTransformHandle)
        : id(id), rootTransformHandle(rootTransformHandle) {
    }

    ~GameObject() {}
};