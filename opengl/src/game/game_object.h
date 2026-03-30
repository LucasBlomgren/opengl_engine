#pragma once

#include "pch.h"

#define GLM_ENABLE_EXPERIMENTAL

#include "world.h"
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

inline bool approxEqual(float a, float b, float epsilon = 0.0001f) {
    return fabs(a - b) < epsilon;
}

// #TODO: decide what is private/public in GameObject
class GameObject {
public:
    int id;
    Transform transform;
    ColliderHandle colliderHandle;
    RigidBodyHandle rigidBodyHandle;

    // render
    Shader* shader = nullptr;
    Mesh* mesh = nullptr;
    GLuint textureId;
    glm::vec3 color;
    bool useRandomColor = false;
    bool seeThrough = false;
    bool isInsideShadowFrustum = true;
    int batchIdx = -1;
    int batchInstanceIdx = -1;

    // editor/player interaction
    bool player = false;
    bool hoveredByEditor = false;
    bool selectedByEditor = false;
    bool selectedByPlayer = false;

    std::vector<glm::vec3> verticesPositions;

    // constructor
    GameObject(
        int id,
        Transform transform,
        Shader* shader,
        Mesh* mesh,
        int textureId,
        glm::vec3 color = glm::vec3(255.0f, 255.0f, 255.0f),
        bool seeThrough = false
    )
        : id(id),
        transform(transform),
        shader(shader),
        mesh(mesh),
        textureId(textureId),
        color(color),
        seeThrough(seeThrough)
    {
        // plain color
        this->color = color / 255.0f;
    }

    ~GameObject() {}
};