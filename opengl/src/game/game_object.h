#pragma once

#include "pch.h"

#include "core/slot_map.h"
#include "shaders/shader.h"
#include "mesh/mesh.h"

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

// #TODO: decide what is private/public
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