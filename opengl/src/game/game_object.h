#pragma once

#include "pch.h"

#include "core/slot_map.h"
#include "shaders/shader.h"
#include "mesh/mesh.h"

struct SubPart {
    std::string name = "SubPart";
    TransformHandle localTransformHandle;
    ColliderHandle colliderHandle;

    // render
    Shader* shader = nullptr;
    Mesh* mesh = nullptr;
    GLuint textureId = 0;
    glm::vec3 color{ 1.0f }; // white
    bool seeThrough = false;
    int batchIdx = -1;
    int batchInstanceIdx = -1;
};

// #TODO: decide what is private/public
class GameObject {
public:
    int id;
    std::string name = "GameObject";
    TransformHandle rootTransformHandle;
    RigidBodyHandle rigidBodyHandle;

    std::vector<SubPart> parts;

    // to skip raycasting on player object
    bool player = false;

    // constructor
    GameObject(int id, TransformHandle rootTransformHandle)
        : id(id), rootTransformHandle(rootTransformHandle) {
    }

    ~GameObject() {}
};