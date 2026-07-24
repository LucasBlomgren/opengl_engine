#pragma once

#include "pch.h"

#include "physics/public/physics_handles.h"
#include "game/game_handles.h"

#include "graphics/shaders/shader.h"
#include "graphics/mesh/mesh.h"

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