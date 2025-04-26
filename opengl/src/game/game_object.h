#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/component_wise.hpp>

#include <iostream>
#include <vector>
#include <algorithm>
#include <set>
#include <array>

#include "shader.h"
#include "aabb.h"
#include "oobb.h"
#include "oobb_renderer.h"
#include "aabb_renderer.h"

inline bool approxEqual(float a, float b, float epsilon = 0.0001f) {
    return fabs(a - b) < epsilon;
}

class GameObject
{
public:
    int id;
    glm::vec3 position;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    unsigned int VAO;
    glm::mat4 modelMatrix;
    bool modelMatrixShouldUpdate = true;

    int textureID;

    std::vector<glm::vec3> verticesPositions;

    glm::quat orientation;
    glm::mat3 inverseInertia;

    glm::vec3 scale;
    glm::vec3 linearVelocity;
    glm::vec3 angularVelocity;
    glm::vec3 biasLinearVelocity;
    glm::vec3 biasAngularVelocity;

    float staticFriction;
    float dynamicFriction;
    float restitution;
    float mass;
    float invMass;
    float invInertia;
    bool hasGravity;
    bool isStatic;
    bool shouldRotate;
    bool isRotating = false;
    glm::vec3 g = glm::vec3(0.0f, -90.82f, 0.0f);

    bool asleep = false;
    float sleepCounter = 0;
    float sleepCounterThreshold;

    AABB AABB;
    OOBB OOBB;
    bool AABB_ShouldUpdate = true;
    bool OOBB_shouldUpdate = false;
    bool OOBB_shouldUpdateBuffer = false;
    OOBBRenderer oobbRenderer;
    AABBRenderer aabbRenderer;

    bool isUniformlyScaled;
    bool selectedByEditor = false;
    glm::vec3 lastPosition;
    glm::vec3 pushCorrection;

    GameObject(int id, std::vector<Vertex> vertices, std::vector<unsigned int> indices, glm::vec3 position, glm::vec3 scale, float mass, bool isStatic, int textureID, glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f), float sleepCounterThreshold = 0.5f)
        : id(id),
        AABB(id),
        vertices(vertices),
        indices(indices),
        position(position),
        shouldRotate(true),
        scale(scale),
        hasGravity(true),
        isStatic(isStatic),
        mass(mass),
        restitution(0.1f),
        textureID(textureID),
        orientation(orientation),
        sleepCounterThreshold(sleepCounterThreshold)
    {
        isUniformlyScaled = approxEqual(scale.x, scale.y) && approxEqual(scale.y, scale.z);

        if (!isStatic) {
            calculateInverseInertiaForCube();
            invMass = 1.0 / mass;
            asleep = false;
        }
        else {
            mass = 0;
            invMass = 0;
            inverseInertia = glm::mat3(0.0f);
            asleep = true;
        }

        for (const Vertex& vertex : this->vertices)
            verticesPositions.push_back(vertex.position);

        setModelMatrix();
        AABB.Init(verticesPositions);
        AABB.update(modelMatrix, position, scale, isRotating);
        OOBB.Init(verticesPositions, modelMatrix);

        aabbRenderer.setupWireframeBox(AABB);
        oobbRenderer.setupWireframeBox();
        oobbRenderer.setupNormals();

        this->setupMesh();
    }

    void setModelMatrix();
    void calculateInverseInertiaForCube();
    void updateOrientation(glm::quat& orientation, const glm::vec3& angularVelocity, float deltaTime);
    void drawMesh(Shader& shader);
    void updateAABB();
    void updateOOBB();
    void updatePos(const float& deltaTime);
    void setAsleep();
    void setAwake();

private:
    void setupMesh();
};