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
#include "AABB.h"
#include "OOBB.h"

inline bool approxEqual(float a, float b, float epsilon = 0.0001f) {
    return fabs(a - b) < epsilon;
}

class Mesh
{
public:
    int id;
    bool colliding;
    glm::vec3 position;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    unsigned int VAO;
    glm::mat4 modelMatrix;
    bool modelMatrixShouldUpdate = true;

    std::vector<glm::vec3> verticesPositions;

    glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    glm::mat3 inverseInertia;

    glm::vec3 scale;
    glm::vec3 linearVelocity;
    glm::vec3 angularVelocity;
    glm::vec3 biasLinearVelocity;
    glm::vec3 biasAngularVelocity;

    glm::vec3 force;
    float staticFriction;
    float dynamicFriction;
    float restitution;
    float mass;
    float invMass;
    float invInertia;
    bool hasGravity;
    bool isStatic;
    bool shouldRotate;
    glm::vec3 g = glm::vec3(0.0f, -90.82f, 0.0f);

    bool asleep = false;
    float sleepCounter = 0;
    glm::vec3 collisionPoint;
    glm::vec3 previousCollisionPoint;
    glm::vec3 jitterMinRange;
    glm::vec3 jitterMaxRange;

    AABB AABB;
    OOBB OOBB;

    bool isUniformlyScaled;
    bool AABB_ShouldUpdate = true;
    bool OOBB_shouldUpdate = false;

    bool isWithinRange = false;

    bool floorTexture = false;

    Mesh(int id, std::vector<Vertex> vertices, std::vector<unsigned int> indices, glm::vec3 position, glm::vec3 scale, float mass, bool isStatic, bool floorTexture)
        : id(id),
        AABB(id),
        vertices(vertices),
        indices(indices),
        position(position),
        colliding(false),
        shouldRotate(true),
        scale(scale),
        hasGravity(true),
        isStatic(isStatic),
        mass(mass),
        restitution(0.1f),
        floorTexture(floorTexture)
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
        AABB.Init(verticesPositions, scale);
        OOBB.Init(verticesPositions);

        this->setupMesh();
    }

    void setModelMatrix();
    void calculateInverseInertiaForCube();
    void updateOrientation(glm::quat& orientation, const glm::vec3& angularVelocity, float deltaTime);
    void drawMesh(Shader& shader);
    void updateAABB();
    void updateOOBB();
    void addForce(const glm::vec3& f);
    void updatePos(const float& num_iterations, const float& deltaTime);
    void setAsleep(const float& deltaTime);

private:
    void setupMesh();
};