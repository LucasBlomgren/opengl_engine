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
#include "vertex.h"
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

    void setModelMatrix()
    {
        if (modelMatrixShouldUpdate == true) {
            modelMatrix = glm::mat4(1.0f);
            modelMatrix = glm::translate(modelMatrix, position);
            modelMatrix *= glm::mat4_cast(orientation);
            modelMatrix = glm::scale(modelMatrix, scale);
            modelMatrixShouldUpdate = false;
        }
    }

    void calculateInverseInertiaForCube() {
        float value = 6.0f / (mass * scale.x * scale.x);

        inverseInertia = glm::mat3(
            glm::vec3(value, 0.0f, 0.0f), // Row 1
            glm::vec3(0.0f, value, 0.0f), // Row 2
            glm::vec3(0.0f, 0.0f, value)  // Row 3
        );
    }

    void updateOrientation(glm::quat& orientation, const glm::vec3& angularVelocity, float deltaTime)
    {
        // 1. Gör en "omega-kvaternion" med w=0, (x,y,z)=angularVelocity
        glm::quat omegaQuat(0.0f, angularVelocity.x, angularVelocity.y, angularVelocity.z);

        // 2. Uppdatera orientation enligt explicit Euler:
        //    q_new = q_old + 0.5 * dt * (omegaQuat * q_old)
        orientation += 0.5f * deltaTime * (omegaQuat * orientation);

        // 3. Normalisera för att bibehålla enhetskvaternion
        orientation = glm::normalize(orientation);
    }
    
    void drawMesh(Shader& shader)
    {
        setModelMatrix();

        shader.setMat4("model", modelMatrix);
        shader.setBool("useTexture", true);
        shader.setBool("useUniformColor", false);

        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
    }

    void updateAABB() 
    {
        setModelMatrix();

        if ((AABB_ShouldUpdate and !asleep) or isStatic) 
            AABB.update(verticesPositions, modelMatrix, position, isUniformlyScaled);
    }

    void updateOOBB()
    {
        setModelMatrix();

        if (((linearVelocity != glm::vec3(0,0,0) or angularVelocity != glm::vec3(0,0,0)) and OOBB_shouldUpdate and !asleep) or isStatic) {
            OOBB.update(verticesPositions, modelMatrix);
            OOBB_shouldUpdate = false;
        }
    }

    void addForce(const glm::vec3& f)
    {
        force += f;
    }

    void updatePos(const float& num_iterations, const float& deltaTime)
    {
        modelMatrixShouldUpdate = true;

        float time = deltaTime / num_iterations;

        if (isStatic)
            return;

        if (hasGravity && !asleep)
            force += g;

        glm::vec3 summedLinearVelocity = linearVelocity + biasLinearVelocity;

        biasLinearVelocity = glm::vec3();

        linearVelocity += force * time;
        position += summedLinearVelocity * time;

        updateOrientation(orientation, angularVelocity, time);

        jitterMinRange = previousCollisionPoint - 0.2f;
        jitterMaxRange = previousCollisionPoint + 0.2f;

        isWithinRange = (position.x >= jitterMinRange.x && position.x <= jitterMaxRange.x) &&
                             (position.y >= jitterMinRange.y && position.y <= jitterMaxRange.y) &&
                             (position.z >= jitterMinRange.z && position.z <= jitterMaxRange.z);

        previousCollisionPoint = collisionPoint;

        if (isWithinRange)
            sleepCounter += deltaTime;
        else {
            sleepCounter = 0.0f;
            asleep = false;
        }

        force = glm::vec3();
    }

    void setAsleep(const float& deltaTime) 
    {
        if (sleepCounter > 10.0f) {
            asleep = true;
            linearVelocity = glm::vec3();
            angularVelocity = glm::vec3();
        }
    }

private:
    void setupMesh()
    {
        unsigned int VBO;
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);

        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

        unsigned int EBO;
        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position));
        glEnableVertexAttribArray(0);
        // color attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, color));
        glEnableVertexAttribArray(1);
        // normal attribute
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
        glEnableVertexAttribArray(2);
        // texture coord attribute
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords));
        glEnableVertexAttribArray(3);
    }
};