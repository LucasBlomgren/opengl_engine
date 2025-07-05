#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/component_wise.hpp>

#include "shader.h"
#include "colliders/collider.h"
#include "colliders/aabb.h"
#include "colliders/oobb.h"
#include "colliders/sphere.h"
#include "colliders/tri_mesh.h"
#include "oobb_renderer.h"
#include "aabb_renderer.h"

inline bool approxEqual(float a, float b, float epsilon = 0.0001f) {
    return fabs(a - b) < epsilon;
}

struct CollisionHistory {
   static constexpr int SLOTS = 3;
   int buffer[SLOTS] = { 0 };
   int index = 0;
   int count = 0;

   // Ska köras varje frame med det nya värdet:
   void push(int newCount) {
      buffer[index] = newCount;
      index = (index + 1) % SLOTS;
      if (count < SLOTS) ++count;
   }

   // Ger medelvärdet över de senast count värdena (upp till 5)
   float average() const {
      if (count == 0) return 0.0f;
      
      int sum = 0;
      for (int i = 0; i < count; ++i) {
         sum += buffer[i];
      }
      return (float)sum / (float)count;
   }
};

class GameObject {
public:
   // mesh variables
    int id;
    glm::vec3 position;
    std::vector<Vertex> vertices;
    std::vector<glm::vec3> verticesPositions;
    std::vector<unsigned int> indices;
    unsigned int VAO;

    glm::mat4 modelMatrix;
    bool modelMatrixDirty = true;

    glm::mat4 invModelMatrix;
    glm::mat3 rotationMatrix;
    glm::mat3 invRotationMatrix;
    bool helperMatrixesDirty = true;

    int textureID;
    glm::vec3 color;

    // physics variables
    glm::quat orientation;
    glm::mat3 inverseInertia;
    glm::mat3 inverseInertiaWorld;
    glm::vec3 scale;
    glm::vec3 linearVelocity{ 0.0f };
    glm::vec3 angularVelocity{ 0.0f };
    glm::vec3 biasLinearVelocity;
    glm::vec3 biasAngularVelocity;
    float linearVelocityLen;
    float angularVelocityLen;

    float mass;
    float invMass;
    float invInertia;
    bool hasGravity;
    bool isStatic;
    bool shouldRotate;
    bool isRotated = false;
    bool canMoveLinearly = true;
    bool isUniformlyScaled;
    glm::vec3 g = glm::vec3(0.0f, -9.81f, 0.0f);
    float radius;
    float invRadius;

    // sleep variables
    bool asleep = false;
    float sleepCounter = 0;
    float sleepCounterThreshold;
    float velocityThreshold = 2;
    float angularVelocityThreshold = 3;

    int totalCollisionCount = 0;
    float lastAvg = 0.0f;
    CollisionHistory collisionHistory;

    // collision variables
    AABB aabb;
    bool aabbDirty = true;
    Collider collider;
    ColliderType colliderType;
    OOBBRenderer oobbRenderer;

    // editor variables
    bool selectedByEditor = false;
    bool isRaycastHit = false;
    glm::vec3 lastPosition;
    glm::vec3 pushCorrection;
    
    // constructor
    GameObject(
        int id,
        std::vector<Vertex> vertices,
        std::vector<unsigned int> indices,
        glm::vec3 position,
        glm::vec3 scale,
        ColliderType colliderType,
        float mass,
        bool isStatic,
        int textureID,
        glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        float sleepCounterThreshold = 1.0f,
        bool asleep = false,
        glm::vec3 color = glm::vec3(255.0f, 255.0f, 255.0f)
    )
        : id(id),
        vertices(vertices),
        indices(indices),
        position(position),
        shouldRotate(true),
        scale(scale),
        hasGravity(true),
        isStatic(isStatic),
        mass(mass),
        textureID(textureID),
        orientation(orientation),
        sleepCounterThreshold(sleepCounterThreshold),
        asleep(asleep),
        color(color),
        colliderType(colliderType),
        collider(this)
    {
        // physics stuff
        isUniformlyScaled = approxEqual(scale.x, scale.y) && approxEqual(scale.y, scale.z);
        if (orientation != glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) { isRotated = true; }
        invRadius = 1.0f / (0.5f * glm::length(scale));

        // dynamic
        if (!isStatic) {
            invMass = 1.0f / mass;
            asleep = false;
        }
        // static
        else {
            mass = 0;
            invMass = 0;
            asleep = true;
        }

        // plain color
        if (textureID == 999)
           this->color = color / 255.0f;

        for (const Vertex& vertex : this->vertices)
            verticesPositions.push_back(vertex.position);

        setModelMatrix();

        // render mesh
        setupMesh();

        // AABB 
        aabb.Init(verticesPositions);
        aabb.update(modelMatrix, position, scale, isRotated);

        // Collider
        if (colliderType == ColliderType::CUBOID) {
            OOBB oobb(verticesPositions, modelMatrix);
            collider.shape = oobb;

            oobbRenderer.setupWireframeBox();
            oobbRenderer.setupNormals();

            if (isStatic) {
                inverseInertia = glm::mat3(0.0f);
            }
            else {
                if (this->isUniformlyScaled)
                    calculateInverseInertiaForCube();
                else
                    calculateInverseInertiaForCuboid();
            }
        }

        else if (colliderType == ColliderType::SPHERE) {
            Sphere sphere; 
            radius = scale.x;
            sphere.radius = scale.x;   // Assuming uniform scaling for sphere
            collider.shape = sphere; 

            if (isStatic) {
                inverseInertia = glm::mat3(0.0f); 
            }
            else {
                calculateInverseInertiaForSolidSphere();
            }
        }

        else if (colliderType == ColliderType::MESH) {
            std::vector<Tri> worldTris;
            worldTris.reserve(verticesPositions.size() / 3);

            for (size_t i = 0; i < verticesPositions.size(); i += 3) {
                glm::vec4 v0 = modelMatrix * glm::vec4(verticesPositions[i + 0], 1.0f);
                glm::vec4 v1 = modelMatrix * glm::vec4(verticesPositions[i + 1], 1.0f);
                glm::vec4 v2 = modelMatrix * glm::vec4(verticesPositions[i + 2], 1.0f);
                worldTris.emplace_back(glm::vec3(v0), glm::vec3(v1), glm::vec3(v2));
            }

            collider.shape.emplace<TriMesh>(worldTris);
        }

        inverseInertiaWorld = inverseInertia; 
    }

    void setPhysicsVariables();

    void resetDirtyFlags();
    void setModelMatrix();
    void setHelperMatrixes();
    void calculateInverseInertiaForCube();
    void calculateInverseInertiaForCuboid();
    void calculateInverseInertiaForSolidSphere();
    void updateOrientation(glm::quat& orientation, const glm::vec3& angularVelocity, float deltaTime);
    void drawMesh(Shader& shader);
    void updateAABB();
    void updateCollider();
    void updatePos(const float& deltaTime);
    void setAsleep();
    void setAwake();

    void applyForceLinear(glm::vec3 f);
    void applyForceAngular(glm::vec3 f);

    AABB getAABB() const;

private:
    void setupMesh();
};