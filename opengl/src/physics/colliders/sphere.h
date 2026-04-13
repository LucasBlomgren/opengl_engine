#pragma once
#include <vector>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

class Sphere {
public:
    glm::vec3 centerWorld{ 0.0f };
    glm::vec3 centerLocal{ 0.0f };
    float     radiusWorld = 0.0f;

    Sphere(const ColliderPose& pose) {
        centerWorld = glm::vec3(pose.modelMatrix * glm::vec4(centerLocal, 1.0f));
        radiusWorld = pose.scale.x; // sphere mesh has diameter = 2 [-1.0f, 1.0f] in local space
    }

    void update(const ColliderPose& pose) {
        centerWorld = glm::vec3(pose.modelMatrix * glm::vec4(centerLocal, 1.0f));
        radiusWorld = pose.scale.x;
    }
};