#pragma once

#include "pch.h"
#include "game/transform.h"

struct ColliderPose {
    glm::vec3 position{ 0.0f };
    glm::quat orientation{ 1,0,0,0 };
    glm::vec3 scale{ 1.0f };

    glm::mat3 rotationMatrix{ 1.0f };
    glm::mat3 invRotationMatrix{ 1.0f };

    glm::mat4 modelMatrix{ 1.0f };
    glm::mat4 invModelMatrix{ 1.0f };

    bool modelDirty = true;
    bool invModelDirty = true;
    bool rotationDirty = true;
    bool invRotationDirty = true;

    void markDirty() {
        modelDirty = true;
        invModelDirty = true;
        rotationDirty = true;
        invRotationDirty = true;
    }

    void ensureModelMatrix() {
        if (modelDirty) {
            glm::mat4 T = glm::translate(glm::mat4(1.0f), position);
            glm::mat4 R = glm::mat4_cast(orientation);
            glm::mat4 S = glm::scale(glm::mat4(1.0f), scale);
            modelMatrix = T * R * S;
            modelDirty = false;
        }
    }

    void ensureInvModelMatrix() {
        if (invModelDirty) {
            ensureModelMatrix();
            invModelMatrix = glm::inverse(modelMatrix);
            invModelDirty = false;
        }
    }

    void ensureRotationMatrix() {
        if (rotationDirty) {
            rotationMatrix = glm::mat3_cast(orientation);
            rotationDirty = false;
        }
    }

    void ensureInvRotationMatrix() {
        if (invRotationDirty) {
            ensureRotationMatrix();
            invRotationMatrix = glm::transpose(rotationMatrix);
            invRotationDirty = false;
        }
    }

    void combineIntoColliderPose(const Transform& root, const Transform& local) {
        glm::mat3 rootRot = glm::mat3_cast(root.orientation);

        position = root.position + rootRot * (root.scale * local.position);
        orientation = glm::normalize(root.orientation * local.orientation);
        scale = root.scale * local.scale;

        markDirty();
    }
};