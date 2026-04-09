#pragma once

#include "pch.h"
#include "transform.h"

inline Transform combineTransforms(const Transform& root, const Transform& local) {
    Transform out;

    out.orientation = root.orientation * local.orientation;
    out.scale = root.scale * local.scale;
    out.position = root.position + root.rotationMatrix * (root.scale * local.position);
    out.rotationMatrix = glm::mat3_cast(out.orientation);

    glm::mat4 T = glm::translate(glm::mat4(1.0f), out.position);
    glm::mat4 R = glm::toMat4(out.orientation);
    glm::mat4 S = glm::scale(glm::mat4(1.0f), out.scale);
    out.modelMatrix = T * R * S;

    return out;
}