#pragma once

#include "shader.h"
#include "aabb.h"

class AABBRenderer {
public:
    static void InitShared();
    static void CleanupShared();

    void updateModel(const AABB& box, const bool asleep);
    void render(const glm::vec3& color, Shader& shader) const;

    glm::vec3 color{ 0.9f, 0.7f, 0.2f };
    glm::mat4 model{ 1.0f };

    static unsigned int sVAO, sVBO;

private:
    static bool         sInitialized;
};