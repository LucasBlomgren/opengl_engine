#pragma once

#include "graphics/shaders/shader.h"
#include "physics/physics.h"

class OOBBRenderer { 
public:
    void setupWireframeBox();
    void renderBox(
        Shader& shader,
        const physics::Pose& pose,
        const physics::BoxGeometry& box,
        const glm::vec3& color
    );

    void destroy();
    void init();
    void setupNormals();
    void renderNormals(Shader& shader, const glm::mat4& model);

    static unsigned int sVAO_box, sVBO_box;
    static unsigned int sVAO_normals, sVBO_normals;

private:
    void makeOOBBModelMatrix(
        glm::mat4& model,
        const physics::BoxGeometry& box,
        const physics::Pose& worldPose
    );

    static bool sInitialized_box;
    static bool sInitialized_normals;
};
