#pragma once

#include "shaders/shader.h"
#include "colliders/oobb.h"

class OOBBRenderer { 
public:
    void setupWireframeBox();
    void renderBox(
        Shader& shader, 
        const OOBB& box,
        const bool asleep, 
        const bool isStatic, 
        const bool selected,
        const bool hovered
    );

    void destroy();
    void initShared();
    void setupNormals();
    void renderNormals(Shader& shader, const glm::mat4& model);
    void makeOOBBModelMatrix(glm::mat4& M, const OOBB& box);

    static unsigned int sVAO_box, sVBO_box;
    static unsigned int sVAO_normals, sVBO_normals;

private:
    static bool sInitialized_box;
    static bool sInitialized_normals;
};