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
        const bool raycastHit
    );

    void setupNormals();
    void renderNormals(Shader& shader, const glm::mat4& model);
    void makeOOBBModelMatrix(glm::mat4& M, const OOBB& box);

    unsigned int VAO_box, VBO_box;
    unsigned int VAO_normals, VBO_normals;
};