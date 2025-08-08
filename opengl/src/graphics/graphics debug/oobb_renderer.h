#pragma once

#include "shader.h"

class OOBBRenderer { 
public:
    void setupWireframeBox();
    void renderBox(Shader& shader, glm::mat4& model, const bool asleep, const bool raycastHit);

    void setupNormals();
    void renderNormals(Shader& shader, const glm::mat4& model);

    unsigned int VAO_box, VBO_box;
    unsigned int VAO_normals, VBO_normals;
};