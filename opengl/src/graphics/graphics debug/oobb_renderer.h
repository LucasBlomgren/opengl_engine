#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "shader.h"

class OOBBRenderer {
public:
    void setupWireframeBox();
    void drawBox(Shader& shader, glm::mat4& model, const bool asleep, const bool raycastHit);

    void setupNormals();
    void drawNormals(Shader& shader, const glm::mat4& model);


private:
    unsigned int VAO_box, VBO_box;
    unsigned int VAO_normals, VBO_normals;
};