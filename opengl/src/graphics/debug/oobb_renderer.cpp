#include "pch.h"
#include "oobb_renderer.h"

unsigned int OOBBRenderer::sVAO_box = 0;
unsigned int OOBBRenderer::sVBO_box = 0;
bool         OOBBRenderer::sInitialized_box = false;

unsigned int OOBBRenderer::sVAO_normals = 0;
unsigned int OOBBRenderer::sVBO_normals = 0;
bool         OOBBRenderer::sInitialized_normals = false;

void OOBBRenderer::renderBox(Shader& shader, const OOBB& box, const bool asleep, const bool isStatic, const bool selected, const bool hovered) const {
    glm::mat4 model(1.0f);
    makeOOBBModelMatrix(model, box);

    shader.setMat4("model", model);
    shader.setInt("debug.objectType", 0);
    if (selected) { shader.setVec3("debug.uColor", glm::vec3(0, 1, 0)); }
    else if (hovered) { shader.setVec3("debug.uColor", glm::vec3(1.0f, 0.65f, 0.0f)); }
    else if (isStatic) { shader.setVec3("debug.uColor", glm::vec3(0.30f, 0.95f, 0.50f)); }
    else if (!asleep) { shader.setVec3("debug.uColor", glm::vec3(1, 0, 0)); }
    else { shader.setVec3("debug.uColor", glm::vec3(0, 0, 1)); }

    glLineWidth(2.0f);
    glBindVertexArray(sVAO_box);
    glDrawArrays(GL_LINES, 0, 24);
}

// #TODO: optimize by passing model matrix directly to shader
void OOBBRenderer::makeOOBBModelMatrix(glm::mat4& M, const OOBB& box) const {
    M[0] = glm::vec4(box.wAxes[0] * (box.lHalfExtents.x * 2.0f * box.scale.x), 0.0f);
    M[1] = glm::vec4(box.wAxes[1] * (box.lHalfExtents.y * 2.0f * box.scale.y), 0.0f);
    M[2] = glm::vec4(box.wAxes[2] * (box.lHalfExtents.z * 2.0f * box.scale.z), 0.0f);

    M[3] = glm::vec4(box.wCenter, 1.0f);
}

void OOBBRenderer::renderNormals(Shader& shader, const glm::mat4& model) {
    shader.setMat4("model", model);
    glDrawArrays(GL_LINES, 0, 18);
}

void OOBBRenderer::init() {
    setupWireframeBox();
    setupNormals();
}

void OOBBRenderer::setupWireframeBox() {
    if (sInitialized_box) return;
    sInitialized_box = true;

    // lines
    float cubeWire[72] = {
        // bottom
       -0.5f, -0.5f, -0.5f,  -0.5f, -0.5f,  0.5f,
       -0.5f, -0.5f,  0.5f,   0.5f, -0.5f,  0.5f,
        0.5f, -0.5f,  0.5f,   0.5f, -0.5f, -0.5f,
        0.5f, -0.5f, -0.5f,  -0.5f, -0.5f, -0.5f,
        // top 
       -0.5f, 0.5f, -0.5f,   -0.5f, 0.5f,  0.5f,
       -0.5f, 0.5f,  0.5f,    0.5f, 0.5f,  0.5f,
        0.5f, 0.5f,  0.5f,    0.5f, 0.5f, -0.5f,
        0.5f, 0.5f, -0.5f,   -0.5f, 0.5f, -0.5f,
        // vertical
       -0.5f, -0.5f, -0.5f,  -0.5f, 0.5f, -0.5f,
        0.5f, -0.5f, -0.5f,   0.5f, 0.5f, -0.5f,
        0.5f, -0.5f,  0.5f,   0.5f, 0.5f,  0.5f,
       -0.5f, -0.5f,  0.5f,  -0.5f, 0.5f,  0.5f
    };

    glGenVertexArrays(1, &sVAO_box); glcount::incVAO();
    glGenBuffers(1, &sVBO_box); glcount::incVBO();

    glBindVertexArray(sVAO_box);
    glBindBuffer(GL_ARRAY_BUFFER, sVBO_box);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeWire), cubeWire, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void OOBBRenderer::setupNormals() {
    if (sInitialized_normals) return;
    sInitialized_normals = true;

    float L = 1.5f;

    float normals[36] = {
        // pos                // color
        0.0f,  0.0f,  0.0f,   1.0f, 0.0f, 0.0f,
        L,     0.0f,  0.0f,   1.0f, 0.0f, 0.0f,
        0.0f,  0.0f,  0.0f,   0.0f, 1.0f, 0.0f,
        0.0f,  L,     0.0f,   0.0f, 1.0f, 0.0f,
        0.0f,  0.0f,  0.0f,   0.0f, 0.0f, 1.0f,
        0.0f,  0.0f,  L,      0.0f, 0.0f, 1.0f,
    };

    glGenVertexArrays(1, &sVAO_normals); glcount::incVAO();
    glGenBuffers(1, &sVBO_normals); glcount::incVBO();

    glBindVertexArray(sVAO_normals);
    glBindBuffer(GL_ARRAY_BUFFER, sVBO_normals);
    glBufferData(GL_ARRAY_BUFFER, sizeof(normals), normals, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}


void OOBBRenderer::destroy() {
    if (sVBO_box) {
        glDeleteBuffers(1, &sVBO_box); 
        sVBO_box = 0; 
        glcount::decVBO();
    }
    if (sVAO_box) { 
        glDeleteVertexArrays(1, &sVAO_box); 
        sVAO_box = 0; 
        glcount::decVAO(); 
    }

    if (sVBO_normals) { 
        glDeleteBuffers(1, &sVBO_normals); 
        sVBO_normals = 0; 
        glcount::decVBO();
    }
    if (sVAO_normals) { 
        glDeleteVertexArrays(1, &sVAO_normals); 
        sVAO_normals = 0; 
        glcount::decVAO();
    }
}

