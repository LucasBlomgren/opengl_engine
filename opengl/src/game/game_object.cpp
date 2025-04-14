#include "game_object.h"

void GameObject::setModelMatrix()
{
    if (modelMatrixShouldUpdate) {
        modelMatrix = glm::mat4(1.0f);
        modelMatrix = glm::translate(modelMatrix, position);
        modelMatrix *= glm::mat4_cast(orientation);
        modelMatrix = glm::scale(modelMatrix, scale);
        modelMatrixShouldUpdate = false;
    }
}

void GameObject::calculateInverseInertiaForCube() {
    float value = 6.0f / (mass * scale.x * scale.x);

    inverseInertia = glm::mat3(
        glm::vec3(value, 0.0f, 0.0f), // Row 1
        glm::vec3(0.0f, value, 0.0f), // Row 2
        glm::vec3(0.0f, 0.0f, value)  // Row 3
    );
}

void GameObject::updateOrientation(glm::quat& orientation, const glm::vec3& angularVelocity, float deltaTime)
{
    glm::quat omegaQuat(0.0f, angularVelocity.x, angularVelocity.y, angularVelocity.z);
    orientation += 0.5f * deltaTime * (omegaQuat * orientation);
    orientation = glm::normalize(orientation);
}

void GameObject::drawMesh(Shader& shader)
{
    setModelMatrix();

    shader.setMat4("model", modelMatrix);
    shader.setBool("useTexture", true);
    shader.setBool("useUniformColor", false);

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
}

void GameObject::updateAABB()
{
    setModelMatrix();

    if ((AABB_ShouldUpdate and !asleep) or isStatic) 
        AABB.update(verticesPositions, modelMatrix, position, isUniformlyScaled);
}

void GameObject::updateOOBB()
{
    setModelMatrix();

    if (OOBB_shouldUpdate and !asleep) 
    {
        OOBB.update(verticesPositions, modelMatrix, OOBB_shouldUpdateBuffer);
        OOBB_shouldUpdate = false;
        OOBB_shouldUpdateBuffer = false;
    }
}

void GameObject::updatePos(const float& deltaTime)
{
    if (isStatic)
        return;

    if (sleepCounter > sleepCounterThreshold)
        setAsleep();

    if (asleep)
        return;

    modelMatrixShouldUpdate = true;

    if (hasGravity)
        linearVelocity += g * deltaTime;

    linearVelocity *= std::pow(0.96f, deltaTime);
    angularVelocity *= std::pow(0.94f, deltaTime);
    biasLinearVelocity *= std::pow(0.96f, deltaTime);

    glm::vec3 summedLinearVelocity = linearVelocity + biasLinearVelocity;
    biasLinearVelocity = glm::vec3();

    position += summedLinearVelocity * deltaTime;
    updateOrientation(orientation, angularVelocity, deltaTime);

    // sleep counter
    float velocityThreshold = 2;
    float angularVelocityThreshold = 2;
    if (std::abs(glm::length(summedLinearVelocity)) < velocityThreshold and std::abs(glm::length(angularVelocity)) < angularVelocityThreshold) {
        sleepCounter += deltaTime;
    }
    else {
        sleepCounter = 0.0f;
    }
}

void GameObject::setAsleep()
{
    OOBB_shouldUpdate = true;
    OOBB_shouldUpdateBuffer = true;
    updateOOBB();

    linearVelocity = glm::vec3(0, 0, 0);
    angularVelocity = glm::vec3(0, 0, 0);
    biasLinearVelocity = glm::vec3(0, 0, 0);

    asleep = true;
}

void GameObject::setAwake() 
{
    if (isStatic)
        return;

    asleep = false;
    sleepCounter = 0.0f;
}

void GameObject::setupMesh()
{
    unsigned int VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    unsigned int EBO;
    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position));
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, color));
    glEnableVertexAttribArray(1);
    // normal attribute
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
    glEnableVertexAttribArray(2);
    // texture coord attribute
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords));
    glEnableVertexAttribArray(3);
}