#include "mesh.h"

void Mesh::setModelMatrix()
{
    if (modelMatrixShouldUpdate == true) {
        modelMatrix = glm::mat4(1.0f);
        modelMatrix = glm::translate(modelMatrix, position);
        modelMatrix *= glm::mat4_cast(orientation);
        modelMatrix = glm::scale(modelMatrix, scale);
        modelMatrixShouldUpdate = false;
    }
}

void Mesh::calculateInverseInertiaForCube() {
    float value = 6.0f / (mass * scale.x * scale.x);

    inverseInertia = glm::mat3(
        glm::vec3(value, 0.0f, 0.0f), // Row 1
        glm::vec3(0.0f, value, 0.0f), // Row 2
        glm::vec3(0.0f, 0.0f, value)  // Row 3
    );
}

void Mesh::updateOrientation(glm::quat& orientation, const glm::vec3& angularVelocity, float deltaTime)
{
    // 1. Gör en "omega-kvaternion" med w=0, (x,y,z)=angularVelocity
    glm::quat omegaQuat(0.0f, angularVelocity.x, angularVelocity.y, angularVelocity.z);

    // 2. Uppdatera orientation enligt explicit Euler:
    //    q_new = q_old + 0.5 * dt * (omegaQuat * q_old)
    orientation += 0.5f * deltaTime * (omegaQuat * orientation);

    // 3. Normalisera för att bibehålla enhetskvaternion
    orientation = glm::normalize(orientation);
}
    
void Mesh::drawMesh(Shader& shader)
{
    setModelMatrix();

    shader.setMat4("model", modelMatrix);
    shader.setBool("useTexture", true);
    shader.setBool("useUniformColor", false);

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
}

void Mesh::updateAABB()
{
    setModelMatrix();

    if ((AABB_ShouldUpdate and !asleep) or isStatic) 
        AABB.update(verticesPositions, modelMatrix, position, isUniformlyScaled);
}

void Mesh::updateOOBB()
{
    setModelMatrix();

    if (((linearVelocity != glm::vec3(0,0,0) or angularVelocity != glm::vec3(0,0,0)) and OOBB_shouldUpdate and !asleep) or isStatic) {
        OOBB.update(verticesPositions, modelMatrix);
        OOBB_shouldUpdate = false;
    }
}

void Mesh::addForce(const glm::vec3& f)
{
    force += f;
}

void Mesh::updatePos(const float& num_iterations, const float& deltaTime)
{
    modelMatrixShouldUpdate = true;

    float time = deltaTime / num_iterations;

    if (isStatic)
        return;

    if (hasGravity && !asleep)
        force += g;

    glm::vec3 summedLinearVelocity = linearVelocity + biasLinearVelocity;

    biasLinearVelocity = glm::vec3();

    linearVelocity += force * time;
    position += summedLinearVelocity * time;

    updateOrientation(orientation, angularVelocity, time);

    jitterMinRange = previousCollisionPoint - 0.2f;
    jitterMaxRange = previousCollisionPoint + 0.2f;

    isWithinRange = (position.x >= jitterMinRange.x && position.x <= jitterMaxRange.x) &&
                            (position.y >= jitterMinRange.y && position.y <= jitterMaxRange.y) &&
                            (position.z >= jitterMinRange.z && position.z <= jitterMaxRange.z);

    previousCollisionPoint = collisionPoint;

    if (isWithinRange)
        sleepCounter += deltaTime;
    else {
        sleepCounter = 0.0f;
        asleep = false;
    }

    force = glm::vec3();
}

void Mesh::setAsleep(const float& deltaTime)
{
    if (sleepCounter > 10.0f) {
        asleep = true;
        linearVelocity = glm::vec3();
        angularVelocity = glm::vec3();
    }
}

void Mesh::setupMesh()
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