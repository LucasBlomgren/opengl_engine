#include "game_object.h"

AABB GameObject::getAABB() const {
    return aabb;
}

void GameObject::resetDirtyFlags() {
    modelMatrixDirty = true;
    helperMatrixesDirty = true;
    aabbDirty = true;

    aabb.facesDirty = true;

}

void GameObject::setRotatedFlag() {
    if (colliderType != ColliderType::SPHERE) {
        const float eps = 1e-5f;
        glm::quat identity{ 1, 0, 0, 0 };
        bool isIdentity =
            glm::epsilonEqual(orientation.w, identity.w, eps) &&
            glm::epsilonEqual(orientation.x, identity.x, eps) &&
            glm::epsilonEqual(orientation.y, identity.y, eps) &&
            glm::epsilonEqual(orientation.z, identity.z, eps);
        isRotated = !isIdentity;
    }
}

void GameObject::setPhysicsVariables() {
    // dynamic
    if (!isStatic) {
        invMass = 1.0f / mass;
        asleep = false;
    }
    // static
    else {
        mass = 0;
        invMass = 0;
        inverseInertia = glm::mat3(0.0f);
        asleep = true;
    }
}

void GameObject::applyForceLinear(glm::vec3 f) {
    linearVelocity += f;
}
void GameObject::applyForceAngular(glm::vec3 f) {
    angularVelocity += f;
}

void GameObject::setModelMatrix() {
    if (!modelMatrixDirty)
        return;

    modelMatrix = glm::mat4(1.0f); 
    modelMatrix = glm::translate(modelMatrix, position); 
    modelMatrix *= glm::mat4_cast(orientation); 
    modelMatrix = glm::scale(modelMatrix, scale); 
    modelMatrixDirty = false; 
}

void GameObject::setHelperMatrixes() {
    invModelMatrix = glm::inverse(modelMatrix);
    rotationMatrix = glm::mat3_cast(orientation);  // enbart ortonormal rotation
    invRotationMatrix = glm::transpose(rotationMatrix);
    inverseInertiaWorld = rotationMatrix * inverseInertia * invRotationMatrix;

    helperMatrixesDirty = false;
}

void GameObject::calculateInverseInertiaForCube() {
    float I = 6.0f / (mass * scale.x * scale.x);

    inverseInertia = glm::mat3(
        glm::vec3(I, 0.0f, 0.0f),
        glm::vec3(0.0f, I, 0.0f),
        glm::vec3(0.0f, 0.0f, I)
    );
}

void GameObject::calculateInverseInertiaForCuboid() {
   float I_x = (1.0f / 12.0f) * mass * (scale.y * scale.y + scale.z * scale.z);
   float I_y = (1.0f / 12.0f) * mass * (scale.x * scale.x + scale.z * scale.z);
   float I_z = (1.0f / 12.0f) * mass * (scale.x * scale.x + scale.y * scale.y);

   inverseInertia = glm::mat3(
      glm::vec3(1.0f / I_x, 0.0f, 0.0f),
      glm::vec3(0.0f, 1.0f / I_y, 0.0f),
      glm::vec3(0.0f, 0.0f, 1.0f / I_z));
}

void GameObject::calculateInverseInertiaForSolidSphere() {
    float I = (2.0f / 5.0f) * (mass * radius * radius);
    float invI = 1.0f / I;

    inverseInertia = glm::mat3(
        glm::vec3(invI, 0.0f, 0.0f),
        glm::vec3(0.0f, invI, 0.0f),
        glm::vec3(0.0f, 0.0f, invI));
}

void GameObject::updateOrientation(glm::quat& orientation, const glm::vec3& angularVelocity, float deltaTime) {
    glm::quat omegaQuat(0.0f, angularVelocity.x, angularVelocity.y, angularVelocity.z);
    orientation += 0.5f * deltaTime * (omegaQuat * orientation);
    orientation = glm::normalize(orientation);
}


void GameObject::updateAABB() {
    setModelMatrix();
    aabb.update(modelMatrix, position, scale, isRotated);
}

void GameObject::updateCollider() {
    setModelMatrix();

    std::visit([&](auto& shape) {
        using T = std::decay_t<decltype(shape)>;

        // OOBB
        if constexpr (std::is_same_v<T, OOBB>) {
            shape.update(modelMatrix);
        }
        // Sphere
        else if constexpr (std::is_same_v<T, Sphere>) {
            shape.update(modelMatrix);
        }
        // TriMesh
        else if constexpr (std::is_same_v<T, TriMesh>) {

        }
    }, collider.shape);
}

void GameObject::updatePos(const float& deltaTime) {
    if (sleepCounter >= sleepCounterThreshold) {
        setAsleep();
        return;
    }

    if (selectedByEditor)
        return;

    if (hasGravity)
        linearVelocity += g * deltaTime;

    linearVelocity *= std::pow(0.96f, deltaTime);
    angularVelocity *= std::pow(0.94f, deltaTime);

    if (!canMoveLinearly) {
        linearVelocity = glm::vec3(0.0f);
        angularVelocity.x = 0.0f;
        angularVelocity.y = 0.0f;
    }

    lastPosition = position;
    position += linearVelocity * deltaTime;
    updateOrientation(orientation, angularVelocity, deltaTime);

    setRotatedFlag();

    // sleep counter
    if (std::abs(glm::length(linearVelocity)) < velocityThreshold and std::abs(glm::length(angularVelocity)) < angularVelocityThreshold) 
        sleepCounter += deltaTime;
    else 
        sleepCounter = 0.0f;
}

void GameObject::setAsleep() {
    if (isStatic)
        return;

    linearVelocity = glm::vec3(0, 0, 0);
    angularVelocity = glm::vec3(0, 0, 0);

    asleep = true;
}

void GameObject::setAwake() {
    if (isStatic)
        return;

    asleep = false;
    sleepCounter = 0.0f;
}

void GameObject::drawMesh(Shader& shader) {
    setModelMatrix();
    shader.setMat4("model", modelMatrix);

    if (textureID != 999) {
        shader.setBool("useTexture", true);
        shader.setBool("useUniformColor", false);
    }
    else {
        shader.setBool("useTexture", false);
        shader.setBool("useUniformColor", true);
        shader.setVec3("uColor", this->color);
    }
    glBindTexture(GL_TEXTURE_2D, textureID);
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, vertices.size(), GL_UNSIGNED_INT, 0);
}

void GameObject::setupMesh() {
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
    // normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1);
    // texture coord attribute
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords));
    glEnableVertexAttribArray(2);
}