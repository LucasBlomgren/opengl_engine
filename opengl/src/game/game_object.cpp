#include "pch.h"
#include "game_object.h"

AABB GameObject::getAABB() const {
    return aabb;
}

void GameObject::resetDirtyFlags() {
    modelMatrixDirty = true;
    helperMatricesDirty = true;
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

void GameObject::applyImpulseLinear(const glm::vec3& j) {
    linearVelocity += j;
}
void GameObject::applyImpulseAngular(const glm::vec3& j) {
    angularVelocity += j;
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

void GameObject::setHelperMatrices() {
    invModelMatrix = glm::inverse(modelMatrix);

    rotationMatrix = glm::mat3_cast(orientation);  // enbart ortonormal rotation
    translationVector = glm::vec3(modelMatrix[3]); // translationen från modelMatrix

    invRotationMatrix = glm::transpose(rotationMatrix);
    inverseInertiaWorld = rotationMatrix * inverseInertia * invRotationMatrix;

    helperMatricesDirty = false;
}

void GameObject::calculateInverseInertiaForCube(float side) {
    float I = 6.0f / (mass * side * side);

    inverseInertia = glm::mat3(
        glm::vec3(I, 0.0f, 0.0f),
        glm::vec3(0.0f, I, 0.0f),
        glm::vec3(0.0f, 0.0f, I)
    );
}

void GameObject::calculateInverseInertiaForCuboid(float sx, float sy, float sz) {
    float I_x = (1.0f / 12.0f) * mass * (sy * sy + sz * sz);
    float I_y = (1.0f / 12.0f) * mass * (sx * sx + sz * sz);
    float I_z = (1.0f / 12.0f) * mass * (sx * sx + sy * sy);

    inverseInertia = glm::mat3(
        glm::vec3(1.0f / I_x, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f / I_y, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f / I_z));
}

void GameObject::calculateInverseInertiaForSolidSphere() {
    Sphere& sphere = std::get<Sphere>(collider.shape);
    float I = (2.0f / 5.0f) * (mass * sphere.radius * sphere.radius);
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

void GameObject::updatePos(const float& dt) {
    if (selectedByEditor)
        return;

    if (allowGravity && !player)
        linearVelocity += g * dt;

    float aLin;
    float aAng;
    bool avgCollisions = collisionHistory.average() > 0;

    if (colliderType == ColliderType::SPHERE) {
        aLin = 0.1f; // konstant linjär retardation (m/s^2)
        aAng = 1.0f; // konstant angulär retardation (rad/s^2)

        float vMag = glm::length(linearVelocity);
        if (vMag > 0.0f && avgCollisions) {
            float newMag = vMag - aLin * dt;
            if (newMag < 0.0f) newMag = 0.0f;
            linearVelocity *= (newMag / vMag); // behåll riktningen
        }

        float wMag = glm::length(angularVelocity);
        if (wMag > 0.0f && avgCollisions) {
            float newMag = wMag - aAng * dt;
            if (newMag < 0.0f) newMag = 0.0f;
            angularVelocity *= (newMag / wMag);
        }
    }

    // anti stuck for objects with high collision counts
    avgCollisions = collisionHistory.average() > 3;
    if (avgCollisions) {
        linearVelocity = linearVelocity * std::pow(0.98f, dt);
        angularVelocity = angularVelocity * std::pow(0.98f, dt);
    }

    // fake constraints
    if (!canMoveLinearly) {
        linearVelocity = glm::vec3(0.0f);
        angularVelocity.x = 0.0f;
        angularVelocity.y = 0.0f;
    }

    // player controls
    if (player) {
        glm::vec3 v = linearVelocity;

        if (!onGround) {
            playerMoveImpulse.x *= 0.02f;
            playerMoveImpulse.z *= 0.02f;
        }

        v.x += playerMoveImpulse.x;
        v.z += playerMoveImpulse.z;

        if (!onGround) {
            v += g * 3.0f * dt;
        }
        else if (v.y < 0.0f) {
            v.y = 0.0f;
        }

        if (playerJumpImpulse != 0.0f) {
            v.y += playerJumpImpulse;
            playerJumpImpulse = 0.0f;
        }

        float maxSpeed = 15.0f;
        glm::vec2 vxz(v.x, v.z);
        float spd = glm::length(vxz);
        if (spd > maxSpeed) {
            vxz *= (maxSpeed / spd); // skala båda komponenterna lika
            v.x = vxz.x;
            v.z = vxz.y;
        }

        //if (onGround && glm::length2(playerMoveImpulse) < 0.01f) {
        //    v.x -= v.x * std::min(20.0f * dt, 3.0f);
        //    v.z -= v.z * std::min(20.0f * dt, 3.0f);
        //}

        if (onGround && glm::length2(playerMoveImpulse) < 0.01f) {
            v.x = 0.0f;
            v.z = 0.0f;
        }

        linearVelocity = v;

        if (onGround) {
            hasJumped = false;
        }

        onGround = false;
    }

    // update position and orientation
    lastPosition = position;
    position += linearVelocity * dt;
    updateOrientation(orientation, angularVelocity, dt);

    setRotatedFlag();
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

void GameObject::renderMesh(Shader& shader) {
    if (seeThrough) return;

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
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
}

void GameObject::setupMesh() {
    glGenVertexArrays(1, &VAO); glcount::incVAO();
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO); glcount::incVBO();
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO); glcount::incEBO();
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