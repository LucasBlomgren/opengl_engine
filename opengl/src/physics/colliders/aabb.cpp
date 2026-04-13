#include "pch.h"
#include "aabb.h"

//--------------------------------------------
// Initialization
//--------------------------------------------
void AABB::init(const std::vector<glm::vec3>& vertices) {
    glm::vec3 mn(std::numeric_limits<float>::max());
    glm::vec3 mx(std::numeric_limits<float>::lowest());

    for (const auto& v : vertices) {
        mn = glm::min(mn, v);
        mx = glm::max(mx, v);
    }

    localMin = mn;
    localMax = mx;

    worldMin = localMin;
    worldMax = localMax;
    worldCenter = (worldMin + worldMax) * 0.5f;
    worldHalfExtents = (worldMax - worldMin) * 0.5f;
    setSurfaceArea();
}

//--------------------------------------------
// Standard AABB functions
//--------------------------------------------
void AABB::update(const ColliderPose& pose) {
    glm::mat3 model3x3 = glm::mat3(pose.modelMatrix);
    transform_withRotation(model3x3, pose.position);

    worldCenter = (worldMin + worldMax) * 0.5f;
    worldHalfExtents = (worldMax - worldMin) * 0.5f;
}

bool AABB::intersects(const AABB& b) const {
    return (worldMin.x <= b.worldMax.x and worldMax.x >= b.worldMin.x)
        and (worldMin.y <= b.worldMax.y and worldMax.y >= b.worldMin.y)
        and (worldMin.z <= b.worldMax.z and worldMax.z >= b.worldMin.z);
}

//--------------------------------------------
// Transformations
//--------------------------------------------
void AABB::transform_noRotation(const glm::mat4& M, const glm::vec3& T, const glm::vec3 S) {
    worldMin.x = localMin.x * S.x + T.x;
    worldMin.y = localMin.y * S.y + T.y;
    worldMin.z = localMin.z * S.z + T.z;

    worldMax.x = localMax.x * S.x + T.x;
    worldMax.y = localMax.y * S.y + T.y;
    worldMax.z = localMax.z * S.z + T.z;
}

// only for box meshes
void AABB::transform_withRotation(const glm::mat3& M, const glm::vec3& T) {
    float  a, b;
    float  Amin[3], Amax[3];
    float  Bmin[3], Bmax[3];

    Amin[0] = localMin.x; Amax[0] = localMax.x;
    Amin[1] = localMin.y; Amax[1] = localMax.y;
    Amin[2] = localMin.z; Amax[2] = localMax.z;

    Bmin[0] = Bmax[0] = T.x;
    Bmin[1] = Bmax[1] = T.y;
    Bmin[2] = Bmax[2] = T.z;

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            a = (M[j][i] * Amin[j]);
            b = (M[j][i] * Amax[j]);

            if (a < b) {
                Bmin[i] += a;
                Bmax[i] += b;
            }
            else {
                Bmin[i] += b;
                Bmax[i] += a;
            }
        }

    worldMin.x = Bmin[0]; worldMax.x = Bmax[0];
    worldMin.y = Bmin[1]; worldMax.y = Bmax[1];
    worldMin.z = Bmin[2]; worldMax.z = Bmax[2];
}

//--------------------------------------------
// BVH functions
//--------------------------------------------
bool AABB::contains(const AABB& other) const {
    return
        (worldMin.x <= other.worldMin.x) and (worldMin.y <= other.worldMin.y) and (worldMin.z <= other.worldMin.z) and
        (worldMax.x >= other.worldMax.x) and (worldMax.y >= other.worldMax.y) and (worldMax.z >= other.worldMax.z);
}
void AABB::setSurfaceArea() {
    surfaceArea = 2.0f * (worldHalfExtents.x * worldHalfExtents.y + worldHalfExtents.y * worldHalfExtents.z + worldHalfExtents.z * worldHalfExtents.x);
}
void AABB::growToInclude(const glm::vec3& p) {
    worldMin = glm::min(worldMin, p);
    worldMax = glm::max(worldMax, p);
}
void AABB::grow(glm::vec3 m) {
    worldMin -= m;
    worldMax += m;
}
float AABB::getMergedSurfaceArea(const AABB& A, const AABB& B) {
    glm::vec3 worldMin = glm::min(A.worldMin, B.worldMin);
    glm::vec3 worldMax = glm::max(A.worldMax, B.worldMax);
    glm::vec3 worldHalfExtents = (worldMax - worldMin) * 0.5f;

    return (2.0f * (worldHalfExtents.x * worldHalfExtents.y + worldHalfExtents.y * worldHalfExtents.z + worldHalfExtents.z * worldHalfExtents.x));
}

//--------------------------------------------
// Editor functions
//--------------------------------------------
glm::vec3 AABB::getCollisionNormal(const AABB& other) const {
    // Skillnad i centrum
    float dx = other.worldCenter.x - worldCenter.x;
    float dy = other.worldCenter.y - worldCenter.y;
    float dz = other.worldCenter.z - worldCenter.z;

    // Totala halv-extent per axel
    float combinedHalfX = worldHalfExtents.x + other.worldHalfExtents.x;
    float combinedHalfY = worldHalfExtents.y + other.worldHalfExtents.y;
    float combinedHalfZ = worldHalfExtents.z + other.worldHalfExtents.z;

    // Överlapp längs vardera axel
    float overlapX = combinedHalfX - std::fabs(dx);
    float overlapY = combinedHalfY - std::fabs(dy);
    float overlapZ = combinedHalfZ - std::fabs(dz);

    // Ingen kollision
    if (overlapX <= 0.0f || overlapY <= 0.0f || overlapZ <= 0.0f) {
        return glm::vec3(0.0f, 0.0f, 0.0f);
    }

    if (overlapX < overlapY && overlapX < overlapZ) {
        // om other ligger åt +X så vill vi separera längs -X
        float signX = (dx >= 0.0f) ? -1.0f : 1.0f;
        return glm::vec3(signX, 0.0f, 0.0f);
    }
    // motsvarande för Y och Z:
    else if (overlapY < overlapZ) {
        float signY = (dy >= 0.0f) ? -1.0f : 1.0f;
        return glm::vec3(0.0f, signY, 0.0f);
    }
    else {
        float signZ = (dz >= 0.0f) ? -1.0f : 1.0f;
        return glm::vec3(0.0f, 0.0f, signZ);
    }
}
glm::vec3 AABB::getOverlapDepth(const AABB& other) const {
    // Skillnad i centrum
    float dx = other.worldCenter.x - worldCenter.x;
    float dy = other.worldCenter.y - worldCenter.y;
    float dz = other.worldCenter.z - worldCenter.z;

    // Totala halv-extent per axel
    float combinedHalfX = worldHalfExtents.x + other.worldHalfExtents.x;
    float combinedHalfY = worldHalfExtents.y + other.worldHalfExtents.y;
    float combinedHalfZ = worldHalfExtents.z + other.worldHalfExtents.z;

    // Beräkna överlapp (kan vara negativt om ingen kollision)
    float overlapX = combinedHalfX - std::fabs(dx);
    float overlapY = combinedHalfY - std::fabs(dy);
    float overlapZ = combinedHalfZ - std::fabs(dz);

    // Kläm överlapp till noll
    float depthX = (overlapX > 0.0f) ? overlapX : 0.0f;
    float depthY = (overlapY > 0.0f) ? overlapY : 0.0f;
    float depthZ = (overlapZ > 0.0f) ? overlapZ : 0.0f;

    return glm::vec3(depthX, depthY, depthZ);
}
float AABB::getMinOverlapDepth(const AABB& other) const {
    glm::vec3 depth = getOverlapDepth(other);
    // Hitta minsta positiva djup
    float minDepth = depth.x;
    if (depth.y > 0.0f && (minDepth <= 0.0f || depth.y < minDepth)) {
        minDepth = depth.y;
    }
    if (depth.z > 0.0f && (minDepth <= 0.0f || depth.z < minDepth)) {
        minDepth = depth.z;
    }
    return (minDepth > 0.0f) ? minDepth : 0.0f;
}