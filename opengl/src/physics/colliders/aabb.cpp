#include "pch.h"
#include "aabb.h"

namespace physics::internal {

//=================================================
// Initialization
//=================================================
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
}

//=================================================
// Standard AABB functions
//=================================================
void AABB::update(
    const ColliderTransformCache& transformCache) 
{
    glm::mat3 model3x3 = glm::mat3(transformCache.modelMatrix);
    transform_withRotation(
        model3x3, 
        glm::vec3(transformCache.modelMatrix[3])
    );

    worldCenter = (worldMin + worldMax) * 0.5f;
    worldHalfExtents = (worldMax - worldMin) * 0.5f;
}

bool AABB::intersects(const AABB& b) const {
    constexpr float margin = 1e-4f;

    return 
        (worldMin.x <= b.worldMax.x + margin && worldMax.x + margin >= b.worldMin.x) &&
        (worldMin.y <= b.worldMax.y + margin && worldMax.y + margin >= b.worldMin.y) && 
        (worldMin.z <= b.worldMax.z + margin && worldMax.z + margin >= b.worldMin.z);
}

//=================================================
// Transformations
//=================================================
void AABB::transform_noRotation(
    const glm::mat4& M, 
    const glm::vec3& T, 
    const glm::vec3 S) 
{
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

//=================================================
// BVH functions
//=================================================
bool AABB::contains(const AABB& other) const {
    return
        (worldMin.x <= other.worldMin.x) && 
        (worldMin.y <= other.worldMin.y) && 
        (worldMin.z <= other.worldMin.z) &&
        (worldMax.x >= other.worldMax.x) && 
        (worldMax.y >= other.worldMax.y) && 
        (worldMax.z >= other.worldMax.z);
}
float AABB::getSurfaceArea() const {
    return 2.0f * (
        worldHalfExtents.x * 
        worldHalfExtents.y + 
        worldHalfExtents.y * 
        worldHalfExtents.z +
        worldHalfExtents.z * 
        worldHalfExtents.x
        );
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

    return 2.0f * (
        worldHalfExtents.x * 
        worldHalfExtents.y + 
        worldHalfExtents.y * 
        worldHalfExtents.z +
        worldHalfExtents.z * 
        worldHalfExtents.x
        );
}

}
