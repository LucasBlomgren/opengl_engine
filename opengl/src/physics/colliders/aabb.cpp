#include "pch.h"
#include "aabb.h"

void AABB::init(const std::vector<glm::vec3>& vertices) {
    computeFromVertices(vertices);
    setLocalFaces();
}

void AABB::update(const Transform& t) {
    glm::mat3 model3x3 = glm::mat3(t.modelMatrix);
    transform_withRotation(model3x3, t.position);

    centroid = (wMin + wMax) * 0.5f;
    halfExtents = (wMax - wMin) * 0.5f;
}

bool AABB::intersects(const AABB& b) const {
    return (wMin.x <= b.wMax.x and wMax.x >= b.wMin.x)
        and (wMin.y <= b.wMax.y and wMax.y >= b.wMin.y)
        and (wMin.z <= b.wMax.z and wMax.z >= b.wMin.z);
}

// ----- bvh funktioner -----
bool AABB::contains(const AABB& other) const {
    return
        (wMin.x <= other.wMin.x) and (wMin.y <= other.wMin.y) and (wMin.z <= other.wMin.z) and
        (wMax.x >= other.wMax.x) and (wMax.y >= other.wMax.y) and (wMax.z >= other.wMax.z);
}
void AABB::setSurfaceArea() {
    surfaceArea = 2.0f * (halfExtents.x * halfExtents.y + halfExtents.y * halfExtents.z + halfExtents.z * halfExtents.x);
}
void AABB::growToInclude(const glm::vec3& p) {
    wMin = glm::min(wMin, p);
    wMax = glm::max(wMax, p);
}
void AABB::grow(glm::vec3 m) {
    wMin -= m;
    wMax += m;
}
float AABB::getMergedSurfaceArea(const AABB& A, const AABB& B) {
    glm::vec3 wMin = glm::min(A.wMin, B.wMin);
    glm::vec3 wMax = glm::max(A.wMax, B.wMax);
    glm::vec3 halfExtents = (wMax - wMin) * 0.5f;

    return (2.0f * (halfExtents.x * halfExtents.y + halfExtents.y * halfExtents.z + halfExtents.z * halfExtents.x));
}

// ----- editor funktioner -----
glm::vec3 AABB::getCollisionNormal(const AABB& other) const {
    // Skillnad i centrum
    float dx = other.centroid.x - centroid.x;
    float dy = other.centroid.y - centroid.y;
    float dz = other.centroid.z - centroid.z;

    // Totala halv-extent per axel
    float combinedHalfX = halfExtents.x + other.halfExtents.x;
    float combinedHalfY = halfExtents.y + other.halfExtents.y;
    float combinedHalfZ = halfExtents.z + other.halfExtents.z;

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
    float dx = other.centroid.x - centroid.x;
    float dy = other.centroid.y - centroid.y;
    float dz = other.centroid.z - centroid.z;

    // Totala halv-extent per axel
    float combinedHalfX = halfExtents.x + other.halfExtents.x;
    float combinedHalfY = halfExtents.y + other.halfExtents.y;
    float combinedHalfZ = halfExtents.z + other.halfExtents.z;

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

// ----- transformations -----
void AABB::transform_noRotation(const glm::mat4& M, const glm::vec3& T, const glm::vec3 S) {
    wMin.x = lMin.x * S.x + T.x;
    wMin.y = lMin.y * S.y + T.y;
    wMin.z = lMin.z * S.z + T.z;

    wMax.x = lMax.x * S.x + T.x;
    wMax.y = lMax.y * S.y + T.y;
    wMax.z = lMax.z * S.z + T.z;
}

// only for boxe meshes
void AABB::transform_withRotation(const glm::mat3& M, const glm::vec3& T) {
    float  a, b;
    float  Amin[3], Amax[3];
    float  Bmin[3], Bmax[3];

    Amin[0] = lMin.x; Amax[0] = lMax.x;
    Amin[1] = lMin.y; Amax[1] = lMax.y;
    Amin[2] = lMin.z; Amax[2] = lMax.z;

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

    wMin.x = Bmin[0]; wMax.x = Bmax[0];
    wMin.y = Bmin[1]; wMax.y = Bmax[1];
    wMin.z = Bmin[2]; wMax.z = Bmax[2];
}

// ----- init -----
void AABB::computeFromVertices(const std::vector<glm::vec3>& vertices) {
    glm::vec3 mn(std::numeric_limits<float>::max());
    glm::vec3 mx(std::numeric_limits<float>::lowest());

    for (const auto& v : vertices) {
        mn = glm::min(mn, v);
        mx = glm::max(mx, v);
    }

    lMin = mn;
    lMax = mx;
}
void AABB::setLocalFaces() {
    lFaces.minX = {
        glm::vec3(lMin.x, lMin.y, lMin.z),
        glm::vec3(lMin.x, lMin.y, lMax.z),
        glm::vec3(lMin.x, lMax.y, lMax.z),
        glm::vec3(lMin.x, lMax.y, lMin.z)
    };

    lFaces.maxX = {
        glm::vec3(lMax.x, lMin.y, lMax.z),
        glm::vec3(lMax.x, lMin.y, lMin.z),
        glm::vec3(lMax.x, lMax.y, lMin.z),
        glm::vec3(lMax.x, lMax.y, lMax.z)
    };

    lFaces.minY = {
        glm::vec3(lMax.x, lMin.y, lMin.z),
        glm::vec3(lMax.x, lMin.y, lMax.z),
        glm::vec3(lMin.x, lMin.y, lMax.z),
        glm::vec3(lMin.x, lMin.y, lMin.z)
    };

    lFaces.maxY = {
        glm::vec3(lMin.x, lMax.y, lMin.z),
        glm::vec3(lMin.x, lMax.y, lMax.z),
        glm::vec3(lMax.x, lMax.y, lMax.z),
        glm::vec3(lMax.x, lMax.y, lMin.z)
    };

    lFaces.minZ = {
        glm::vec3(lMin.x, lMin.y, lMin.z),
        glm::vec3(lMin.x, lMax.y, lMin.z),
        glm::vec3(lMax.x, lMax.y, lMin.z),
        glm::vec3(lMax.x, lMin.y, lMin.z)
    };

    lFaces.maxZ = {
        glm::vec3(lMin.x, lMin.y, lMax.z),
        glm::vec3(lMax.x, lMin.y, lMax.z),
        glm::vec3(lMax.x, lMax.y, lMax.z),
        glm::vec3(lMin.x, lMax.y, lMax.z)
    };
}