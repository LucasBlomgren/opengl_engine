#include "sat.h"

bool SAT::speculativeBoxSphere(
    const Collider& boxCollider,
    const Collider& sphereCollider,
    const RigidBody& boxBody,
    const RigidBody& sphereBody,
    float dt,
    SAT::Result& out)
{
    constexpr float eps = 1e-6f;

    if (dt <= 0.0f) {
        return false;
    }

    const OOBB& box = std::get<OOBB>(boxCollider.shape);
    const Sphere& sphere = std::get<Sphere>(sphereCollider.shape);

    const glm::vec3 boxCenter = boxCollider.pose.position;
    const glm::vec3 sphereCenter = sphereCollider.pose.position;

    // Adjust these names to your actual fields.
    const glm::vec3 half = box.localHalfExtents * box.scale; // half extents in local space
    const float radius = sphere.radiusWorld;

    glm::mat3 rot = glm::mat3_cast(boxCollider.pose.orientation);

    glm::vec3 boxAxes[3] = {
        glm::normalize(rot[0]),
        glm::normalize(rot[1]),
        glm::normalize(rot[2])
    };

    auto toLocalDir = [&](const glm::vec3& worldDir) -> glm::vec3 {
        return glm::vec3(
            glm::dot(worldDir, boxAxes[0]),
            glm::dot(worldDir, boxAxes[1]),
            glm::dot(worldDir, boxAxes[2])
        );
        };

    auto toWorldDir = [&](const glm::vec3& localDir) -> glm::vec3 {
        return
            boxAxes[0] * localDir.x +
            boxAxes[1] * localDir.y +
            boxAxes[2] * localDir.z;
        };

    // Treat box as static and sphere as moving relative to box.
    const glm::vec3 relCenterWorld = sphereCenter - boxCenter;
    const glm::vec3 relVelocityWorld =
        sphereBody.linearVelocity - boxBody.linearVelocity;

    const glm::vec3 p0 = toLocalDir(relCenterWorld);
    const glm::vec3 v = toLocalDir(relVelocityWorld);

    auto clampf = [](float x, float lo, float hi) -> float {
        return glm::min(glm::max(x, lo), hi);
        };

    auto closestPointOnAabb = [&](const glm::vec3& p) -> glm::vec3 {
        return glm::vec3(
            clampf(p.x, -half.x, half.x),
            clampf(p.y, -half.y, half.y),
            clampf(p.z, -half.z, half.z)
        );
        };

    glm::vec3 closest0 = closestPointOnAabb(p0);
    glm::vec3 delta0 = p0 - closest0;
    float dist2_0 = glm::dot(delta0, delta0);

    // Already touching/overlapping: normal discrete narrowphase should handle it.
    if (dist2_0 <= radius * radius) {
        return false;
    }

    float currentDistance = std::sqrt(dist2_0);
    float currentSeparation = currentDistance - radius;

    float bestT = dt + 1.0f;
    glm::vec3 bestNormalLocal{ 0.0f };

    SAT::AxisType bestType = SAT::AxisType::None;
    int bestFace = -1;
    int bestEdge = -1;
    int bestVertex = -1;

    auto solveQuadraticEarliest = [](
        float a,
        float b,
        float c,
        float maxT,
        float& outT) -> bool
        {
            constexpr float eps = 1e-8f;

            if (std::abs(a) < eps) {
                if (std::abs(b) < eps) {
                    return false;
                }

                float t = -c / b;

                if (t >= 0.0f && t <= maxT) {
                    outT = t;
                    return true;
                }

                return false;
            }

            float disc = b * b - 4.0f * a * c;

            if (disc < 0.0f) {
                return false;
            }

            float sqrtDisc = std::sqrt(disc);
            float invDenom = 0.5f / a;

            float t0 = (-b - sqrtDisc) * invDenom;
            float t1 = (-b + sqrtDisc) * invDenom;

            if (t0 > t1) {
                std::swap(t0, t1);
            }

            if (t0 >= 0.0f && t0 <= maxT) {
                outT = t0;
                return true;
            }

            if (t1 >= 0.0f && t1 <= maxT) {
                outT = t1;
                return true;
            }

            return false;
        };

    auto acceptHit = [&](
        float t,
        glm::vec3 normalLocal,
        SAT::AxisType type,
        int faceIndex,
        int edgeIndex,
        int vertexIndex)
        {
            if (t < 0.0f || t > dt) {
                return;
            }

            if (t >= bestT) {
                return;
            }

            float len2 = glm::dot(normalLocal, normalLocal);

            if (len2 < 1e-10f) {
                return;
            }

            bestT = t;
            bestNormalLocal = normalLocal * (1.0f / std::sqrt(len2));
            bestType = type;
            bestFace = faceIndex;
            bestEdge = edgeIndex;
            bestVertex = vertexIndex;
        };

    //=======================================================
    // Face hits
    //=======================================================
    for (int axis = 0; axis < 3; ++axis) {
        for (int side = -1; side <= 1; side += 2) {
            float plane = side * (half[axis] + radius);

            if (std::abs(v[axis]) < eps) {
                continue;
            }

            // Must approach the outside face.
            if (side > 0) {
                if (!(p0[axis] > plane && v[axis] < -eps)) {
                    continue;
                }
            }
            else {
                if (!(p0[axis] < plane && v[axis] > eps)) {
                    continue;
                }
            }

            float t = (plane - p0[axis]) / v[axis];

            if (t < 0.0f || t > dt) {
                continue;
            }

            glm::vec3 p = p0 + v * t;

            int a0 = (axis + 1) % 3;
            int a1 = (axis + 2) % 3;

            if (p[a0] < -half[a0] - eps || p[a0] > half[a0] + eps) {
                continue;
            }

            if (p[a1] < -half[a1] - eps || p[a1] > half[a1] + eps) {
                continue;
            }

            glm::vec3 nLocal{ 0.0f };
            nLocal[axis] = static_cast<float>(side);

            acceptHit(
                t,
                nLocal,
                SAT::AxisType::FaceA,
                axis,
                -1,
                -1
            );
        }
    }

    //=======================================================
    // Edge hits
    //=======================================================
    int edgeIndex = 0;

    for (int freeAxis = 0; freeAxis < 3; ++freeAxis) {
        int ax0 = (freeAxis + 1) % 3;
        int ax1 = (freeAxis + 2) % 3;

        for (int s0 = -1; s0 <= 1; s0 += 2) {
            for (int s1 = -1; s1 <= 1; s1 += 2) {
                glm::vec3 edgePoint{ 0.0f };
                edgePoint[ax0] = s0 * half[ax0];
                edgePoint[ax1] = s1 * half[ax1];

                glm::vec3 m = p0 - edgePoint;

                glm::vec2 m2(m[ax0], m[ax1]);
                glm::vec2 v2(v[ax0], v[ax1]);

                float a = glm::dot(v2, v2);
                float b = 2.0f * glm::dot(m2, v2);
                float c = glm::dot(m2, m2) - radius * radius;

                // Already inside this infinite edge-cylinder.
                // Let normal narrowphase/vertex tests handle edge cases.
                if (c <= 0.0f) {
                    ++edgeIndex;
                    continue;
                }

                float t;
                if (!solveQuadraticEarliest(a, b, c, dt, t)) {
                    ++edgeIndex;
                    continue;
                }

                if (t >= bestT) {
                    ++edgeIndex;
                    continue;
                }

                glm::vec3 p = p0 + v * t;

                if (p[freeAxis] < -half[freeAxis] - eps ||
                    p[freeAxis] >  half[freeAxis] + eps)
                {
                    ++edgeIndex;
                    continue;
                }

                glm::vec3 closestOnEdge = edgePoint;
                closestOnEdge[freeAxis] = p[freeAxis];

                glm::vec3 nLocal = p - closestOnEdge;

                acceptHit(
                    t,
                    nLocal,
                    SAT::AxisType::EdgeEdge,
                    -1,
                    edgeIndex,
                    -1
                );

                ++edgeIndex;
            }
        }
    }

    //=======================================================
    // Vertex hits
    //=======================================================
    int vertexIndex = 0;

    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                glm::vec3 vertex{
                    sx * half.x,
                    sy * half.y,
                    sz * half.z
                };

                glm::vec3 m = p0 - vertex;

                float a = glm::dot(v, v);
                float b = 2.0f * glm::dot(m, v);
                float c = glm::dot(m, m) - radius * radius;

                if (c <= 0.0f) {
                    ++vertexIndex;
                    continue;
                }

                float t;
                if (!solveQuadraticEarliest(a, b, c, dt, t)) {
                    ++vertexIndex;
                    continue;
                }

                if (t >= bestT) {
                    ++vertexIndex;
                    continue;
                }

                glm::vec3 p = p0 + v * t;
                glm::vec3 nLocal = p - vertex;

                acceptHit(
                    t,
                    nLocal,
                    SAT::AxisType::SpherePoint,
                    -1,
                    -1,
                    vertexIndex
                );

                ++vertexIndex;
            }
        }
    }

    if (bestT > dt) {
        return false;
    }

    glm::vec3 normalWorld = glm::normalize(toWorldDir(bestNormalLocal));

    glm::vec3 sphereCenterAtHit =
        sphereCenter + sphereBody.linearVelocity * bestT;

    glm::vec3 contactPoint =
        sphereCenterAtHit - normalWorld * radius;

    out.hitType = SAT::HitType::Speculative;
    out.normal = normalWorld; // box A -> sphere B
    out.separation = currentSeparation;
    out.depth = -currentSeparation;
    out.toi = bestT;
    out.point = contactPoint;

    out.feature.type = bestType;
    out.feature.faceIndex = bestFace;
    out.feature.edgeIndexA = bestEdge;
    out.feature.edgeIndexB = -1;
    out.feature.vertexIndex = bestVertex;

    return true;
}