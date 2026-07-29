#include "sat.h"

namespace physics::internal {

bool SAT::speculativeBoxTriangle(
    const Collider& boxCollider,
    const RigidBody& boxBody,
    const Tri& tri,
    float dt,
    SAT::Result& out)
{
    constexpr float eps = 1e-6f;

    if (dt <= 0.0f) {
        return false;
    }

    const OOBB& box = std::get<OOBB>(boxCollider.shape);

    glm::vec3 boxCenter = boxCollider.worldPose.position;
    glm::vec3 boxVelocity = boxBody.linearVelocity;

    // Adjust to your actual OOBB field name.
    glm::vec3 half = box.localHalfExtents * box.scale;

    glm::mat3 rot = glm::mat3_cast(boxCollider.worldPose.orientation);

    glm::vec3 boxAxes[3] = {
        glm::normalize(rot[0]),
        glm::normalize(rot[1]),
        glm::normalize(rot[2])
    };

    // Adjust to your actual Tri field names.
    glm::vec3 triA = tri.vertices[0];
    glm::vec3 triB = tri.vertices[1];
    glm::vec3 triC = tri.vertices[2];

    glm::vec3 triEdges[3] = {
        triB - triA,
        triC - triB,
        triA - triC
    };

    glm::vec3 triNormal = glm::cross(triB - triA, triC - triA);
    float triNormalLen2 = glm::dot(triNormal, triNormal);

    if (triNormalLen2 < 1e-12f) {
        return false;
    }

    triNormal = glm::normalize(triNormal);

    float enterTime = 0.0f;
    float exitTime = dt;

    bool separatedAtStart = false;

    glm::vec3 bestAxis{ 0.0f };
    float bestSeparation = 0.0f;
    SAT::AxisType bestType = SAT::AxisType::None;
    int bestBoxAxis = -1;
    int bestTriEdge = -1;

    auto boxRadiusOnAxis = [&](
        const glm::vec3& axis) -> float
        {
            return
                half.x * std::abs(glm::dot(boxAxes[0], axis)) +
                half.y * std::abs(glm::dot(boxAxes[1], axis)) +
                half.z * std::abs(glm::dot(boxAxes[2], axis));
        };

    auto projectTriangle = [&](
        const glm::vec3& axis,
        float& outMin,
        float& outMax)
        {
            float p0 = glm::dot(triA, axis);
            float p1 = glm::dot(triB, axis);
            float p2 = glm::dot(triC, axis);

            outMin = std::min(p0, std::min(p1, p2));
            outMax = std::max(p0, std::max(p1, p2));
        };

    auto testAxis = [&](
        glm::vec3 axis,
        SAT::AxisType axisType,
        int boxAxisIndex,
        int triEdgeIndex) -> bool
        {
            float len2 = glm::dot(axis, axis);

            if (len2 < 1e-10f) {
                return true; // degenerate cross axis
            }

            axis *= 1.0f / std::sqrt(len2);

            float triMin;
            float triMax;
            projectTriangle(axis, triMin, triMax);

            float triCenter = 0.5f * (triMin + triMax);
            float triRadius = 0.5f * (triMax - triMin);

            float boxCenterProjection = glm::dot(boxCenter, axis);
            float boxRadius = boxRadiusOnAxis(axis);

            float totalRadius = boxRadius + triRadius;

            // B relative to A along this axis.
            // B = triangle interval center.
            // A = box interval center.
            float dist = triCenter - boxCenterProjection;

            // Triangle is static, so relative velocity B - A = -boxVelocity.
            float speed = glm::dot(-boxVelocity, axis);

            float currentSeparation = std::abs(dist) - totalRadius;

            if (currentSeparation > 0.0f) {
                separatedAtStart = true;
            }

            // Need:
            // -totalRadius <= dist + speed * t <= totalRadius
            if (std::abs(speed) < eps) {
                if (currentSeparation > 0.0f) {
                    return false;
                }

                return true;
            }

            float t0 = (-totalRadius - dist) / speed;
            float t1 = (totalRadius - dist) / speed;

            if (t0 > t1) {
                std::swap(t0, t1);
            }

            if (t0 > enterTime) {
                enterTime = t0;

                float distAtEnter = dist + speed * t0;

                // normal A -> B
                bestAxis = (distAtEnter >= 0.0f) ? axis : -axis;

                bestSeparation = glm::max(currentSeparation, 0.0f);
                bestType = axisType;
                bestBoxAxis = boxAxisIndex;
                bestTriEdge = triEdgeIndex;
            }

            exitTime = glm::min(exitTime, t1);

            if (enterTime > exitTime) {
                return false;
            }

            return true;
        };

    // 3 box face axes
    for (int i = 0; i < 3; ++i) {
        if (!testAxis(
            boxAxes[i],
            SAT::AxisType::FaceA,
            i,
            -1))
        {
            return false;
        }
    }

    // triangle face normal
    if (!testAxis(
        triNormal,
        SAT::AxisType::TriFace,
        -1,
        -1))
    {
        return false;
    }

    // 3 x 3 box-edge x triangle-edge axes
    for (int boxAxisIndex = 0; boxAxisIndex < 3; ++boxAxisIndex) {
        for (int triEdgeIndex = 0; triEdgeIndex < 3; ++triEdgeIndex) {
            glm::vec3 axis = glm::cross(
                boxAxes[boxAxisIndex],
                triEdges[triEdgeIndex]
            );

            if (!testAxis(
                axis,
                SAT::AxisType::EdgeEdge,
                boxAxisIndex,
                triEdgeIndex))
            {
                return false;
            }
        }
    }

    // Already overlapping: normal narrowphase should handle it.
    if (!separatedAtStart) {
        return false;
    }

    if (enterTime < 0.0f || enterTime > dt) {
        return false;
    }

    if (glm::dot(bestAxis, bestAxis) < 1e-10f) {
        return false;
    }

    //=======================================================
    // Terrain speculative MVP fix:
    // Do not emit edge/vertex contacts against triangle terrain.
    // Internal triangle edges can create ghost normals that push objects
    // sideways/down into the terrain.
    //=======================================================
    constexpr bool terrainFaceOnly = true;
    constexpr bool oneSidedTerrain = true;

    if (terrainFaceOnly) {
        if (bestType != SAT::AxisType::TriFace) {
            return false;
        }

        // For terrain, assume triNormal is the valid surface normal.
        // Since A = box and B = terrain, the solver normal should point
        // from box -> terrain, i.e. opposite the terrain normal when the
        // box is above/front-side of the triangle.
        if (oneSidedTerrain) {
            if (glm::dot(bestAxis, triNormal) > -0.5f) {
                return false; // backside hit
            }

            bestAxis = -triNormal;
        }
        else {
            bestAxis =
                (glm::dot(bestAxis, triNormal) < 0.0f)
                ? -triNormal
                : triNormal;
        }

        // Extra safety: only speculative hit if box is moving toward terrain.
        if (glm::dot(boxVelocity, bestAxis) <= eps) {
            return false;
        }

        bestType = SAT::AxisType::TriFace;
        bestBoxAxis = -1;
        bestTriEdge = -1;
    }

    glm::vec3 boxCenterAtHit = boxCenter + boxVelocity * enterTime;

    auto supportPointBox = [&](
        const glm::vec3& center,
        const glm::vec3& dir) -> glm::vec3
        {
            glm::vec3 p = center;

            p += boxAxes[0] * ((glm::dot(boxAxes[0], dir) >= 0.0f) ? half.x : -half.x);
            p += boxAxes[1] * ((glm::dot(boxAxes[1], dir) >= 0.0f) ? half.y : -half.y);
            p += boxAxes[2] * ((glm::dot(boxAxes[2], dir) >= 0.0f) ? half.z : -half.z);

            return p;
        };

    auto supportPointTriangle = [&](
        const glm::vec3& dir) -> glm::vec3
        {
            float p0 = glm::dot(triA, dir);
            float p1 = glm::dot(triB, dir);
            float p2 = glm::dot(triC, dir);

            if (p0 >= p1 && p0 >= p2) {
                return triA;
            }

            if (p1 >= p0 && p1 >= p2) {
                return triB;
            }

            return triC;
        };

    glm::vec3 pointA = supportPointBox(boxCenterAtHit, bestAxis);
    glm::vec3 pointB;

    if (bestType == SAT::AxisType::TriFace) {
        // Project box support point onto triangle plane.
        pointB = pointA - triNormal * glm::dot(pointA - triA, triNormal);
    }
    else {
        pointB = supportPointTriangle(-bestAxis);
    }

    out.hitType = SAT::HitType::Speculative;
    out.normal = bestAxis; // A -> B
    out.separation = bestSeparation;
    out.depth = -bestSeparation;
    out.toi = enterTime;
    out.point = 0.5f * (pointA + pointB);

    out.feature.type = bestType;
    out.feature.faceIndex = -1;
    out.feature.edgeIndexA = -1;
    out.feature.edgeIndexB = -1;
    out.feature.vertexIndex = -1;

    if (bestType == SAT::AxisType::FaceA) {
        out.feature.faceIndex = bestBoxAxis;
    }
    else if (bestType == SAT::AxisType::EdgeEdge) {
        out.feature.edgeIndexA = bestBoxAxis;
        out.feature.edgeIndexB = bestTriEdge;
    }

    out.tri_ptr = const_cast<Tri*>(&tri);

    return true;
}

}
