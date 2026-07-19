#include "sat.h"

bool SAT::speculativeBoxBox(
    const Collider& colliderA,
    const Collider& colliderB,
    const RigidBody& bodyA,
    const RigidBody& bodyB,
    float dt,
    SAT::Result& out)
{
    constexpr float eps = 1e-6f;

    if (dt <= 0.0f) {
        return false;
    }

    const OOBB& boxA = std::get<OOBB>(colliderA.shape);
    const OOBB& boxB = std::get<OOBB>(colliderB.shape);

    glm::vec3 centerA = colliderA.pose.position;
    glm::vec3 centerB = colliderB.pose.position;

    // Adjust these names to your actual OOBB fields.
    glm::vec3 halfExtentsA = boxA.localHalfExtents * boxA.scale;
    glm::vec3 halfExtentsB = boxB.localHalfExtents * boxB.scale;

    glm::mat3 rotA = glm::mat3_cast(colliderA.pose.orientation);
    glm::mat3 rotB = glm::mat3_cast(colliderB.pose.orientation);

    glm::vec3 axesA[3] = {
        glm::normalize(rotA[0]),
        glm::normalize(rotA[1]),
        glm::normalize(rotA[2])
    };

    glm::vec3 axesB[3] = {
        glm::normalize(rotB[0]),
        glm::normalize(rotB[1]),
        glm::normalize(rotB[2])
    };

    glm::vec3 velocityA = bodyA.linearVelocity;
    glm::vec3 velocityB = bodyB.linearVelocity;

    // B relative to A.
    glm::vec3 relV = velocityB - velocityA;

    float enterTime = 0.0f;
    float exitTime = dt;

    glm::vec3 bestAxis{ 0.0f };
    float bestSeparation = 0.0f;
    SAT::AxisType bestType = SAT::AxisType::None;
    int bestFaceA = -1;
    int bestFaceB = -1;
    int bestEdgeA = -1;
    int bestEdgeB = -1;

    bool separatedAtStart = false;

    auto projectRadius = [](
        const glm::vec3& axis,
        const glm::vec3 shapeAxes[3],
        const glm::vec3& halfExtents) -> float
        {
            return
                halfExtents.x * std::abs(glm::dot(shapeAxes[0], axis)) +
                halfExtents.y * std::abs(glm::dot(shapeAxes[1], axis)) +
                halfExtents.z * std::abs(glm::dot(shapeAxes[2], axis));
        };

    auto testAxis = [&](
        glm::vec3 axis,
        SAT::AxisType axisType,
        int faceA,
        int faceB,
        int edgeA,
        int edgeB) -> bool
        {
            float len2 = glm::dot(axis, axis);

            if (len2 < 1e-10f) {
                return true; // degenerate cross axis, skip
            }

            axis *= 1.0f / std::sqrt(len2);

            float ra = projectRadius(axis, axesA, halfExtentsA);
            float rb = projectRadius(axis, axesB, halfExtentsB);
            float r = ra + rb;

            float dist = glm::dot(centerB - centerA, axis);
            float speed = glm::dot(relV, axis);

            float absDist = std::abs(dist);
            float currentSeparation = absDist - r;

            if (currentSeparation > 0.0f) {
                separatedAtStart = true;
            }

            // Solve:
            // -r <= dist + speed * t <= r
            if (std::abs(speed) < eps) {
                // Not moving along this axis. If separated now, never overlaps.
                if (currentSeparation > 0.0f) {
                    return false;
                }

                return true;
            }

            float t0 = (-r - dist) / speed;
            float t1 = (r - dist) / speed;

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
                bestFaceA = faceA;
                bestFaceB = faceB;
                bestEdgeA = edgeA;
                bestEdgeB = edgeB;
            }

            exitTime = glm::min(exitTime, t1);

            if (enterTime > exitTime) {
                return false;
            }

            return true;
        };

    // A face axes
    for (int i = 0; i < 3; ++i) {
        if (!testAxis(
            axesA[i],
            SAT::AxisType::FaceA,
            i,
            -1,
            -1,
            -1))
        {
            return false;
        }
    }

    // B face axes
    for (int i = 0; i < 3; ++i) {
        if (!testAxis(
            axesB[i],
            SAT::AxisType::FaceB,
            -1,
            i,
            -1,
            -1))
        {
            return false;
        }
    }

    // Edge cross edge axes
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            glm::vec3 axis = glm::cross(axesA[i], axesB[j]);

            if (!testAxis(
                axis,
                SAT::AxisType::EdgeEdge,
                -1,
                -1,
                i,
                j))
            {
                return false;
            }
        }
    }

    // If they already overlap, normal narrowphase should handle it.
    if (!separatedAtStart) {
        return false;
    }

    if (enterTime < 0.0f || enterTime > dt) {
        return false;
    }

    if (glm::dot(bestAxis, bestAxis) < 1e-10f) {
        return false;
    }

    glm::vec3 centerAAtHit = centerA + velocityA * enterTime;
    glm::vec3 centerBAtHit = centerB + velocityB * enterTime;

    auto supportPoint = [](
        const glm::vec3& center,
        const glm::vec3 shapeAxes[3],
        const glm::vec3& halfExtents,
        const glm::vec3& dir) -> glm::vec3
        {
            glm::vec3 p = center;

            p += shapeAxes[0] * ((glm::dot(shapeAxes[0], dir) >= 0.0f) ? halfExtents.x : -halfExtents.x);
            p += shapeAxes[1] * ((glm::dot(shapeAxes[1], dir) >= 0.0f) ? halfExtents.y : -halfExtents.y);
            p += shapeAxes[2] * ((glm::dot(shapeAxes[2], dir) >= 0.0f) ? halfExtents.z : -halfExtents.z);

            return p;
        };

    glm::vec3 pointA = supportPoint(centerAAtHit, axesA, halfExtentsA, bestAxis);
    glm::vec3 pointB = supportPoint(centerBAtHit, axesB, halfExtentsB, -bestAxis);

    out.hitType = SAT::HitType::Speculative;
    out.normal = bestAxis; // A -> B
    out.separation = bestSeparation;
    out.depth = -bestSeparation;
    out.toi = enterTime;
    out.point = 0.5f * (pointA + pointB);

    out.feature.type = bestType;
    out.feature.faceIndex = -1;
    out.feature.edgeIndexA = bestEdgeA;
    out.feature.edgeIndexB = bestEdgeB;

    if (bestType == SAT::AxisType::FaceA) {
        out.feature.faceIndex = bestFaceA;
    }
    else if (bestType == SAT::AxisType::FaceB) {
        out.feature.faceIndex = bestFaceB;
    }

    return true;
}