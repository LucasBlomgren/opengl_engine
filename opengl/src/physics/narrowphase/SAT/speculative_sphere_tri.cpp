#include "sat.h"

namespace physics::internal {

//=======================================================
//     Speculative Sphere-Triangle
//=======================================================
bool SAT::speculativeSphereTriangle(
    const Collider& sphereCollider,
    const RigidBody& sphereBody,
    const Tri& tri,
    float dt,
    SAT::Result& out)
{
    constexpr float eps = 1e-6f;

    if (dt <= 0.0f) {
        return false;
    }

    const Sphere& sphere = std::get<Sphere>(sphereCollider.shape);

    const glm::vec3 center = sphereCollider.worldPose.position;
    const glm::vec3 velocity = sphereBody.linearVelocity;
    const float radius = sphere.radiusWorld;

    if (radius <= 0.0f) {
        return false;
    }

    // Adjust names to your Tri layout if needed.
    const glm::vec3 triA = tri.vertices[0];
    const glm::vec3 triB = tri.vertices[1];
    const glm::vec3 triC = tri.vertices[2];

    glm::vec3 triN = glm::cross(triB - triA, triC - triA);
    float triNLen2 = glm::dot(triN, triN);

    if (triNLen2 < 1e-12f) {
        return false;
    }

    triN = glm::normalize(triN);

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

            float sqrtD = std::sqrt(disc);
            float inv2a = 1.0f / (2.0f * a);

            float t0 = (-b - sqrtD) * inv2a;
            float t1 = (-b + sqrtD) * inv2a;

            bool found = false;
            float best = maxT;

            if (t0 >= 0.0f && t0 <= maxT) {
                best = t0;
                found = true;
            }

            if (t1 >= 0.0f && t1 <= maxT && (!found || t1 < best)) {
                best = t1;
                found = true;
            }

            if (!found) {
                return false;
            }

            outT = best;
            return true;
        };

    auto pointInTriangle = [](
        const glm::vec3& p,
        const glm::vec3& a,
        const glm::vec3& b,
        const glm::vec3& c) -> bool
        {
            constexpr float eps = 1e-5f;

            glm::vec3 v0 = b - a;
            glm::vec3 v1 = c - a;
            glm::vec3 v2 = p - a;

            float d00 = glm::dot(v0, v0);
            float d01 = glm::dot(v0, v1);
            float d11 = glm::dot(v1, v1);
            float d20 = glm::dot(v2, v0);
            float d21 = glm::dot(v2, v1);

            float denom = d00 * d11 - d01 * d01;

            if (std::abs(denom) < 1e-8f) {
                return false;
            }

            float v = (d11 * d20 - d01 * d21) / denom;
            float w = (d00 * d21 - d01 * d20) / denom;
            float u = 1.0f - v - w;

            return u >= -eps && v >= -eps && w >= -eps;
        };

    auto closestPointOnSegment = [](
        const glm::vec3& p,
        const glm::vec3& a,
        const glm::vec3& b) -> glm::vec3
        {
            glm::vec3 ab = b - a;
            float abLen2 = glm::dot(ab, ab);

            if (abLen2 < 1e-8f) {
                return a;
            }

            float t = glm::dot(p - a, ab) / abLen2;
            t = glm::clamp(t, 0.0f, 1.0f);

            return a + t * ab;
        };

    bool hit = false;
    float bestT = dt;

    auto acceptHit = [&](
        float t,
        const glm::vec3& terrainToSphereNormal,
        float separation,
        SAT::AxisType featureType)
        {
            if (t < 0.0f || t > dt || t > bestT) {
                return;
            }

            glm::vec3 centerAtHit = center + velocity * t;

            out.hitType = SAT::HitType::Speculative;

            // Solver convention:
            // A = sphere, B = terrain
            // normal must point A -> B.
            out.normal = -terrainToSphereNormal;

            out.separation = glm::max(separation, 0.0f);
            out.depth = -out.separation;
            out.toi = t;

            // Point on sphere surface / terrain surface at TOI.
            out.point = centerAtHit - terrainToSphereNormal * radius;

            out.feature.type = featureType;
            out.tri_ptr = const_cast<Tri*>(&tri);

            bestT = t;
            hit = true;
        };

    // =====================================================
    // 1. Face hit: sphere surface touches triangle plane
    // =====================================================
    {
        float dist = glm::dot(center - triA, triN);
        float vn = glm::dot(velocity, triN);

        // Front-side hit.
        if (dist > radius && vn < -eps) {
            float t = (radius - dist) / vn;

            if (t >= 0.0f && t <= dt) {
                glm::vec3 centerAtHit = center + velocity * t;
                glm::vec3 pointOnPlane = centerAtHit - triN * radius;

                if (pointInTriangle(pointOnPlane, triA, triB, triC)) {
                    float separation = dist - radius;

                    acceptHit(
                        t,
                        triN, // terrain -> sphere
                        separation,
                        SAT::AxisType::TriFace
                    );
                }
            }
        }

        // Terrain MVP: no edge/vertex ghost contacts.
        constexpr bool terrainFaceOnly = true;
        if (terrainFaceOnly) {
            return hit;
        }

        //// Optional two-sided terrain.
        //// If your terrain triangles should be hit from the back too.
        //if (dist < -radius && vn > eps) {
        //    glm::vec3 n = -triN;
        //    float backDist = -dist;
        //    float backVn = glm::dot(velocity, n);

        //    float t = (radius - backDist) / backVn;

        //    if (t >= 0.0f && t <= dt) {
        //        glm::vec3 centerAtHit = center + velocity * t;
        //        glm::vec3 pointOnPlane = centerAtHit - n * radius;

        //        if (pointInTriangle(pointOnPlane, triA, triB, triC)) {
        //            float separation = backDist - radius;

        //            acceptHit(
        //                t,
        //                n, // terrain -> sphere
        //                separation,
        //                SAT::AxisType::TriFace
        //            );
        //        }
        //    }
        //}
    }

    // =====================================================
    // 2. Edge hit: swept sphere vs triangle edge capsule
    // =====================================================
    auto testEdge = [&](const glm::vec3& a, const glm::vec3& b) {
        glm::vec3 edge = b - a;
        float edgeLen = glm::length(edge);

        if (edgeLen < eps) {
            return;
        }

        glm::vec3 axis = edge / edgeLen;

        glm::vec3 m = center - a;
        glm::vec3 v = velocity;

        glm::vec3 mPerp = m - axis * glm::dot(m, axis);
        glm::vec3 vPerp = v - axis * glm::dot(v, axis);

        float qa = glm::dot(vPerp, vPerp);
        float qb = 2.0f * glm::dot(mPerp, vPerp);
        float qc = glm::dot(mPerp, mPerp) - radius * radius;

        // Already overlapping edge cylinder; let normal contact handle it.
        if (qc <= 0.0f) {
            return;
        }

        float t = 0.0f;

        if (!solveQuadraticEarliest(qa, qb, qc, dt, t)) {
            return;
        }

        if (t > bestT) {
            return;
        }

        glm::vec3 centerAtHit = center + velocity * t;

        float s = glm::dot(centerAtHit - a, axis);

        if (s < 0.0f || s > edgeLen) {
            return;
        }

        glm::vec3 closest = a + axis * s;
        glm::vec3 terrainToSphere = centerAtHit - closest;

        float len2 = glm::dot(terrainToSphere, terrainToSphere);

        if (len2 < 1e-10f) {
            return;
        }

        terrainToSphere *= 1.0f / std::sqrt(len2);

        glm::vec3 closestNow = closestPointOnSegment(center, a, b);
        float separation = glm::length(center - closestNow) - radius;

        acceptHit(
            t,
            terrainToSphere,
            separation,
            SAT::AxisType::EdgeEdge
        );
        };

    testEdge(triA, triB);
    testEdge(triB, triC);
    testEdge(triC, triA);

    // =====================================================
    // 3. Vertex hit: swept sphere vs triangle vertex
    // =====================================================
    auto testVertex = [&](const glm::vec3& p) {
        glm::vec3 m = center - p;
        glm::vec3 v = velocity;

        float qa = glm::dot(v, v);
        float qb = 2.0f * glm::dot(m, v);
        float qc = glm::dot(m, m) - radius * radius;

        // Already overlapping vertex sphere; let normal contact handle it.
        if (qc <= 0.0f) {
            return;
        }

        float t = 0.0f;

        if (!solveQuadraticEarliest(qa, qb, qc, dt, t)) {
            return;
        }

        if (t > bestT) {
            return;
        }

        glm::vec3 centerAtHit = center + velocity * t;
        glm::vec3 terrainToSphere = centerAtHit - p;

        float len2 = glm::dot(terrainToSphere, terrainToSphere);

        if (len2 < 1e-10f) {
            return;
        }

        terrainToSphere *= 1.0f / std::sqrt(len2);

        float separation = glm::length(center - p) - radius;

        acceptHit(
            t,
            terrainToSphere,
            separation,
            SAT::AxisType::SpherePoint
        );
        };

    testVertex(triA);
    testVertex(triB);
    testVertex(triC);

    return hit;
}

}
