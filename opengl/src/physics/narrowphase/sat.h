#pragma once

#include <span>
#include "tri.h"
#include "collider_pose.h"
#include "collider.h"
#include "rigidbody.h"

namespace SAT {

    enum class AxisType {
        None,
        FaceA,
        FaceB,
        EdgeEdge,
        TriFace,
        SpherePoint
    };

    enum class HitType {
        Overlap,        // regular penetration
        Speculative     // swept hit
    };

    struct FeatureId {
        AxisType type = AxisType::None;

        int faceIndex = -1;
        int edgeIndexA = -1;
        int edgeIndexB = -1;
        int vertexIndex = -1;
    };

    struct Result {
        HitType hitType = HitType::Overlap;

        // Always from A to B after reverseNormal/canonicalization.
        glm::vec3 normal{ 0.0f };

        // Positive only for real penetration.
        float depth = 0.0f;

        // Positive only for speculative/non-overlapping contacts.
        // Example: sphere-sphere gap before impact.
        float separation = 0.0f;

        // Time of impact within current dt.
        // Normal overlap contacts can keep this at 0.
        float toi = 0.0f;

        // Useful for sphere-box, sphere-tri, speculative hit point, etc.
        glm::vec3 point{ 0.0f };

        // Feature information for manifold generation.
        FeatureId feature{};

        // Terrain-specific. Could later become a more generic mesh feature pointer/id.
        Tri* tri_ptr = nullptr;

        // Optional debug / SAT internals.
        float separationA = -std::numeric_limits<float>::infinity();
        float separationB = -std::numeric_limits<float>::infinity();
    };

    bool boxBox(Collider& A, Collider& B, Result& out);
    bool boxSphere(Collider& A, Collider& B, const ColliderPose& pose, Result& out);

    bool boxTri(Collider& A, Tri& tri, Result& out);
    bool boxTriSpecialized(Collider& A, Tri& tri, Result& out);

    bool sphereSphere(Collider& A, Collider& B, Result& out);
    bool sphereTri(Collider& A, Tri& tri, Result& out);

    bool speculativeSphereSphere(
        const Collider& colliderA,
        const Collider& colliderB,
        const RigidBody& bodyA,
        const RigidBody& bodyB,
        float dt,
        SAT::Result& out);

    std::pair<float, float> projectVertices(const std::span<const glm::vec3> vertices, const glm::vec3& axis);
    bool intersectPolygons(
        std::span<const glm::vec3> vertsA,
        std::span<const glm::vec3> vertsB,
        std::span<const glm::vec3> normalsA,
        std::span<const glm::vec3> normalsB,
        Result& satResult);

    void reverseNormal(glm::vec3& posA, glm::vec3& posB, glm::vec3& normal);
    void findBestTriangles(std::vector<SAT::Result>& results);
    void addFurthestTriangle(std::vector<SAT::Result>& results, std::vector<int>& addedIndices);

    bool overlapIntervals(float minA, float maxA, float minB, float maxB, float& outOverlap);

    void projectTriOntoAxisLocal(
        const glm::vec3& p0,
        const glm::vec3& p1,
        const glm::vec3& p2,
        const glm::vec3& axisLocal,
        float& outMin,
        float& outMax);
}