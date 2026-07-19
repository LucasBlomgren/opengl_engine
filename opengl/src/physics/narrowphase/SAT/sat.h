#pragma once

#include "sat_types.h"
#include <span>
#include "tri.h"
#include "collider_pose.h"
#include "collider.h"
#include "rigidbody.h"

namespace SAT {
    //======================================================
    //      Utility functions for SAT results
    //=======================================================
    void reverseNormal(glm::vec3& posA, glm::vec3& posB, glm::vec3& normal);
    void findBestTriangles(std::vector<SAT::Result>& results);
    void addFurthestTriangle(std::vector<SAT::Result>& results, std::vector<int>& addedIndices);


    //=======================================================
    //     Normal contact tests
    //=======================================================
    bool boxBox(Collider& A, Collider& B, Result& out);
    bool boxSphere(Collider& A, Collider& B, const ColliderPose& pose, Result& out);
    bool boxTri(Collider& A, Tri& tri, Result& out);
    bool sphereSphere(Collider& A, Collider& B, Result& out);
    bool sphereTri(Collider& A, Tri& tri, Result& out);


    //=======================================================
    //     Speculative contact tests
    //=======================================================
    bool speculativeBoxBox(
        const Collider& colliderA,
        const Collider& colliderB,
        const RigidBody& bodyA,
        const RigidBody& bodyB,
        float dt,
        SAT::Result& out
    );

    bool speculativeBoxSphere(
        const Collider& boxCollider,
        const Collider& sphereCollider,
        const RigidBody& boxBody,
        const RigidBody& sphereBody,
        float dt,
        SAT::Result& out
    );

    bool speculativeBoxTriangle(
        const Collider& boxCollider,
        const RigidBody& boxBody,
        const Tri& tri,
        float dt,
        SAT::Result& out
    );

    bool speculativeSphereSphere(
        const Collider& colliderA,
        const Collider& colliderB,
        const RigidBody& bodyA,
        const RigidBody& bodyB,
        float dt,
        SAT::Result& out
    );

    bool speculativeSphereTriangle(
        const Collider& sphereCollider,
        const RigidBody& sphereBody,
        const Tri& tri,
        float dt,
        SAT::Result& out
    );


    //=======================================================
    //     General SAT test for convex polygons
    //======================================================
    std::pair<float, float> projectVertices(const std::span<const glm::vec3> vertices, const glm::vec3& axis);
    bool intersectPolygons(
        std::span<const glm::vec3> vertsA,
        std::span<const glm::vec3> vertsB,
        std::span<const glm::vec3> normalsA,
        std::span<const glm::vec3> normalsB,
        Result& satResult);
}