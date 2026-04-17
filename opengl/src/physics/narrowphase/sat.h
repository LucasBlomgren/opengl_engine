#pragma once

#include <span>
#include "tri.h"
#include "collider_pose.h"
#include "collider.h"

namespace SAT { 
    enum class AxisType { FaceA, FaceB, EdgeEdge };

    struct Result {
        float depth = std::numeric_limits<float>::infinity();
        glm::vec3 normal{ 0.0f };
        AxisType axisType = AxisType::FaceA;

        // #TODO: fixa så att faceIndex och edgeIndex används i SAT
        int faceIndex = -1;
        int edgeIndexA = -1;
        int edgeIndexB = -1;

        float separationA = -std::numeric_limits<float>::infinity();
        float separationB = -std::numeric_limits<float>::infinity();

        Tri* tri_ptr = nullptr;
        glm::vec3 point{}; // sphereCube or sphereTri
    };

    bool boxBox(Collider& A, Collider& B, Result& out);
    bool boxSphere(Collider& A, Collider& B, const ColliderPose& pose, Result& out);
    bool boxTri(Collider& A, Tri& tri, Result& out);
    bool sphereSphere(Collider& A, Collider& B, Result& out);
    bool sphereTri(Collider& A, Tri& tri, Result& out);

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
}