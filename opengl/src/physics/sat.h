#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include <span>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/io.hpp>

#include "game_object.h"

namespace SAT { 
    enum class AxisType { FaceA, FaceB, EdgeEdge };

    struct Result {
        float depth = std::numeric_limits<float>::max();
        glm::vec3 normal;
        AxisType axisType;
        int faceIndex;
        int edgeIndexA;
        int edgeIndexB;

        Tri* tri_ptr = nullptr;
    };

    bool cuboidVsCuboid(Collider& A, Collider& B, Result& out);
    bool sphereVsCuboid(Collider& A, Collider& B, Result& out);
    bool sphereVsSphere(Collider& A, Collider& B, Result& out);
    bool triVsCuboid(Collider& A, Tri& tri, Result& out);
    bool triVsSphere(Collider& A, Collider& B, Result& out);
    bool triVsTri(Collider& A, Collider& B, Result& out);

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