#include "SAT.h"
#include <glm/gtx/string_cast.hpp>

void SAT::reverseNormal(glm::vec3& posA, glm::vec3& posB, glm::vec3& normal) { 
    glm::vec3 direction = posA - posB;
    if (glm::dot(direction, normal) > 0) { 
        normal = -normal; 
    }
}

void SAT::findBestTriangles(std::vector<SAT::Result>& results) { 
    if (results.size() <= 4)
        return; 

    std::vector<int> indices; 
    indices.reserve(4); 
    indices.push_back(0);

    // Sort results by depth
    std::sort(results.begin(), results.end(), [](const Result a, const Result b) { 
        return a.depth > b.depth;
        });  

    int loopRange = std::min(3, static_cast<int>(results.size()));
    for (int i = 0; i < loopRange; i++) {
        addFurthestTriangle(results, indices); 
    }

    std::vector<SAT::Result> temp;
    temp.reserve(indices.size());
    for (int idx : indices) {
        temp.push_back(results[idx]);
    } 

    results.swap(temp);
}

void SAT::addFurthestTriangle(std::vector<SAT::Result>& result, std::vector<int>& indices) {
    int bestIdx = -1;
    float bestDist = std::numeric_limits<float>::lowest();

    for (int i = 0; i < result.size(); i++)  
    { 
        // Skip if already added
        bool alreadyAdded = false;
        for (int j = 0; j < indices.size(); j++) { 
            if (i == indices[j]) {
                alreadyAdded = true;
                break;
            }
        }
        if (alreadyAdded) 
            continue; 

        float shortestDist = std::numeric_limits<float>::max();
        for (int j = 0; j < indices.size(); j++)
        { 
            glm::vec3 diff = result[i].tri_ptr->centroid - result[indices[j]].tri_ptr->centroid;
            float dist2 = glm::dot(diff, diff);

            if (dist2 < shortestDist) {
                shortestDist = dist2;
            } 
        }
        if (shortestDist > bestDist) {
            bestDist = shortestDist; 
            bestIdx = i; 
        }
    }

    indices.push_back(bestIdx);
}

bool SAT::cuboidVsCuboid(Collider& A, Collider& B, Result& out) {
    OOBB& boxA = std::get<OOBB>(A.shape);
    OOBB& boxB = std::get<OOBB>(B.shape);

    return intersectPolygons(boxA.wVertices, boxB.wVertices, boxA.wAxes, boxB.wAxes, out);
}
bool SAT::sphereVsCuboid(Collider& A, Collider& B, Result& out) {
    return false;
}
bool SAT::sphereVsSphere(Collider& A, Collider& B, Result& out) {
    return false;
}
bool SAT::triVsCuboid(Collider& A, Tri& tri, Result& out) {
    OOBB& boxA = std::get<OOBB>(A.shape);
    out.tri_ptr = &tri;

    return intersectPolygons(boxA.wVertices, tri.vertices, boxA.wAxes, tri.axes, out);
}
bool SAT::triVsSphere(Collider& A, Collider& B, Result& out) {
    return false;
}
bool SAT::triVsTri(Collider& A, Collider& B, Result& out) {
    return false;
}

std::pair<float, float> SAT::projectVertices(std::span<const glm::vec3> vertices, const glm::vec3& axis) {
    float min = std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::lowest();

    for (int i = 0; i < vertices.size(); i++)
    {
        const glm::vec3& v = vertices[i];
        float proj = glm::dot(v, axis);

        if (proj < min) { min = proj; }
        if (proj > max) { max = proj; }
    }

    return std::make_pair(min, max);
}

bool SAT::intersectPolygons(
    std::span<const glm::vec3> vertsA,
    std::span<const glm::vec3> vertsB,
    std::span<const glm::vec3> normalsA,
    std::span<const glm::vec3> normalsB,
    Result& satResult)
{
    // testa normaler från A
    for (const glm::vec3& A : normalsA) {
        auto [minA, maxA] = projectVertices(vertsA, A);
        auto [minB, maxB] = projectVertices(vertsB, A);

        if (minA >= maxB or minB >= maxA)
            return false;

        float axisDepth = std::min(maxB - minA, maxA - minB);

        if (axisDepth < satResult.depth) {
            satResult.depth = axisDepth;
            satResult.normal = A;
            satResult.axisType = AxisType::FaceA;
        }
    }

    // testa normaler från B
    for (const glm::vec3& B : normalsB) {
        auto [minA, maxA] = projectVertices(vertsA, B);
        auto [minB, maxB] = projectVertices(vertsB, B);

        if (minA >= maxB or minB >= maxA)
            return false;

        float axisDepth = std::min(maxB - minA, maxA - minB);

        if (axisDepth < satResult.depth) {
            satResult.depth = axisDepth;
            satResult.normal = B;
            satResult.axisType = AxisType::FaceB;
        }
    }

    // testa korsprodukter mellan normaler
    for (const glm::vec3& A : normalsA) {
        for (const glm::vec3& B : normalsB) {
            glm::vec3 axis = glm::cross(A, B);

            if (glm::length2(axis) < 1e-6f)
                continue; // Skip degenerate axis

            axis = glm::normalize(axis);

            auto [minA, maxA] = projectVertices(vertsA, axis);
            auto [minB, maxB] = projectVertices(vertsB, axis);

            if (minA >= maxB or minB >= maxA)
                return false;

            float axisDepth = std::min(maxB - minA, maxA - minB);

            if (axisDepth < satResult.depth) {
                satResult.depth = axisDepth;
                satResult.normal = axis;
                satResult.axisType = AxisType::EdgeEdge;
            }
        }
    }

    return true;
}