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
    // Plocka alltid ut cuboid i A och sphere i B
    OOBB*   box  = std::get_if<OOBB>(&A.shape);
    Sphere* sph  = std::get_if<Sphere>(&B.shape);

    // Matriser från GameObject (samma som du använder för rendering och OOBB/AABB)
    const glm::mat4& M  = A.owner->modelMatrix;
    const glm::mat4& iM = A.owner->invModelMatrix;

    // Sphere center i värld
    glm::vec3 worldC = sph->wCenter;
    // Transformera world → box-local
    glm::vec3 localC = glm::vec3(iM * glm::vec4(worldC, 1.0f));

    // Clamp i lokal AABB
    glm::vec3 clamped;
    clamped.x = glm::clamp(localC.x, -box->halfExtents.x, +box->halfExtents.x);
    clamped.y = glm::clamp(localC.y, -box->halfExtents.y, +box->halfExtents.y);
    clamped.z = glm::clamp(localC.z, -box->halfExtents.z, +box->halfExtents.z);

    // Transformera local → world
    glm::vec3 closestWorld = glm::vec3(M * glm::vec4(clamped, 1.0f));

    // Beräkna delta och dist2
    glm::vec3 delta = worldC - closestWorld;
    float dist2     = glm::dot(delta, delta);
    float r2        = sph->radius * sph->radius;

    // Test mot radie
    if (dist2 > r2) {
        return false;
    }

    // Fyll ut resultat, logga penetration & normal
    float dist = std::sqrt(dist2);
    out.depth  = sph->radius - dist;
    out.normal = (dist > 1e-6f ? delta / dist : glm::vec3(0.0f,1.0f,0.0f));
    out.point = closestWorld;

    return true;
}


bool SAT::sphereVsSphere(Collider& A, Collider& B, Result& out) {
    Sphere& sphereA = std::get<Sphere>(A.shape);
    Sphere& sphereB = std::get<Sphere>(B.shape);

    glm::vec3 delta = sphereB.wCenter - sphereA.wCenter; 
    float dist2 = glm::dot(delta, delta); 
    float r2 = (sphereA.radius + sphereB.radius) * (sphereA.radius + sphereB.radius); 

    if (dist2 > r2) {
        return false; // No intersection
    }

    float dist = std::sqrt(dist2);
    out.depth = (sphereA.radius + sphereB.radius) - dist;
    out.normal = (dist > 1e-6f ? delta / dist : glm::vec3(0.0f, 1.0f, 0.0f));
    out.point = sphereB.wCenter + out.normal * sphereB.radius; // Contact point on sphereB

    return true;

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