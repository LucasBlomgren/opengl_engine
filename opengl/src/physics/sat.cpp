#include "SAT.h"

static constexpr int N = 3;

// Typ för alla collisons-funktioner
using CollisionFn = bool(*)(GameObject&, GameObject&, glm::vec3&, float&, int&);

// Dispatch-tabellen (måste initieras i en .cpp-fil)
static CollisionFn dispatchTable[N][N] = {
    { cuboidVsCuboid, nullptr,        nullptr    }, 
    { sphereVsCuboid, sphereVsSphere, nullptr    }, 
    { triVsCuboid,    triVsSphere,   triVsTri    }  
};

bool SATQuery(GameObject& A, GameObject& B, glm::vec3& normal, float& depth, int& normalOwner) {
    int i = int(A.collider.shape.index());
    int j = int(B.collider.shape.index());

    if (i < j)  std::swap(i, j); 

    auto fn = dispatchTable[i][j];
    return fn(A, B, normal, depth, normalOwner);
}

// skapa contact manifolds här inne och returnera manifold, sen kör PGS-solver i physics.cpp
bool cuboidVsCuboid(GameObject& a, GameObject& b, glm::vec3& n, float& d, int& o) {
    return intersectPolygons(a, b, n, d, o);
}
bool sphereVsCuboid(GameObject& a, GameObject& b, glm::vec3& n, float& d, int& o) {
    return false; 
}
bool sphereVsSphere(GameObject& a, GameObject& b, glm::vec3& n, float& d, int& o) {
    return false; 
}
bool triVsCuboid(GameObject& a, GameObject& b, glm::vec3& n, float& d, int& o) {
    return false; 
}
bool triVsSphere(GameObject& a, GameObject& b, glm::vec3& n, float& d, int& o) {
    return false; 
}
bool triVsTri(GameObject& a, GameObject& b, glm::vec3& n, float& d, int& o) {
    return false;
}

std::pair<float, float> projectVertices(const std::array<glm::vec3, 8>& vertices, const glm::vec3& axis) {
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

bool intersectPolygons(GameObject& objA, GameObject& objB, glm::vec3& normal, float& depth, int& collisionNormalOwner){
    OOBB& boxA = std::get<OOBB>(objA.collider.shape);
    OOBB& boxB = std::get<OOBB>(objB.collider.shape);

    const std::array<glm::vec3, 8>& verticesA = boxA.transformedVertices;
    const std::array<glm::vec3, 8>& verticesB = boxB.transformedVertices;
    const std::array<glm::vec3, 3>& normalsA = boxA.normals;
    const std::array<glm::vec3, 3>& normalsB = boxB.normals;

    const float epsilon = 1e-6f;

    for (const glm::vec3& faceNormal : normalsA) {
        auto [minA, maxA] = projectVertices(verticesA, faceNormal);
        auto [minB, maxB] = projectVertices(verticesB, faceNormal);

        if (minA >= maxB or minB >= maxA)
            return false;

        float axisDepth = std::min(maxB - minA, maxA - minB);

        if (axisDepth < depth) {
            depth = axisDepth;
            normal = faceNormal;
            collisionNormalOwner = 0;
        }
    }

    for (const glm::vec3& faceNormal : normalsB) {
        auto [minA, maxA] = projectVertices(verticesA, faceNormal);
        auto [minB, maxB] = projectVertices(verticesB, faceNormal);

        if (minA >= maxB or minB >= maxA)
            return false;

        float axisDepth = std::min(maxB - minA, maxA - minB);

        if (axisDepth < depth) {
            depth = axisDepth;
            normal = faceNormal;
            collisionNormalOwner = 1;
        }
    }

    for (const glm::vec3& faceNormalA : normalsA)
        for (const glm::vec3& faceNormalB : normalsB) {
            glm::vec3 axis = glm::cross(faceNormalA, faceNormalB);

            if (glm::length2(axis) < epsilon)
                continue; // Skip degenerate axis

            axis = glm::normalize(axis);

            auto [minA, maxA] = projectVertices(verticesA, axis);
            auto [minB, maxB] = projectVertices(verticesB, axis);

            if (minA >= maxB or minB >= maxA)
                return false;			

            float axisDepth = std::min(maxB - minA, maxA - minB);

            if (axisDepth < depth) {
                depth = axisDepth;
                normal = axis;
                collisionNormalOwner = 0; // godtyckligt för edge vs edge
            }
        }

    return true;
}