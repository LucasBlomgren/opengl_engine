#include "sat.h"

//=======================================================
//  Sphere-Triangle
//=======================================================
bool SAT::sphereTri(Collider& A, Tri& tri, Result& out) {
    Sphere& sphere = std::get<Sphere>(A.shape);
    glm::vec3 P = sphere.centerWorld - tri.normal * glm::dot(sphere.centerWorld - tri.vertices[0], tri.normal);

    // test vs edge planes
    bool isInside = true;
    for (int i = 0; i < 3; i++) {
        glm::vec3 planeNormal = (glm::normalize(glm::cross(tri.edgeDirs[i], tri.normal)));
        glm::vec3 planePoint = tri.vertices[i];

        if (glm::dot(planeNormal, P - planePoint) > 0.0f) {
            isInside = false;
            break;
        }
    }

    float depth = sphere.radiusWorld - glm::length(sphere.centerWorld - P);

    if (isInside and depth > 0.0f) {
        out.point = P;
        out.normal = tri.normal;
        out.depth = depth;
        out.tri_ptr = &tri;
        return true;
    }

    // not inside planes, project sphere center onto triangle edges
    float bestDist2 = std::numeric_limits<float>::infinity();
    glm::vec3 bestQ;
    for (int i = 0; i < 3; i++) {
        glm::vec3& edge = tri.edgeDirs[i];

        float t = glm::dot(sphere.centerWorld - tri.vertices[i], edge) / glm::dot(edge, edge);
        t = glm::clamp(t, 0.0f, 1.0f);
        glm::vec3 Q = tri.vertices[i] + t * edge;

        float dist2 = glm::dot(sphere.centerWorld - Q, sphere.centerWorld - Q);

        if (dist2 < bestDist2) {
            bestDist2 = dist2;
            bestQ = Q;
        }
    }

    if (bestDist2 <= sphere.radiusWorld * sphere.radiusWorld) {
        out.point = bestQ;
        out.normal = glm::normalize(sphere.centerWorld - bestQ);
        out.depth = sphere.radiusWorld - std::sqrt(bestDist2);
        out.tri_ptr = &tri;
        return true;
    }

    return false;
}