#include "sat.h"

//=======================================================
//  Box-Sphere
//=======================================================
bool SAT::boxSphere(Collider& A, Collider& B, const ColliderTransformCache& transformCache, Result& out) {
    // Plocka alltid ut cuboid i A och sphere i B
    OOBB* box = std::get_if<OOBB>(&A.shape);
    Sphere* sph = std::get_if<Sphere>(&B.shape);

    // Matriser från GameObject (samma som du använder för rendering och OOBB/AABB)
    const glm::mat4& M = transformCache.modelMatrix;
    const glm::mat4& iM = transformCache.invModelMatrix;

    // Sphere center i värld
    glm::vec3 worldC = sph->centerWorld;
    // Transformera world → box-local
    glm::vec3 localC = glm::vec3(iM * glm::vec4(worldC, 1.0f));

    // Clamp i lokal AABB
    glm::vec3 clamped;
    clamped.x = glm::clamp(localC.x, -box->localHalfExtents.x, +box->localHalfExtents.x);
    clamped.y = glm::clamp(localC.y, -box->localHalfExtents.y, +box->localHalfExtents.y);
    clamped.z = glm::clamp(localC.z, -box->localHalfExtents.z, +box->localHalfExtents.z);

    // Transformera local → world
    glm::vec3 closestWorld = glm::vec3(M * glm::vec4(clamped, 1.0f));

    // Beräkna delta och dist2
    glm::vec3 delta = worldC - closestWorld;
    float dist2 = glm::dot(delta, delta);
    float r2 = sph->radiusWorld * sph->radiusWorld;

    // Test mot radie
    if (dist2 > r2) {
        return false;
    }

    // Fyll ut resultat, logga penetration & normal
    float dist = std::sqrt(dist2);
    out.depth = sph->radiusWorld - dist;
    out.normal = (dist > 1e-6f ? delta / dist : glm::vec3(0.0f, 1.0f, 0.0f));
    out.point = closestWorld;

    return true;
}