#include "sat.h"

namespace physics::internal {

//=======================================================
//  Sphere-Sphere
//=======================================================
bool SAT::sphereSphere(Collider& A, Collider& B, Result& out) {
    Sphere& sphereA = std::get<Sphere>(A.shape);
    Sphere& sphereB = std::get<Sphere>(B.shape);

    glm::vec3 delta = sphereB.centerWorld - sphereA.centerWorld;
    float dist2 = glm::dot(delta, delta);
    float r2 = (sphereA.radiusWorld + sphereB.radiusWorld) * (sphereA.radiusWorld + sphereB.radiusWorld);

    if (dist2 > r2) {
        return false; // No intersection
    }

    float dist = std::sqrt(dist2);
    out.depth = (sphereA.radiusWorld + sphereB.radiusWorld) - dist;
    out.normal = (dist > 1e-6f ? delta / dist : glm::vec3(0.0f, 1.0f, 0.0f));
    out.point = sphereB.centerWorld - out.normal * sphereB.radiusWorld; // Contact point on sphereB

    return true;
}

}
