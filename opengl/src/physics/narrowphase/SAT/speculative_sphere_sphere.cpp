#include "sat.h"

//=======================================================
//  Speculative Sphere-Sphere
//=======================================================
bool SAT::speculativeSphereSphere(
    const Collider& colliderA,
    const Collider& colliderB,
    const RigidBody& bodyA,
    const RigidBody& bodyB,
    float dt,
    SAT::Result& out)
{
    const Sphere& sphereA = std::get<Sphere>(colliderA.shape);
    const Sphere& sphereB = std::get<Sphere>(colliderB.shape);

    glm::vec3 centerA = colliderA.pose.position;
    glm::vec3 centerB = colliderB.pose.position;

    float radiusA = sphereA.radiusWorld;
    float radiusB = sphereB.radiusWorld;
    float radiusSum = radiusA + radiusB;

    glm::vec3 p = centerB - centerA; // relative position A -> B now
    glm::vec3 v = bodyB.linearVelocity - bodyA.linearVelocity; // relative velocity

    float dist2 = glm::dot(p, p);
    float r2 = radiusSum * radiusSum;

    // Already overlapping. Let normal discrete sphereSphere handle it.
    if (dist2 <= r2) {
        return false;
    }

    float a = glm::dot(v, v);
    if (a < 1e-8f) {
        return false;
    }

    // If they are moving apart, no speculative contact.
    float pv = glm::dot(p, v);
    if (pv >= 0.0f) {
        return false;
    }

    // Solve:
    // |p + v*t|^2 = r^2
    //
    // dot(v,v)t^2 + 2dot(p,v)t + dot(p,p)-r^2 = 0
    float b = 2.0f * pv;
    float c = dist2 - r2;

    float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f) {
        return false;
    }

    float sqrtD = std::sqrt(discriminant);

    float t = (-b - sqrtD) / (2.0f * a); // first time of impact

    if (t < 0.0f || t > dt) {
        return false;
    }

    glm::vec3 pAtHit = p + v * t;

    float hitDist2 = glm::dot(pAtHit, pAtHit);
    if (hitDist2 < 1e-8f) {
        return false;
    }

    glm::vec3 normal = pAtHit / std::sqrt(hitDist2); // A -> B at impact

    float currentDistance = std::sqrt(dist2);
    float separation = currentDistance - radiusSum;

    out.normal = normal;

    // Important: this is NOT penetration.
    // Use negative depth or add a separate separation field.
    out.depth = -separation;

    out.hitType = SAT::HitType::Speculative;
    out.separation = separation;
    out.toi = t;

    // Approx contact point at impact.
    glm::vec3 centerAAtHit = centerA + bodyA.linearVelocity * t;
    glm::vec3 centerBAtHit = centerB + bodyB.linearVelocity * t;

    glm::vec3 pointA = centerAAtHit + normal * radiusA;
    glm::vec3 pointB = centerBAtHit - normal * radiusB;

    out.point = 0.5f * (pointA + pointB);

    return true;
}