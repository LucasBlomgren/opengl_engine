#include "pch.h"

#include "physics/raycast/raycast.h"
#include "physics/colliders/aabb.h"

namespace physics::internal {

namespace raycast {
    RaycastHit raycastTree(
        const Ray& ray,
        const BVHTree& bvh,
        const SlotMap<RigidBody, BodyHandle>& bodyMap,
        BodyHandle ignoredBody)
    {
        AABB rayAABB;
        rayAABB.worldMin = glm::min(ray.start, ray.end);
        rayAABB.worldMax = glm::max(ray.start, ray.end);

        BodyHandle bestBodyHandle;
        float bestT = std::numeric_limits<float>::max();

        std::vector<BodyHandle> candidates;
        bvh.singleQuery(rayAABB, candidates);

        for (BodyHandle handle : candidates) {
            if (handle == ignoredBody) {
                continue;
            }

            const RigidBody& body = bodyMap.get(handle);

            const glm::vec3 tMin =
                (body.aabb.worldMin - ray.start) / ray.direction;
            const glm::vec3 tMax =
                (body.aabb.worldMax - ray.start) / ray.direction;

            const glm::vec3 t1 = glm::min(tMin, tMax);
            const glm::vec3 t2 = glm::max(tMin, tMax);

            const float tNear = glm::compMax(t1);
            const float tFar = glm::compMin(t2);

            if (tNear <= tFar &&
                tFar >= 0.0f &&
                tNear < bestT)
            {
                bestT = tNear;
                bestBodyHandle = handle;
            }
        }

        if (!bestBodyHandle.isValid()) {
            return {};
        }

        const RigidBody& bestBody = bodyMap.get(bestBodyHandle);

        const glm::vec3 tMin =
            (bestBody.aabb.worldMin - ray.start) / ray.direction;
        const glm::vec3 tMax =
            (bestBody.aabb.worldMax - ray.start) / ray.direction;
        const glm::vec3 entry = glm::min(tMin, tMax);

        int hitAxis = 2;

        if (entry.x > entry.y && entry.x > entry.z) {
            hitAxis = 0;
        }
        else if (entry.y > entry.z) {
            hitAxis = 1;
        }

        glm::vec3 normal(0.0f);

        if (hitAxis == 0) {
            normal.x = ray.direction.x > 0.0f ? -1.0f : 1.0f;
        }
        else if (hitAxis == 1) {
            normal.y = ray.direction.y > 0.0f ? -1.0f : 1.0f;
        }
        else {
            normal.z = ray.direction.z > 0.0f ? -1.0f : 1.0f;
        }

        RaycastHit hit;
        hit.hit = true;
        hit.bodyHandle = bestBodyHandle;
        hit.point = ray.start + ray.direction * bestT;
        hit.normal = normal;
        hit.t = bestT;
        return hit;
    }
}

}
