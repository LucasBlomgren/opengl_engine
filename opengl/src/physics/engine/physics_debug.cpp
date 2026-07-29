#include "pch.h"

#include "physics/engine/physics_engine_impl.h"

namespace physics::internal {

namespace {
    physics::AABB toPublicBounds(const AABB& bounds) {
        physics::AABB result;
        result.worldMin = bounds.worldMin;
        result.worldMax = bounds.worldMax;
        result.worldCenter = bounds.worldCenter;
        result.worldHalfExtents = bounds.worldHalfExtents;
        return result;
    }

    glm::vec3 getColliderCenter(const Collider* collider) {
        if (!collider) {
            return glm::vec3(0.0f);
        }

        if (collider->type == ColliderType::CUBOID) {
            return std::get<OOBB>(collider->shape).worldCenter;
        }

        return std::get<Sphere>(collider->shape).centerWorld;
    }

    physics::debug::Bvh copyBvh(const BVHTree& tree) {
        physics::debug::Bvh result;
        result.nodes.reserve(tree.nodes.size());

        for (const BVHTree::Node& node : tree.nodes) {
            if (!node.alive) {
                continue;
            }

            result.nodes.push_back({
                node.isLeaf,
                toPublicBounds(node.isLeaf ? node.tightBox : node.fatBox)
            });
        }

        return result;
    }
}

physics::debug::Data EngineImpl::getDebugData() const {
    physics::debug::Data debugData;
    debugData.awake = broadphaseManager.getAwakeList().size();
    debugData.asleep = broadphaseManager.getAsleepList().size();
    debugData.staticBodies = broadphaseManager.getStaticList().size();
    debugData.colliders = physicsWorld.getCollidersMap().dense().size();
    debugData.terrainTris = terrainTriangles.size();
    debugData.contacts = contactsGeneratedThisFrame;
    return debugData;
}

physics::debug::StepPhase
EngineImpl::getDebugPhase() const {
    return debugPhase;
}

void EngineImpl::updateBVHRenderData(
    const physics::debug::BvhType& type,
    bool update) {
    broadphaseManager.updateBVHRenderData(type, update);
}

std::vector<physics::AABB>
EngineImpl::getDebugSweeps() const {
    std::vector<physics::AABB> result;
    result.reserve(debugSweeps.size());

    for (const AABB& bounds : debugSweeps) {
        result.push_back(toPublicBounds(bounds));
    }

    return result;
}

std::vector<physics::debug::SpeculativeContact>
EngineImpl::getDebugSpeculativeContacts() const {
    std::vector<physics::debug::SpeculativeContact> result;
    result.reserve(debugSpeculativeContacts.size());

    for (const DebugSpeculativeContact& contact :
        debugSpeculativeContacts) {
        result.push_back({
            contact.bodyA,
            contact.bodyB,
            contact.worldPos
        });
    }

    return result;
}

std::vector<physics::debug::Contact>
EngineImpl::getDebugContacts() const {
    std::vector<physics::debug::Contact> result;
    result.reserve(contactCache.size());

    for (const auto& [key, contact] : contactCache) {
        (void)key;

        if (!contact.wasUsedThisFrame) {
            continue;
        }

        physics::debug::Contact debugContact;
        debugContact.normal = contact.normal;
        const size_t sourcePointCount =
            (std::min)(
                static_cast<size_t>(contact.numPoints),
                debugContact.points.size()
            );

        for (size_t index = 0;
            index < sourcePointCount;
            ++index) {
            const ContactPoint& point = contact.points[index];

            if (!point.wasUsedThisFrame) {
                continue;
            }

            debugContact.points[debugContact.pointCount] = {
                point.worldPos,
                point.wasWarmStarted
            };
            debugContact.representativePoint += point.worldPos;
            ++debugContact.pointCount;
        }

        if (debugContact.pointCount > 0) {
            debugContact.representativePoint /=
                static_cast<float>(debugContact.pointCount);
        }
        else if (contact.runtimeData.colliderA) {
            debugContact.representativePoint =
                getColliderCenter(contact.runtimeData.colliderA);
        }
        else {
            debugContact.representativePoint =
                getColliderCenter(contact.runtimeData.colliderB);
        }

        result.push_back(debugContact);
    }

    return result;
}

const std::vector<BodyHandle>&
EngineImpl::getAwakeList() const {
    return broadphaseManager.getAwakeList();
}

physics::debug::Bvh EngineImpl::getDebugBvh(
    physics::debug::BvhType type) const
{
    switch (type) {
    case physics::debug::BvhType::Awake:
        return copyBvh(broadphaseManager.getAwakeBVH());
    case physics::debug::BvhType::Asleep:
        return copyBvh(broadphaseManager.getAsleepBVH());
    case physics::debug::BvhType::Static:
        return copyBvh(broadphaseManager.getStaticBVH());
    }

    return {};
}

physics::debug::Bvh
EngineImpl::getTerrainDebugBvh() const {
    physics::debug::Bvh result;
    const TerrainBVH& tree = broadphaseManager.getTerrainBVH();
    result.nodes.reserve(tree.nodes.size());

    for (const TerrainBVH::Node& node : tree.nodes) {
        if (!node.alive || !node.isLeaf) {
            continue;
        }

        result.nodes.push_back({
            true,
            toPublicBounds(node.fatBox)
        });
    }

    return result;
}

}
