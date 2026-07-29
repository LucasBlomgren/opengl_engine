#include "pch.h"

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

namespace physics {

physics::debug::Data Engine::getDebugData() const {
    return impl->getDebugData();
}

physics::debug::StepPhase Engine::getDebugPhase() const {
    return impl->getDebugPhase();
}

void Engine::updateBVHRenderData(
    const physics::debug::BvhType& type,
    bool update) {
    impl->updateBVHRenderData(type, update);
}

std::vector<physics::AABB>
Engine::getDebugSweeps() const {
    return impl->getDebugSweeps();
}

std::vector<physics::debug::SpeculativeContact>
Engine::getDebugSpeculativeContacts() const {
    return impl->getDebugSpeculativeContacts();
}

std::vector<physics::debug::Contact>
Engine::getDebugContacts() const {
    return impl->getDebugContacts();
}

const std::vector<BodyHandle>&
Engine::getAwakeList() const {
    return impl->getAwakeList();
}

physics::debug::Bvh Engine::getDebugBvh(
    physics::debug::BvhType type) const {
    return impl->getDebugBvh(type);
}

physics::debug::Bvh
Engine::getTerrainDebugBvh() const {
    return impl->getTerrainDebugBvh();
}

}
