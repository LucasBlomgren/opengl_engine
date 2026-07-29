#include "physics/engine/physics_engine_impl.h"

namespace physics::internal {

//=========================================
// Pending command processing
//=========================================
void EngineImpl::processPendingCommands() {
    PhysicsCommandBuffer::Batch batch = commandBuffer.take();

    if (batch.empty()) {
        return;
    }

    processLifecycleCommands(batch);
    applyMutationCommands(batch.mutations);
}

}
