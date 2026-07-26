#include "pch.h"

#include "physics/engine/physics_engine_impl.h"

//=========================================
// Pending command processing
//=========================================
void PhysicsEngine::Impl::processPendingCommands() {
    PhysicsCommandBuffer::Batch batch = commandBuffer.take();

    if (batch.empty()) {
        return;
    }

    processLifecycleCommands(batch);
    applyMutationCommands(batch.mutations);
}
