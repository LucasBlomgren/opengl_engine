#include "physics/physics_engine.h"

namespace physics {

using namespace internal;

//=========================================
// Pending command processing
//=========================================
void Engine::processPendingCommands() {
    CommandBuffer::Batch batch = commandBuffer.take();

    if (batch.empty()) {
        return;
    }

    processLifecycleCommands(batch);
    applyMutationCommands(batch.mutations);
}

}
