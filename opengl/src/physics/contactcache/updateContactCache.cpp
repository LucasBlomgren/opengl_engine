#include "pch.h"
#include "physics/physics_engine.h"

namespace physics {

//=================================================
//       Contact Cache
//=================================================
void Engine::updateContactCache() {
    constexpr int maxFramesWithoutCollision = 5;
    for (auto it = contactCache.begin(); it != contactCache.end(); ) {
        if (!it->second.wasUsedThisFrame) {
            it->second.framesSinceUsed++;

            // Ta bort manifold efter X antal frames utan kollisionsmatch
            if (it->second.framesSinceUsed > maxFramesWithoutCollision) {
                it = contactCache.erase(it);
                continue;
            }
        }
        else {
            it->second.framesSinceUsed = 0;
        }
        ++it;
    }
}

}
