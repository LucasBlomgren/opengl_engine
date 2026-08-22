#include "pch.h"

#include "physics/public/engine.h"

namespace physics {

//====================================
// Scene-wide command submission
//====================================
void Engine::sleepAllObjects() {
    commandBuffer.recordSleepAllObjects();
}

void Engine::awakenAllObjects() {
    commandBuffer.recordAwakenAllObjects();
}

}
