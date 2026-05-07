#include "pch.h"
#include "engine_state.h"

void EngineState::toggleShowWireframes() { 
    showWireframes = !showWireframes; 

    if (showWireframes) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
}

void EngineState::togglePause() { paused = !paused; }
void EngineState::toggleShowFPS() { showFPS = !showFPS; }
void EngineState::toggleShowAABB() { showAABB = !showAABB; }
void EngineState::toggleShowColliders() { showColliders = !showColliders; }
void EngineState::toggleShowContactPoints() { showContactPoints = !showContactPoints; }
void EngineState::toggleShowObjectLocalNormals() { showObjectLocalNormals = !showObjectLocalNormals; }
void EngineState::toggleShowCollisionNormals() { showCollisionNormals = !showCollisionNormals; }
void EngineState::toggleShowBVH_awake() { showBVH_awake = !showBVH_awake; }
void EngineState::toggleShowBVH_asleep() { showBVH_asleep = !showBVH_asleep; }
void EngineState::toggleShowBVH_static() { showBVH_static = !showBVH_static; }
void EngineState::toggleShowBVH_terrain() { showBVH_terrain = !showBVH_terrain; }

void EngineState::setAdvanceStep(bool arg) { advanceStep = arg; }
void EngineState::setPaused(bool arg) { paused = arg; }
void EngineState::setPlayerMode(bool arg) { playerMode = arg; }

bool EngineState::getShowWireframes() const { return showWireframes; }
bool EngineState::getAdvanceStep() const { return advanceStep; }
bool EngineState::isPaused() const { return paused; }
bool EngineState::getShowFPS() const { return showFPS; }
bool EngineState::getShowAABB() const { return showAABB; }
bool EngineState::getShowColliders() const { return showColliders; }
bool EngineState::getShowContactPoints() const { return showContactPoints; }
bool EngineState::getShowObjectLocalNormals() const { return showObjectLocalNormals; }
bool EngineState::getShowCollisionNormals() const { return showCollisionNormals; }
bool EngineState::getShowBVH_awake() const { return showBVH_awake; }
bool EngineState::getShowBVH_asleep() const { return showBVH_asleep; }
bool EngineState::getShowBVH_static() const { return showBVH_static; }
bool EngineState::getShowBVH_terrain() const { return showBVH_terrain; }
bool EngineState::isPlayerMode() const { return playerMode; }