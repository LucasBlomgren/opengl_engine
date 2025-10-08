#include "pch.h"
#include "engine_state.h"

void EngineState::setAdvanceStep(bool arg) { advanceStep = arg; }
void EngineState::setPaused(bool arg) { paused = arg; }
void EngineState::togglePause() { paused = !paused; }
void EngineState::toggleShowFPS() { showFPS = !showFPS; }
void EngineState::toggleShowAABB() { showAABB = !showAABB; }
void EngineState::toggleShowColliders() { showColliders = !showColliders; }
void EngineState::toggleShowContactPoints() { showContactPoints = !showContactPoints; }
void EngineState::toggleShowNormals() { showNormals = !showNormals; }
void EngineState::toggleShowCollisionNormal() { showCollisionNormal = !showCollisionNormal; }
void EngineState::toggleShowBVH_awake() { showBVH_awake = !showBVH_awake; }
void EngineState::toggleShowBVH_asleep() { showBVH_asleep = !showBVH_asleep; }
void EngineState::toggleShowBVH_static() { showBVH_static = !showBVH_static; }
void EngineState::toggleShowBVH_terrain() { showBVH_terrain = !showBVH_terrain; }
void EngineState::setPressedKey(const std::string& key) { pressedKey = key; }
void EngineState::clearPressedKey() { pressedKey.clear(); }
void EngineState::setPlayerMode(bool arg) { playerMode = arg; }

bool EngineState::IsKeyDown(int key) const {
    if (key < 0 || key > GLFW_KEY_LAST) return false;
    return keyStates[key];
}
void EngineState::SetKeyState(int key, bool down) {
    if (key < 0 || key > GLFW_KEY_LAST) return;
    keyStates[key] = down;
}

bool EngineState::getAdvanceStep() const { return advanceStep; }
bool EngineState::isPaused() const { return paused; }
bool EngineState::getShowFPS() const { return showFPS; }
bool EngineState::getShowAABB() const { return showAABB; }
bool EngineState::getShowColliders() const { return showColliders; }
bool EngineState::getShowContactPoints() const { return showContactPoints; }
bool EngineState::getShowNormals() const { return showNormals; }
bool EngineState::getShowCollisionNormal() const { return showCollisionNormal; }
bool EngineState::getShowBVH_awake() const { return showBVH_awake; }
bool EngineState::getShowBVH_asleep() const { return showBVH_asleep; }
bool EngineState::getShowBVH_static() const { return showBVH_static; }
bool EngineState::getShowBVH_terrain() const { return showBVH_terrain; }
std::string EngineState::GetPressedKey() const { return pressedKey; }
bool EngineState::isPlayerMode() const { return playerMode; }