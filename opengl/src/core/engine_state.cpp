#include "engine_state.h"

void EngineState::togglePause() { paused = !paused; }
void EngineState::toggleShowAABB() { showAABB = !showAABB; }
void EngineState::toggleShowOOBB() { showOOBB = !showOOBB; }
void EngineState::toggleShowContactPoints() { showContactPoints = !showContactPoints; }
void EngineState::toggleShowNormals() { showNormals = !showNormals; }
void EngineState::toggleShowCollisionNormal() { showCollisionNormal = !showCollisionNormal; }
void EngineState::setPressedKey(const std::string& key) { pressedKey = key; }
void EngineState::clearPressedKey() { pressedKey.clear(); }

bool EngineState::isPaused() const { return paused; }
bool EngineState::getShowAABB() const { return showAABB; }
bool EngineState::getShowOOBB() const { return showOOBB; }
bool EngineState::getShowContactPoints() const { return showContactPoints; }
bool EngineState::getShowNormals() const { return showNormals; }
bool EngineState::getShowCollisionNormal() const { return showCollisionNormal; }
std::string EngineState::GetPressedKey() const { return pressedKey; }