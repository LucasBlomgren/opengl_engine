#include "engine_state.h"

void EngineState::togglePause() { paused = !paused; }
void EngineState::toggleShowAABB() { showAabb = !showAabb; }
void EngineState::toggleShowContactPoints() { showContactPoints = !showContactPoints; }
void EngineState::toggleShowNormals() { showNormals = !showNormals; }
void EngineState::toggleShowCollisionNormal() { showCollisionNormal = !showCollisionNormal; }
void EngineState::setPressedKey(const std::string& key) { pressedKey = key; }
void EngineState::clearPressedKey() { pressedKey.clear(); }

bool EngineState::isPaused() const { return paused; }
bool EngineState::getShowAABB() const { return showAabb; }
bool EngineState::getShowContactPoints() const { return showContactPoints; }
bool EngineState::getShowNormals() const { return showNormals; }
bool EngineState::getShowCollisionNormal() const { return showCollisionNormal; }
std::string EngineState::GetPressedKey() const { return pressedKey; }