#include "engine_state.h"

void EngineState::TogglePause() { paused = !paused; }
void EngineState::ToggleShowAABB() { showAabb = !showAabb; }
void EngineState::ToggleShowContactPoints() { showContactPoints = !showContactPoints; }
void EngineState::ToggleShowNormals() { showNormals = !showNormals; }
void EngineState::ToggleShowCollisionNormal() { showCollisionNormal = !showCollisionNormal; }
void EngineState::SetPressedKey(const std::string& key) { pressedKey = key; }
void EngineState::ClearPressedKey() { pressedKey.clear(); }

bool EngineState::IsPaused() const { return paused; }
bool EngineState::GetShowAABB() const { return showAabb; }
bool EngineState::GetShowContactPoints() const { return showContactPoints; }
bool EngineState::GetShowNormals() const { return showNormals; }
bool EngineState::GetShowCollisionNormal() const { return showCollisionNormal; }
std::string EngineState::GetPressedKey() const { return pressedKey; }