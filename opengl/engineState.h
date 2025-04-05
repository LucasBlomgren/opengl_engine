#pragma once

#include <string>

class EngineState 
{
public:
    void TogglePause();
    void ToggleShowAABB();
    void ToggleShowContactPoints();
    void ToggleShowCollisionNormal();
    void ToggleShowNormals();

    void SetPressedKey(const std::string& key);

    bool IsPaused() const;
    bool GetShowNormals() const;
    bool GetShowAABB() const;
    bool GetShowContactPoints() const;
    bool GetShowCollisionNormal() const;
    std::string GetPressedKey() const;

private:
    bool paused = false;
    bool showAabb = false;
    bool showContactPoints = false;
    bool showCollisionNormal = false;
    bool showNormals = false;
    std::string pressedKey;
};