#pragma once

#include <string>

class EngineState 
{
public:
    void togglePause();
    void toggleShowAABB();
    void toggleShowContactPoints();
    void toggleShowCollisionNormal();
    void toggleShowNormals();

    void setPressedKey(const std::string& key);
    void clearPressedKey();

    bool isPaused() const;
    bool getShowNormals() const;
    bool getShowAABB() const;
    bool getShowContactPoints() const;
    bool getShowCollisionNormal() const;
    std::string GetPressedKey() const;

private:
    bool paused = false;
    bool showAabb = false;
    bool showContactPoints = false;
    bool showCollisionNormal = false;
    bool showNormals = false;
    std::string pressedKey;
};