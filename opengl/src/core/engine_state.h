#pragma once

#include <string>

class EngineState 
{
public:

    void setAdvanceStep(bool);
    void setPaused(bool);
    void togglePause();
    void toggleShowFPS();
    void toggleShowAABB();
    void toggleShowColliders();
    void toggleShowContactPoints();
    void toggleShowCollisionNormal();
    void toggleShowNormals();
    void toggleShowBVH();

    void setPressedKey(const std::string& key);
    void clearPressedKey();

    bool isPaused() const;
    bool getAdvanceStep() const;
    bool getShowFPS() const;
    bool getShowNormals() const;
    bool getShowAABB() const;
    bool getShowColliders() const;
    bool getShowContactPoints() const;
    bool getShowCollisionNormal() const;
    bool getShowBVH() const;
    std::string GetPressedKey() const;

private:
    bool advanceStep = false;
    bool paused = false;
    bool showFPS = true;
    bool showAABB = false;
    bool showColliders = false;
    bool showContactPoints = false;
    bool showCollisionNormal = false;
    bool showNormals = false;
    bool showBVH = false;
    std::string pressedKey;
};