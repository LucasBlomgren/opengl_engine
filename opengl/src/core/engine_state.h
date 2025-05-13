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
    void toggleShowOOBB();
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
    bool getShowOOBB() const;
    bool getShowContactPoints() const;
    bool getShowCollisionNormal() const;
    bool getShowBVH() const;
    std::string GetPressedKey() const;

private:
    bool advanceStep = false;
    bool paused = true;
    bool showFPS = true;
    bool showAABB = false;
    bool showOOBB = false;
    bool showContactPoints = false;
    bool showCollisionNormal = false;
    bool showNormals = false;
    bool showBVH = false;
    std::string pressedKey;
};