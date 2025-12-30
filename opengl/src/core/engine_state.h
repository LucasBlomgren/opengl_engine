#pragma once

#include <string>
#include <array>
#include <GLFW/glfw3.h>

class EngineState {
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
    void toggleShowBVH_awake();
    void toggleShowBVH_asleep();
    void toggleShowBVH_static();
    void toggleShowBVH_terrain();

    bool isPaused() const;
    bool getAdvanceStep() const;
    bool getShowFPS() const;
    bool getShowNormals() const;
    bool getShowAABB() const;
    bool getShowColliders() const;
    bool getShowContactPoints() const;
    bool getShowCollisionNormal() const;
    bool getShowBVH_awake() const;
    bool getShowBVH_asleep() const;
    bool getShowBVH_static() const;
    bool getShowBVH_terrain() const;
    void setPlayerMode(bool);
    bool isPlayerMode() const;

private:
    bool playerMode = false;
    bool advanceStep = false;
    bool paused = false;
    bool showFPS = false;
    bool showAABB = false;
    bool showColliders = false;
    bool showContactPoints = false;
    bool showCollisionNormal = false;
    bool showNormals = false;
    bool showBVH_awake = false;
    bool showBVH_asleep = false;
    bool showBVH_static = false;
    bool showBVH_terrain = false;
};