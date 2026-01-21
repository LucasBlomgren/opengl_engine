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
    void toggleShowCollisionNormals();
    void toggleShowObjectLocalNormals();
    void toggleShowBVH_awake();
    void toggleShowBVH_asleep();
    void toggleShowBVH_static();
    void toggleShowBVH_terrain();

    bool isPaused() const;
    bool getAdvanceStep() const;
    bool getShowFPS() const;
    bool getShowObjectLocalNormals() const;
    bool getShowCollisionNormals() const;
    bool getShowAABB() const;
    bool getShowColliders() const;
    bool getShowContactPoints() const;
    bool getShowBVH_awake() const;
    bool getShowBVH_asleep() const;
    bool getShowBVH_static() const;
    bool getShowBVH_terrain() const;
    void setPlayerMode(bool);
    bool isPlayerMode() const;

    float deltaTime = 0.0f;

private:
    bool playerMode = false;
    bool advanceStep = false;
    bool paused = false;
    bool showFPS = false;
    bool showAABB = false;
    bool showColliders = false;
    bool showContactPoints = false;
    bool showCollisionNormals = false;
    bool showObjectLocalNormals = false;
    bool showBVH_awake = false;
    bool showBVH_asleep = false;
    bool showBVH_static = false;
    bool showBVH_terrain = false;
};