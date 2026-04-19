#pragma once

#include "game/player.h"
#include "editor/editor_main.h"
#include "physics/physics.h"
#include "game_object.h"
#include "collider.h"
#include "tri.h"

class Player;
class Editor::EditorMain;
class World;
class TextureManager;
class LightManager;
class ShaderManager;
class MeshManager;
class Renderer;

class SceneBuilder {
public:
    SceneBuilder(
        Player& p, Editor::EditorMain& e, World& w, PhysicsEngine& pe, Renderer& re, TextureManager& tm, MeshManager& mm, ShaderManager& sm, LightManager& lm, std::mt19937& rng):
        player(p), editor(e), world(w), physicsEngine(pe), renderer(re), textureManager(tm), meshManager(mm), shaderManager(sm), lightManager(lm), rng(rng)
    {}

    bool sceneDirty = true; 
    void objectRain(float& current_time, glm::vec3& pos, int mode);

    void createScene(int sceneID, bool isPlayerMode);
    void mainScene();
    void terrainScene();
    void tumblerScene();
    void tallStructureScene();
    void castleScene();
    void containerScene();
    void testFloorScene();
    void emptyFloorScene();
    void shapePileScene();

    void createGridFloor(
        glm::vec3& offset,
        glm::vec3& cellSize,
        int gridWidth,
        int gridLength
    );

    void createBlockPyramid(
        const std::string& textureName,
        glm::vec3 color,
        const glm::vec3& pos,
        int pWidth,
        int pHeight,
        float sWidth,
        float sLength,
        float sHeight,
        float sDistance,
        float sWeight,
        bool asleep
    );

    void createSpherePyramid(
        const std::string& textureName,
        glm::vec3 color,
        const glm::vec3& pos,
        int pWidth,
        int pHeight,
        float sRadius,
        float sDistance,
        float sWeight,
        bool asleep
    );

    void createBrickWall(
        glm::vec3 startPos,
        int wallDirection,
        float wallHeight,
        float wallWidth,
        glm::vec3 brickSize,
        float brickDistance,
        float brickWeight,
        int brickDecrease,
        glm::vec2 colorRange,
        bool fullColorRange
    );

    void generateFlatTerrain(
        glm::vec3 offset,
        int   gridSizeX,
        int   gridSizeZ,
        float cellSize,
        float maxHeight
    );

    void smoothHeightMap(std::vector<std::vector<float>>& H, float smoothness, int passes);

    struct TerrainData {
        std::vector<Vertex> vertices;
        std::vector<uint32_t> indices;
        std::vector<Tri> triangles; 
    };

    TerrainData terrainData; 
    TerrainData& getTerrainData();

    void toggleLightsState();
    void setLights();
    void toggleDayNight();

    float randomRange(float start, float end);

    int lightsState = 0;

private:
    Player& player;
    Editor::EditorMain& editor;
    World& world;
    PhysicsEngine& physicsEngine;
    Renderer& renderer;
    TextureManager& textureManager;
    MeshManager& meshManager;
    ShaderManager& shaderManager;
    LightManager& lightManager;
    std::mt19937& rng;

    int objectId = 0;
    int dayNightCycle = 0;
    float lastTime = 0.0f;
};