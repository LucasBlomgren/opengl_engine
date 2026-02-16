#pragma once

#include "game_object.h"
#include "collider.h"
#include "tri.h"

class World;
class TextureManager;
class LightManager;
class PhysicsEngine;
class ShaderManager;
class MeshManager;
class Renderer;

class SceneBuilder {
public:
    SceneBuilder(
        World& w, PhysicsEngine& pe, Renderer& re, TextureManager& tm, MeshManager& mm, ShaderManager& sm, LightManager& lm, std::mt19937& rng):
        world(w), physicsEngine(pe), renderer(re), textureManager(tm), meshManager(mm), shaderManager(sm), lightManager(lm), rng(rng)
    {}

    bool sceneDirty = true; 
    void objectRain(float& current_time, glm::vec3& pos, int mode);

    void createScene(int sceneID);
    void mainScene();
    void terrainScene();
    void tumblerScene();
    void tallStructureScene();
    void castleScene();
    void containerScene();
    void testFloorScene();
    void emptyFloorScene();
    void shapePileScene();

    //GameObject& createObject(
    //    const std::string& textureName,
    //    const std::string& meshName,
    //    ColliderType colliderType,
    //    const glm::vec3& pos,
    //    const glm::vec3& size,
    //    float mass,
    //    bool isStatic,
    //    const glm::quat& orientation = glm::quat(1, 0, 0, 0),
    //    float sleepCounterThreshold = 1.0f,
    //    bool asleep = 0,
    //    const glm::vec3& color = glm::vec3(255.0f, 255.0f, 255.0f),
    //    const bool seeThrough = false
    //);
    //int objectsAddedThisFrame = 0;

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

    void createHalo(
        float width,
        float height,
        float length,
        glm::vec3 baseRot,
        glm::vec3 rotDir,
        float rotSpeed,
        glm::vec3 pos,
        int segments,
        glm::vec3 color,
        bool createsShadows,
        bool seeThrough
    );

    // halos
    struct Halo { 
        glm::vec3 center; 
        glm::vec3 rotDir;  
        float rotSpeed; 
        std::vector<int> indices; 
    };
    std::vector<Halo> allHalos; 

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

    int playerObjectId = -1;
    int lightsState = 0;

private:
    World& world;
    PhysicsEngine& physicsEngine;
    Renderer& renderer;
    TextureManager& textureManager;
    MeshManager& meshManager;
    ShaderManager& shaderManager;
    LightManager& lightManager;
    std::mt19937& rng;

    int objectId = 0;
    std::vector<GameObject> staticObjects;

    //int lightsState = 0;
    int dayNightCycle = 0;

    float lastTime = 0.0f;
};