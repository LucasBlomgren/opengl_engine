#pragma once

#include "game_object.h"
#include "collider.h"

class TextureManager;
class LightManager;
class PhysicsEngine;

class SceneBuilder {
public:
   bool sceneDirty = true; 
   void setPointers(PhysicsEngine* pm, TextureManager* tm, LightManager* lm, std::mt19937& rng);
   void objectRain(float& current_time, std::mt19937& rng, int mode);

   void createScene(PhysicsEngine& physicsEngine, int sceneID);
   void mainScene();
   void terrainScene();
   void tumblerScene();
   void tallStructureScene();
   void castleScene();
   void containerScene();
   void testFloorScene();

   GameObject& createObject(
      const std::string& textureName,
      ColliderType colliderType,
      const glm::vec3& pos,
      const glm::vec3& size,
      float mass,
      bool isStatic,
      const glm::quat& orientation = glm::quat(1, 0, 0, 0),
      float sleepCounterThreshold = 1.0f,
      bool asleep = 0,
      const glm::vec3& color = glm::vec3(255.0f, 255.0f, 255.0f)
   );
   int objectsAddedThisFrame = 0;

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
       int brickWeight,
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
       bool createsShadows
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

   std::vector<GameObject>& getDynamicObjects();
   void toggleLightsState();
   void setLights();
   void toggleDayNight();

   float randomRange(float start, float end);

   int playerObjectId = -1;

private:
   PhysicsEngine* physicsEngine   = nullptr;
   TextureManager* textureManager = nullptr;
   LightManager* lightManager     = nullptr;

   int objectId;
   std::vector<GameObject> dynamicObjects;
   std::vector<GameObject> staticObjects;

   int lightsState = 0;
   int dayNightCycle = 0;

   std::mt19937* rng;
   float lastTime = 0.0f;
};