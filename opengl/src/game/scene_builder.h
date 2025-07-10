#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "physics.h"
#include "game_object.h"
#include "geometry/vertex.h"
#include "texture_manager.h"
#include "light_manager.h"
#include "collider.h"
#include "geometry/sphere_data.h"
#include "geometry/cube_data.h"

class SceneBuilder {
public:
   bool sceneDirty = true; 
   void setPointers(TextureManager* tm, LightManager* lm, std::mt19937& rng);
   void objectRain(float& current_time, std::mt19937& rng, int mode);

   void createScene(PhysicsEngine& physicsEngine);
   void mainScene();
   void testScene();

   GameObject& createObject(
      const std::string& textureName,
      const ColliderType colliderType,
      const glm::vec3& pos,
      const glm::vec3& size,
      float mass,
      bool isStatic,
      const glm::quat& orientation = glm::quat(1, 0, 0, 0),
      float sleepCounterThreshold = 1.0f,
      bool asleep = 0,
      const glm::vec3& color = glm::vec3(255.0f, 255.0f, 255.0f)
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

   float randomRange(float start, float end);

   // halos
   glm::vec3 haloACenter;
   std::vector<int> haloA;

   glm::vec3 haloBCenter;
   std::vector<int> haloB;

   glm::vec3 haloCCenter;
   std::vector<int> haloC;

private:
   TextureManager* textureManager = nullptr;
   LightManager* lightManager = nullptr;

   int lightsState = 0;

   int objectId;
   std::vector<GameObject> dynamicObjects;
   std::vector<GameObject> staticObjects;

   std::mt19937* rng;
   float lastTime = 0.0f;
};