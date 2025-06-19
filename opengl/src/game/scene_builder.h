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
   void setPointers(TextureManager* tm, LightManager* lm, std::mt19937& rng);
   void objectRain(float& current_time, std::mt19937& rng);
   void createScene(PhysicsEngine& physicsEngine);
   GameObject& createObject(
      const std::string& textureName,
      ColliderType colliderType,
      glm::vec3 pos,
      glm::vec3 size,
      float mass,
      bool isStatic,
      glm::quat orientation = glm::quat(1, 0, 0, 0),
      float sleepCounterThreshold = 1.0f,
      bool asleep = 0,
      glm::vec3 color = glm::vec3(255.0f, 255.0f, 255.0f)
   );

   void mainScene();
   void testScene();

   void generateFlatTerrain(
       int   gridSizeX,
       int   gridSizeZ,
       float cellSize,
       float maxHeight,
       float flatness /* i [0,1], 0 = helt plant, 1 = full variation */
   );

   std::vector<Tri>& getTerrainTriangles();
   std::vector<GameObject>& getDynamicObjects();
   void toggleLightsState();
   void setLights();

   int randomRange(int start, int end);

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
   std::vector<Tri> terrainTriangles;

   std::mt19937* rng;
   float lastTime = 0.0f;
};