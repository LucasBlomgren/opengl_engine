#pragma once

#include <vector>

#include "physics/public/handles.h"

namespace physics::internal {

class Tri;

struct TerrainPair {
   BodyHandle body;
   ColliderHandle collider;
   std::vector<Tri*> tris;
};
struct DynamicPair {
   BodyHandle bodyA;
   BodyHandle bodyB;
   ColliderHandle colliderA;
   ColliderHandle colliderB;
};

struct SpeculativeDynamicPair {
   BodyHandle bodyA;
   BodyHandle bodyB;
   ColliderHandle colliderA;
   ColliderHandle colliderB;
   BodyHandle sweepOwner;
};

struct SpeculativeTerrainPair {
   BodyHandle body;
   ColliderHandle collider;
   std::vector<Tri*> tris;
};

struct PairBatch {
   std::vector<DynamicPair> dynamicPairs;
   std::vector<TerrainPair> terrainPairs;

   std::vector<SpeculativeDynamicPair> speculativeDynamicPairs;
   std::vector<SpeculativeTerrainPair> speculativeTerrainPairs;

   void clear() {
      dynamicPairs.clear();
      terrainPairs.clear();

      speculativeDynamicPairs.clear();
      speculativeTerrainPairs.clear();
   }
};

}
