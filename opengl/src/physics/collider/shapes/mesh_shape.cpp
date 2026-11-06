#include "mesh_shape.h"

#include "physics/bvh/bvh.h"

#include <iostream>
#include <utility>
#include <glm/glm.hpp>
#include <unordered_map>
#include <glm/gtx/hash.hpp>

namespace physics::internal {

AABB MeshShape::initBVH() {
   std::vector<AABB> aabbs;
   aabbs.reserve(faces.size());

   bool initialized = false;
   AABB rootAABB;

   for (MeshFace& face : faces) {
      glm::vec3 minimum = vertices[faceIndices[face.start]];
      glm::vec3 maximum = minimum;

      for (size_t i = 1; i < face.count; i++) {
         minimum = glm::min(minimum, vertices[faceIndices[face.start + i]]);
         maximum = glm::max(maximum, vertices[faceIndices[face.start + i]]);
      }

      aabbs.emplace_back(minimum , maximum);

      // grow root AABB
      if (!initialized) {
         rootAABB.min = minimum;
         rootAABB.max = maximum;
         initialized = true;
      }
      else {
         rootAABB.growToInclude(minimum);
         rootAABB.growToInclude(maximum);
      }
   }

   return rootAABB;

   //MeshBVH.build(aabbs);
}

}