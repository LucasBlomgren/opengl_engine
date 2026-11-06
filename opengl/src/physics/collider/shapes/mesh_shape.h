#pragma once

#include "physics/collider/geometry/mesh_geometry.h"
#include "physics/collider/aabb.h"
#include "physics/collider/collider_transform_cache.h"

#include <cstdint>
#include <vector>
#include <glm/vec3.hpp>

namespace physics::internal {

struct MeshShape : MeshGeometry {
   AABB localAABB;
   // MeshBVH bvh;

   MeshShape(
      const std::vector<glm::vec3>& vertices,
      const std::vector<MeshFace>& faces,
      const std::vector<uint32_t>& faceIndices)
      : MeshGeometry(vertices, faces, faceIndices)
   {
      deduplicateVertices();
      computeFaceNormals();
      localAABB = initBVH();
   }

   AABB initBVH();
};

struct MeshData {
   MeshShape mesh;
   ColliderTransformCache transformCache;
};

}