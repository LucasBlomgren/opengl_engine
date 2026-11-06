#pragma once

#include "physics/collider/geometry/mesh_geometry.h"

#include <cstdint>
#include <vector>
#include <glm/vec3.hpp>

namespace physics::internal {

struct Edge {
   uint32_t vertexA;
   uint32_t vertexB;
   uint32_t faceA;
   uint32_t faceB;
};

struct Hull : MeshGeometry {
   std::vector<Edge> edges;
   AABB localAABB;

   Hull(
      const std::vector<glm::vec3>& vertices,
      const std::vector<MeshFace>& faces,
      const std::vector<uint32_t>& faceIndices)
      : MeshGeometry(vertices, faces, faceIndices)
   {
      MeshGeometry::deduplicateVertices();
      MeshGeometry::computeFaceNormals();
      localAABB = MeshGeometry::computeAABB();
   }
};

}