#pragma once

#include <cstdint>
#include <vector>
#include <glm/vec3.hpp>

namespace physics::internal {

class AABB;

struct MeshFace {
   uint32_t start;   // first index
   uint16_t count;   // vertex count
   glm::vec3 normal;
};

struct MeshGeometry {
   std::vector<glm::vec3> vertices;
   std::vector<MeshFace> faces;
   std::vector<uint32_t> faceIndices;

   MeshGeometry(
      const std::vector<glm::vec3>& vertices,
      const std::vector<MeshFace>& faces,
      const std::vector<uint32_t>& faceIndices)
      : vertices(vertices)
      , faces(faces)
      , faceIndices(faceIndices)
   {}

   void deduplicateVertices();
   void computeFaceNormals();
   void computeInertiaMatrix();
   AABB computeAABB();

   void printData();
};

}