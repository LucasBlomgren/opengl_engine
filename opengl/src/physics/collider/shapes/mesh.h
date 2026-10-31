#pragma once

#include <iostream>

#include <vector>
#include <glm/glm.hpp>

#include "physics/bvh/bvh.h"

namespace physics::internal {

struct MeshFace {
   uint32_t firstIndex;
   uint16_t vertexCount;
   glm::vec3 normal;
};

struct Mesh {
   std::vector<glm::vec3> vertices;
   std::vector<MeshFace> faces;
   std::vector<uint32_t> faceIndices;
   BVHTree bvh;
   AABB aabb;

   void printData() {
      std::cout << "Vertices: " << std::endl;
      for (glm::vec3& v: vertices) {
         std::cout << "(" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
      }
      std::cout << "\n";

      std::cout << "Face indices: " << std::endl;
      for (uint32_t& v : faceIndices) {
         std::cout << v << ", ";
      }
      std::cout << "\n";

      std::cout << "Faces: " << std::endl;
      for (MeshFace& face : faces) {
         std::cout << "[";
         for (int i = 0; i < face.vertexCount; i++) {
            std::cout << faceIndices[face.firstIndex + i] << ", ";
         }
         std::cout << "],";
      }
      std::cout << "\n";
   }
};

struct MeshData {
   Mesh mesh;
   ColliderTransformCache transformCache;
};

}