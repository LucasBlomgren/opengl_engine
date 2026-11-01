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
   AABB localAABB;
   //MeshBVH bvh;

   Mesh(
      const std::vector<glm::vec3>& vertices,
      const std::vector<MeshFace>& faces,
      const std::vector<uint32_t>& faceIndices)
      : vertices(vertices)
      , faces(faces)
      , faceIndices(faceIndices)
   {
      initBVH();
   }

   void initBVH() {
      std::vector<AABB> aabbs;
      aabbs.reserve(faces.size());

      bool initialized = false;

      for (MeshFace& face : faces) {
         glm::vec3 minimum = vertices[faceIndices[face.firstIndex]];
         glm::vec3 maximum = minimum;

         for (size_t i = 1; i < face.vertexCount; i++) {
            minimum = glm::min(minimum, vertices[faceIndices[face.firstIndex + i]]);
            maximum = glm::max(maximum, vertices[faceIndices[face.firstIndex + i]]);
         }

         AABB aabb;
         aabb.worldMin = minimum;
         aabb.worldMax = maximum;
         aabb.worldCenter = (minimum + maximum) * 0.5f;
         aabb.worldHalfExtents = (maximum - minimum) * 0.5f;
         aabbs.push_back(aabb);

         // set root AABB
         if (!initialized) {
            localAABB = aabb;
            initialized = true;
         }
         else {
            localAABB.growToInclude(minimum);
            localAABB.growToInclude(maximum);
         }
      }

      localAABB.worldCenter =
         (localAABB.worldMin + localAABB.worldMax) * 0.5f;

      localAABB.worldHalfExtents =
         (localAABB.worldMax - localAABB.worldMin) * 0.5f;

      //MeshBVH.build(aabbs);
   }

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