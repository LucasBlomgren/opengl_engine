#include "physics/collider/geometry/mesh_geometry.h"

#include "physics/collider/aabb.h"

#include <iostream>
#include <utility>
#include <glm/glm.hpp>
#include <unordered_map>
#include <glm/gtx/hash.hpp>

namespace physics::internal {

void MeshGeometry::deduplicateVertices() {
   std::unordered_map<glm::vec3, uint32_t> vertexToIndex;
   std::vector<glm::vec3> uniqueVertices;

   for (uint32_t& index : faceIndices) {
      const glm::vec3& v = vertices[index];

      auto [it, inserted] =
         vertexToIndex.emplace(v, static_cast<uint32_t>(uniqueVertices.size()));

      if (inserted)
         uniqueVertices.push_back(v);

      index = it->second;
   }

   vertices = std::move(uniqueVertices);
}

void MeshGeometry::computeFaceNormals() {
   for (MeshFace& face : faces) {
      glm::vec3& a = vertices[faceIndices[face.start]];
      glm::vec3& b = vertices[faceIndices[face.start + 1]];
      glm::vec3& c = vertices[faceIndices[face.start + 2]];

      glm::vec3 edgeA = b - a;
      glm::vec3 edgeB = c - b;

      face.normal = glm::normalize(glm::cross(edgeA, edgeB));
   }
}

void MeshGeometry::computeInertiaMatrix() {
   glm::vec3 meshCOM{ 0.0f };
   float meshVolume = 0.0f;

   glm::vec3 P{ 0.0f }; // avg point
   for (glm::vec3& v : vertices) {
      P += v;
   }
   P /= vertices.size();

   for (MeshFace& face : faces)
   {
      float tetrahedronVolume = 0.0f;
      // dela upp pyramidens bas i trianglar och summera tetraedrarnas volymer
      for (uint32_t i = 1; i < face.count - 1; i++) {
         glm::vec3& v0 = vertices[faceIndices[face.start]];
         glm::vec3& v1 = vertices[faceIndices[face.start + i]];
         glm::vec3& v2 = vertices[faceIndices[face.start + i + 1]];

         glm::vec3 tetrahedron_COM = (P + v0 + v1 + v2) / 4.0f;

         // n = (v1-v0) kryss (v2-v0)
         // 
         // area_bas = abs(kryss) / 2
         // 
         // höjd = (abs(P-v0) dot n) / abs(n)
         //
         // tetraedron_volume = (area_bas * höjd) / 3   -> per tetrahedron
         //
         // meshVolume += tetraedron_volume
         // meshCOM += tetraedron_volume * tetraedron_COM
      }

      // meshCOM += (tetrahedronVolume * tetrahedron_COM) / totalVolume
   }
}

AABB MeshGeometry::computeAABB() {
   bool initialized = false;
   AABB aabb;

   for (MeshFace& face : faces) {
      glm::vec3 minimum = vertices[faceIndices[face.start]];
      glm::vec3 maximum = minimum;

      for (size_t i = 1; i < face.count; i++) {
         minimum = glm::min(minimum, vertices[faceIndices[face.start + i]]);
         maximum = glm::max(maximum, vertices[faceIndices[face.start + i]]);
      }

      // set root AABB
      if (!initialized) {
         aabb.min = minimum;
         aabb.max = maximum;
         initialized = true;
      }
      else {
         aabb.growToInclude(minimum);
         aabb.growToInclude(maximum);
      }
   }

   return aabb;
}

void MeshGeometry::printData() {
   std::cout << "Vertices: " << std::endl;
   for (glm::vec3& v : vertices) {
      std::cout << "(" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
   }
   std::cout << "\n";

   std::cout << "MeshFace indices: " << std::endl;
   for (uint32_t& v : faceIndices) {
      std::cout << v << ", ";
   }
   std::cout << "\n";

   std::cout << "MeshFaces: " << std::endl;
   for (MeshFace& face : faces) {
      std::cout << "[";
      for (int i = 0; i < face.count; i++) {
         std::cout << faceIndices[face.start + i] << ", ";
      }
      std::cout << "],";
   }
   std::cout << "\n";
}

}