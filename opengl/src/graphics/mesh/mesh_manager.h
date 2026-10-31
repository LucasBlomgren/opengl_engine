#pragma once

#include "mesh.h"
#include "mesh_loader.h"
#include <unordered_map>

class MeshManager {
public:
   MeshManager() {
      // Debug Arrow
      std::vector<Vertex> debugArrowVerts = loadVerticesFromTxt("src/assets/debugarrow_vertices.txt");
      std::vector<unsigned int> debugArrowIndices = loadIndicesFromTxt("src/assets/debugarrow_indices.txt");
      recenterVertices(debugArrowVerts);
      meshes.emplace("debug_arrow", Mesh(debugArrowVerts, debugArrowIndices));

      // Cube
      std::vector<Vertex> cubeVerts = loadVerticesFromTxt("src/assets/cube_vertices.txt");
      std::vector<unsigned int> cubeIndices = loadIndicesFromTxt("src/assets/cube_indices.txt");
      meshes.emplace("cube", Mesh(cubeVerts, cubeIndices));

      // Sphere
      std::vector<Vertex> sphereVerts = loadVerticesFromTxt("src/assets/sphere_vertices.txt");
      std::vector<unsigned int> sphereIndices = loadIndicesFromTxt("src/assets/sphere_indices.txt");
      meshes.emplace("sphere", Mesh(sphereVerts, sphereIndices));

      // Cylinder render
      std::vector<Vertex> cylinder_renderVerts = loadVerticesFromTxt("src/assets/cylinder_render_vertices.txt");
      std::vector<unsigned int> cylinder_renderIndices = loadIndicesFromTxt("src/assets/cylinder_render_indices.txt");
      recenterVertices(cylinder_renderVerts);
      meshes.emplace("cylinder_render", Mesh(cylinder_renderVerts, cylinder_renderIndices));

      // Cylinder collider
      std::vector<Vertex> cylinder_colliderVerts = loadVerticesFromTxt("src/assets/cylinder_collider_vertices.txt");
      std::vector<unsigned int> cylinder_colliderIndices = loadIndicesFromTxt("src/assets/cylinder_collider_indices.txt");
      recenterVertices(cylinder_colliderVerts);

      Mesh mesh(cylinder_colliderVerts, cylinder_colliderIndices);
      mesh.faces = loadFacesFromTxt("src/assets/cylinder_collider_faces.txt");
      meshes.emplace("cylinder_collider", mesh);

      //// Teapot
      //std::vector<Vertex> teapotVerts         = loadVerticesFromTxt("src/assets/teapot_vertices.txt");
      //std::vector<unsigned int> teapotIndices = loadIndicesFromTxt("src/assets/teapot_indices.txt");
      //recenterVertices(teapotVerts);
      //meshes.emplace("teapot", Mesh(teapotVerts, teapotIndices));

      //// Pylon
      //std::vector<Vertex> pylonVerts          = loadVerticesFromTxt("src/assets/pylon_vertices.txt");
      //std::vector<unsigned int> pylonIndices  = loadIndicesFromTxt("src/assets/pylon_indices.txt");
      //recenterVertices(pylonVerts);
      //meshes.emplace("pylon", Mesh(pylonVerts, pylonIndices));

      //// Girl
      //std::vector<Vertex> girlVerts           = loadVerticesFromTxt("src/assets/girl_vertices.txt");
      //std::vector<unsigned int> girlIndices   = loadIndicesFromTxt("src/assets/girl_indices.txt");
      //recenterVertices(girlVerts);
      //meshes.emplace("girl", Mesh(girlVerts, girlIndices));

      //// Tank
      //std::vector<Vertex> tankVerts           = loadVerticesFromTxt("src/assets/tank_vertices.txt");
      //std::vector<unsigned int> tankIndices   = loadIndicesFromTxt("src/assets/tank_indices.txt");
      //recenterVertices(tankVerts);
      //meshes.emplace("tank", Mesh(tankVerts, tankIndices));
   };

   Mesh* getMesh(const std::string& name) const;

private:
   static std::unordered_map<std::string, Mesh> meshes;
   void recenterVertices(std::vector<Vertex>& vertices);
};