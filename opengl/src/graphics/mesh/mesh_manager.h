#pragma once

#include "mesh.h"
#include "mesh_loader.h"
#include <unordered_map>

class MeshManager {
public:
    MeshManager() {
        std::vector<Vertex> cubeVerts           = loadVerticesFromTxt("src/assets/cube_vertices.txt");
        std::vector<unsigned int> cubeIndices   = loadIndicesFromTxt("src/assets/cube_indices.txt");

        std::vector<Vertex> sphereVerts         = loadVerticesFromTxt("src/assets/sphere_vertices.txt");
        std::vector<unsigned int> sphereIndices = loadIndicesFromTxt("src/assets/sphere_indices.txt");

        std::vector<Vertex> teapotVerts         = loadVerticesFromTxt("src/assets/teapot_vertices.txt");
        std::vector<unsigned int> teapotIndices = loadIndicesFromTxt("src/assets/teapot_indices.txt");

        std::vector<Vertex> pylonVerts          = loadVerticesFromTxt("src/assets/pylon_vertices.txt");
        std::vector<unsigned int> pylonIndices  = loadIndicesFromTxt("src/assets/pylon_indices.txt");

        meshes.emplace("cube", Mesh(cubeVerts, cubeIndices));
        meshes.emplace("sphere", Mesh(sphereVerts, sphereIndices));
        meshes.emplace("teapot", Mesh(teapotVerts, teapotIndices));
        meshes.emplace("pylon", Mesh(pylonVerts, pylonIndices));
    };

    Mesh* getMesh(const std::string& name);

private:
    static std::unordered_map<std::string, Mesh> meshes;
};