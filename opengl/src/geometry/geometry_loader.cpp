#include "pch.h"
#include "geometry_loader.h"
#include "mesh_loader.h"

std::vector<Vertex>        cubeVertices;
std::vector<unsigned int>  cubeIndices;

std::vector<Vertex>        sphereVertices;
std::vector<unsigned int>  sphereIndices;

std::vector<Vertex>        icoSphereVertices;
std::vector<unsigned int>  icoSphereIndices;

void loadCubeData() {
    cubeVertices = loadVerticesFromTxt("src/assets/cube_vertices.txt");
    cubeIndices = loadIndicesFromTxt("src/assets/cube_indices.txt");
}

void loadSphereData() {
    sphereVertices = loadVerticesFromTxt("src/assets/sphere_vertices.txt");
    sphereIndices = loadIndicesFromTxt("src/assets/sphere_indices.txt");
}

void loadIcoSphereData() {
    icoSphereVertices = loadVerticesFromTxt("src/assets/icoSphere_vertices.txt");
    icoSphereIndices = loadIndicesFromTxt("src/assets/icoSphere_indices.txt");
}
