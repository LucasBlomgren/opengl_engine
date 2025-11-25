#pragma once

#include <vector>
#include "vertex.h"

extern std::vector<Vertex>        cubeVertices;
extern std::vector<unsigned int>  cubeIndices;

extern std::vector<Vertex>        sphereVertices;
extern std::vector<unsigned int>  sphereIndices;

extern std::vector<Vertex>        icoSphereVertices;
extern std::vector<unsigned int>  icoSphereIndices;

extern std::vector<Vertex> pylonVertices;
extern std::vector<unsigned int> pylonIndices;

extern std::vector<Vertex> teapotVertices;
extern std::vector<unsigned int> teapotIndices;

void loadCubeData();
void loadSphereData();
void loadIcoSphereData();