#pragma once

#include <vector>
#include "vertex.h"

extern std::vector<Vertex>        cubeVertices;
extern std::vector<unsigned int>  cubeIndices;

extern std::vector<Vertex>        sphereVertices;
extern std::vector<unsigned int>  sphereIndices;

extern std::vector<Vertex>        icoSphereVertices;
extern std::vector<unsigned int>  icoSphereIndices;

void loadCubeData();
void loadSphereData();
void loadIcoSphereData();