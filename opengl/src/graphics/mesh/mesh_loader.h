#pragma once
#include <vector>
#include <string>
#include "geometry/vertex.h"

std::vector<Vertex> loadVerticesFromTxt(const std::string& path);
std::vector<unsigned int> loadIndicesFromTxt(const std::string& path);
std::vector<std::vector<uint32_t>> loadFacesFromTxt(const std::string& path);