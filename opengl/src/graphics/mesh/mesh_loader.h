#pragma once
#include <vector>
#include <string>
#include "vertex.h"

// Laddar vertex-data från en txt-fil
std::vector<Vertex> loadVerticesFromTxt(const std::string& path);

// Laddar index-data från en txt-fil
std::vector<unsigned int> loadIndicesFromTxt(const std::string& path);