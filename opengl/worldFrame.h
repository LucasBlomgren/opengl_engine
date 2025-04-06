#pragma once
#include <iostream>
#include <vector>
#include "shader.h"

unsigned int setup_worldFrame();
void draw_worldFrame(const Shader& shader, const unsigned int& VAO_xyz);