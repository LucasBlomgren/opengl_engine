#pragma once
#include <iostream>
#include <vector>
#include "shader.h"

unsigned int setup_xyzObject();
void draw_xyzObject(const Shader& shader, const unsigned int& VAO_xyz);