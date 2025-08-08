#pragma once

#define GLM_FORCE_SIMD_AVX2 
#define GLM_ENABLE_EXPERIMENTAL

#include <stb/stb_image.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <chrono>
#include <iomanip>
#include <span>
#include <string>
#include <unordered_map>
#include <stdexcept>


#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/io.hpp>

#include "shader.h"
#include "camera.h"
