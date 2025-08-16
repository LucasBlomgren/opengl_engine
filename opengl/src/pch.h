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



#include <atomic>
namespace glcount {
    inline std::atomic<int> vaos{ 0 }, vbos{ 0 }, ebos{ 0 };

    inline void incVAO(int n = 1) { vaos += n; }
    inline void decVAO(int n = 1) { vaos -= n; }
    inline void incVBO(int n = 1) { vbos += n; }
    inline void decVBO(int n = 1) { vbos -= n; }
    inline void incEBO(int n = 1) { ebos += n; }
    inline void decEBO(int n = 1) { ebos -= n; }

    inline void print() {
        printf("[GL] live VAO=%d VBO=%d EBO=%d\n",
            (int)vaos.load(), (int)vbos.load(), (int)ebos.load());
    }
}