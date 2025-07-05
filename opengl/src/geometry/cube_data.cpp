#include "cube_data.h"

extern std::vector<Vertex> cubeVertices = {
    // pos                          // color          // normal         // texCoord
    {glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,0,-1), glm::vec2(0.0f, 0.0f)},  // FRONT
    {glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,0,-1), glm::vec2(1.0f, 0.0f)},
    {glm::vec3(0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,0,-1), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,0,-1), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(-0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,0,-1), glm::vec2(0.0f, 1.0f)},
    {glm::vec3(0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,0,-1), glm::vec2(0.0f, 0.0f)},

    {glm::vec3(-0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,0,1), glm::vec2(0.0f, 0.0f)},  // BACK
    {glm::vec3(0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,0,1), glm::vec2(1.0f, 0.0f)},
    {glm::vec3(0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,0,1), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,0,1), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(-0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,0,1), glm::vec2(0.0f, 1.0f)},
    {glm::vec3(-0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,0,1), glm::vec2(0.0f, 0.0f)},

    {glm::vec3(-0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(-1,0,0), glm::vec2(1.0f, 0.0f)},  // LEFT
    {glm::vec3(-0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(-1,0,0), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(-1,0,0), glm::vec2(0.0f, 1.0f)},
    {glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(-1,0,0), glm::vec2(0.0f, 1.0f)},
    {glm::vec3(-0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(-1,0,0), glm::vec2(0.0f, 0.0f)},
    {glm::vec3(-0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(-1,0,0), glm::vec2(1.0f, 0.0f)},

    {glm::vec3(0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(1,0,0), glm::vec2(1.0f, 0.0f)},  // RIGHT
    {glm::vec3(0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(1,0,0), glm::vec2(0.0f, 0.0f)},
    {glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(1,0,0), glm::vec2(0.0f, 1.0f)},
    {glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(1,0,0), glm::vec2(0.0f, 1.0f)},
    {glm::vec3(0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(1,0,0), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(1,0,0), glm::vec2(1.0f, 0.0f)},

    {glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,-1,0), glm::vec2(0.0f, 1.0f)},  // BOT
    {glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,-1,0), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,-1,0), glm::vec2(1.0f, 0.0f)},
    {glm::vec3(0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,-1,0), glm::vec2(1.0f, 0.0f)},
    {glm::vec3(-0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,-1,0), glm::vec2(0.0f, 0.0f)},
    {glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,-1,0), glm::vec2(0.0f, 1.0f)},

    {glm::vec3(-0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,1,0), glm::vec2(0.0f, 1.0f)},  // TOP 
    {glm::vec3(-0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,1,0), glm::vec2(0.0f, 0.0f)},
    {glm::vec3(0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,1,0), glm::vec2(1.0f, 0.0f)},
    {glm::vec3(0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,1,0), glm::vec2(1.0f, 0.0f)},
    {glm::vec3(0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,1,0), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(-0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,1,0), glm::vec2(0.0f, 1.0f)}
};

extern std::vector<unsigned int> cubeIndices = {
    0, 1, 2,
    3, 4, 5,
    6, 7, 8,
    9, 10, 11,
    12, 13, 14,
    15, 16, 17,
    18, 19, 20,
    21, 22, 23,
    24, 25, 26,
    27, 28, 29,
    30, 31, 32,
    33, 34, 35
};