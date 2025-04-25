#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include <array>
#include <vector>

#include "shader.h"
#include "vertex.h"

struct Edge {
    int id;
    bool isMin;
    float coord;

    Edge(int id = 0, bool isMin = 0, float coord = 0)
    : id(id), isMin(isMin), coord(coord) {}
};

struct Box {
    struct Min {
        Edge x;
        Edge y;
        Edge z;
        Min(int id) : x(id, true, 0), y(id, true, 0), z(id, true, 0) {}
    } min;

    struct Max {
        Edge x;
        Edge y;
        Edge z;
        Max(int id) : x(id, false, 0), y(id, false, 0), z(id, false, 0) {}
    } max;

    Box(int id) : min(id), max(id) {}
};

struct defaultBox {
    glm::vec3 min;
    glm::vec3 max;
};

struct defaultFaces {
    std::array<glm::vec3,4> minX;
    std::array<glm::vec3,4> maxX;
    std::array<glm::vec3,4> minY;
    std::array<glm::vec3,4> maxY;
    std::array<glm::vec3,4> minZ;
    std::array<glm::vec3,4> maxZ;
};

class AABB 
{
public:
    unsigned int VAO;
    unsigned int VBO;
    Box Box;
    defaultBox defaultBox;
    defaultFaces defaultFaces;
    glm::vec3 color;
    std::vector<float> bufferData;

    AABB(int id)
    : 
    Box(id) 
    {}

    void Init(const std::vector<glm::vec3>& vertices, glm::vec3& scale)
    {
            std::array<float, 6> extremePoints = getExtremePoints(vertices);
            setDefaultBox(extremePoints);
            setBox(extremePoints);
            setDefaultFaces(extremePoints);
    }

    void update(std::vector<glm::vec3>& vertices,glm::mat4& modelMatrix, glm::vec3& position)
    {
        glm::mat3 model3x3 = glm::mat3(modelMatrix);
        Transform_Box(model3x3, position);
    }

private:
    void Transform_Box(const glm::mat3& M, const glm::vec3& T)
    {
        float  a, b;
        float  Amin[3], Amax[3];
        float  Bmin[3], Bmax[3];
        int  i, j;

        Amin[0] = defaultBox.min.x; Amax[0] = defaultBox.max.x;
        Amin[1] = defaultBox.min.y; Amax[1] = defaultBox.max.y;
        Amin[2] = defaultBox.min.z; Amax[2] = defaultBox.max.z;


        Bmin[0] = Bmax[0] = T.x;
        Bmin[1] = Bmax[1] = T.y;
        Bmin[2] = Bmax[2] = T.z;

        for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) {
            a = (M[j][i] * Amin[j]);
            b = (M[j][i] * Amax[j]);

            if (a < b) {
                Bmin[i] += a;
                Bmax[i] += b;
            }
            else {
                Bmin[i] += b;
                Bmax[i] += a;
            }
        }

        setBox({ Bmin[0], Bmin[1], Bmin[2], Bmax[0], Bmax[1], Bmax[2] });
    }

    void setBox(const std::array<float, 6>& extremePoints)
    {
        Box.min.x.coord = extremePoints[0]; Box.max.x.coord = extremePoints[3];
        Box.min.y.coord = extremePoints[1]; Box.max.y.coord = extremePoints[4];
        Box.min.z.coord = extremePoints[2]; Box.max.z.coord = extremePoints[5];
    }

    void setDefaultBox(const std::array<float, 6>& extremePoints)
    {
        defaultBox.min.x = extremePoints[0]; defaultBox.max.x = extremePoints[3];
        defaultBox.min.y = extremePoints[1]; defaultBox.max.y = extremePoints[4];
        defaultBox.min.z = extremePoints[2]; defaultBox.max.z = extremePoints[5];
    }

    void setDefaultFaces(const std::array<float, 6>& extremePoints)
    {
        float min_X = extremePoints[0];
        float min_Y = extremePoints[1];
        float min_Z = extremePoints[2];
        float max_X = extremePoints[3];
        float max_Y = extremePoints[4];
        float max_Z = extremePoints[5];

        defaultFaces.minX = { glm::vec3(min_X, min_Y, min_Z), glm::vec3(min_X, min_Y, max_Z), glm::vec3(min_X, max_Y, max_Z), glm::vec3(min_X, max_Y, min_Z) };
        defaultFaces.maxX = { glm::vec3(max_X, min_Y, max_Z), glm::vec3(max_X, min_Y, min_Z), glm::vec3(max_X, max_Y, min_Z), glm::vec3(max_X, max_Y, max_Z) };
        defaultFaces.minY = { glm::vec3(max_X, min_Y, min_Z), glm::vec3(max_X, min_Y, max_Z), glm::vec3(min_X, min_Y, max_Z), glm::vec3(min_X, min_Y, min_Z) };
        defaultFaces.maxY = { glm::vec3(min_X, max_Y, min_Z), glm::vec3(min_X, max_Y, max_Z), glm::vec3(max_X, max_Y, max_Z), glm::vec3(max_X, max_Y, min_Z) };
        defaultFaces.minZ = { glm::vec3(min_X, min_Y, min_Z), glm::vec3(min_X, max_Y, min_Z), glm::vec3(max_X, max_Y, min_Z), glm::vec3(max_X, min_Y, min_Z) };
        defaultFaces.maxZ = { glm::vec3(min_X, min_Y, max_Z), glm::vec3(max_X, min_Y, max_Z), glm::vec3(max_X, max_Y, max_Z), glm::vec3(min_X, max_Y, max_Z) };
    }

    std::array<float, 6> getExtremePoints(const std::vector<glm::vec3>& vertices)
    {
        // Initialize min and max values
        float max_X = std::numeric_limits<float>::lowest();
        float min_X = std::numeric_limits<float>::max();
        float max_Y = std::numeric_limits<float>::lowest();
        float min_Y = std::numeric_limits<float>::max();
        float max_Z = std::numeric_limits<float>::lowest();
        float min_Z = std::numeric_limits<float>::max();

        // Loop through the vertices and update min and max values
        for (size_t i = 0; i < vertices.size(); i++) {
            float x = vertices[i].x;
            float y = vertices[i].y;
            float z = vertices[i].z;

            if (x > max_X) max_X = x;
            if (x < min_X) min_X = x;
            if (y > max_Y) max_Y = y;
            if (y < min_Y) min_Y = y;
            if (z > max_Z) max_Z = z;
            if (z < min_Z) min_Z = z;
        }

        return { min_X, min_Y, min_Z, max_X, max_Y, max_Z };
    }
};