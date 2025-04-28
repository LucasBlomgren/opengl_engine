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
        Edge x, y, z;
        Min(int id) : 
            x(id, true, 0), 
            y(id, true, 0), 
            z(id, true, 0) {}
    } min;

    struct Max {
        Edge x ,y, z;
        Max(int id) : 
            x(id, false, 0), 
            y(id, false, 0), 
            z(id, false, 0) {}
    } max;

    Box(int id) : min(id), max(id) {}
};

struct localFaces {
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
    Box Box;
    localFaces localFaces;

    glm::vec3 localMin;
    glm::vec3 localMax;
    glm::vec3 worldMin;
    glm::vec3 worldMax;

    glm::vec3 worldCenter;
    glm::vec3 halfExtents;

    AABB(int id) : Box(id) { }

    void Init(const std::vector<glm::vec3>& vertices) {
        computeFromVertices(vertices);
        setLocalFaces();
    }

    void update(glm::mat4& model, glm::vec3& pos, glm::vec3& scale, bool hasRotated) {
        if (hasRotated) {
            glm::mat3 model3x3 = glm::mat3(model);
            transform_withRotation(model3x3, pos);
        }
        else { 
            transform_noRotation(model, pos, scale); 
        }

        worldCenter = (worldMin + worldMax) * 0.5f;
        halfExtents = (worldMax - worldMin) * 0.5f;
    }

private:
    void transform_noRotation(const glm::mat4& M, const glm::vec3& T, const glm::vec3 S) {
        worldMin.x = localMin.x * S.x + T.x;
        worldMin.y = localMin.y * S.y + T.y;
        worldMin.z = localMin.z * S.z + T.z;

        worldMax.x = localMax.x * S.x + T.x;
        worldMax.y = localMax.y * S.y + T.y;
        worldMax.z = localMax.z * S.z + T.z;

        Box.min.x.coord = worldMin.x; Box.max.x.coord = worldMax.x;
        Box.min.y.coord = worldMin.y; Box.max.y.coord = worldMax.y;
        Box.min.z.coord = worldMin.z; Box.max.z.coord = worldMax.z;
    }
    void transform_withRotation(const glm::mat3& M, const glm::vec3& T) {
        float  a, b;
        float  Amin[3], Amax[3];
        float  Bmin[3], Bmax[3];

        Amin[0] = localMin.x; Amax[0] = localMax.x;
        Amin[1] = localMin.y; Amax[1] = localMax.y;
        Amin[2] = localMin.z; Amax[2] = localMax.z;

        Bmin[0] = Bmax[0] = T.x;
        Bmin[1] = Bmax[1] = T.y;
        Bmin[2] = Bmax[2] = T.z;

        for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
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

        worldMin.x = Bmin[0]; worldMax.x = Bmax[0];
        worldMin.y = Bmin[1]; worldMax.y = Bmax[1];
        worldMin.z = Bmin[2]; worldMax.z = Bmax[2];

        Box.min.x.coord = worldMin.x; Box.max.x.coord = worldMax.x;
        Box.min.y.coord = worldMin.y; Box.max.y.coord = worldMax.y;
        Box.min.z.coord = worldMin.z; Box.max.z.coord = worldMax.z;
    }

    void computeFromVertices(const std::vector<glm::vec3>& vertices) {
        glm::vec3 mn(std::numeric_limits<float>::max());
        glm::vec3 mx(std::numeric_limits<float>::lowest());

        for (const auto& v : vertices) {
            mn = glm::min(mn, v);
            mx = glm::max(mx, v);
        }

        localMin = mn;
        localMax = mx;
    }

    void setLocalFaces() {
        localFaces.minX = {
            glm::vec3(localMin.x, localMin.y, localMin.z),
            glm::vec3(localMin.x, localMin.y, localMax.z),
            glm::vec3(localMin.x, localMax.y, localMax.z),
            glm::vec3(localMin.x, localMax.y, localMin.z)
        };

        localFaces.maxX = {
            glm::vec3(localMax.x, localMin.y, localMax.z),
            glm::vec3(localMax.x, localMin.y, localMin.z),
            glm::vec3(localMax.x, localMax.y, localMin.z),
            glm::vec3(localMax.x, localMax.y, localMax.z)
        };

        localFaces.minY = {
            glm::vec3(localMax.x, localMin.y, localMin.z),
            glm::vec3(localMax.x, localMin.y, localMax.z),
            glm::vec3(localMin.x, localMin.y, localMax.z),
            glm::vec3(localMin.x, localMin.y, localMin.z)
        };

        localFaces.maxY = {
            glm::vec3(localMin.x, localMax.y, localMin.z),
            glm::vec3(localMin.x, localMax.y, localMax.z),
            glm::vec3(localMax.x, localMax.y, localMax.z),
            glm::vec3(localMax.x, localMax.y, localMin.z)
        };

        localFaces.minZ = {
            glm::vec3(localMin.x, localMin.y, localMin.z),
            glm::vec3(localMin.x, localMax.y, localMin.z),
            glm::vec3(localMax.x, localMax.y, localMin.z),
            glm::vec3(localMax.x, localMin.y, localMin.z)
        };

        localFaces.maxZ = {
            glm::vec3(localMin.x, localMin.y, localMax.z),
            glm::vec3(localMax.x, localMin.y, localMax.z),
            glm::vec3(localMax.x, localMax.y, localMax.z),
            glm::vec3(localMin.x, localMax.y, localMax.z)
        };
    }
};