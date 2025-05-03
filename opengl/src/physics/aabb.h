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
        Min()                       // ← default-ctor
            : x(0, true, 0), y(0, true, 0), z(0, true, 0)
        {}
        Min(int id)                 // ← id-ctor kvar som förut
            : x(id, true, 0), y(id, true, 0), z(id, true, 0)
        {}
    } min;

    struct Max {
        Edge x, y, z;
        Max()                       // ← default-ctor
            : x(0, false, 0), y(0, false, 0), z(0, false, 0)
        {}
        Max(int id)
            : x(id, false, 0), y(id, false, 0), z(id, false, 0)
        {}
    } max;

    Box() : min(), max() {}      // ← anropar ovanstående
    Box(int id) : min(id), max(id) {}
};

struct localFaces {
    std::array<glm::vec3, 4> minX;
    std::array<glm::vec3, 4> maxX;
    std::array<glm::vec3, 4> minY;
    std::array<glm::vec3, 4> maxY;
    std::array<glm::vec3, 4> minZ;
    std::array<glm::vec3, 4> maxZ;
};

class AABB
{
public:
    Box Box;
    localFaces localFaces;

    // local
    glm::vec3 lMin; 
    glm::vec3 lMax;
    // world
    glm::vec3 wMin; 
    glm::vec3 wMax;

    glm::vec3 centroid;
    glm::vec3 halfExtents;

    AABB() = default;
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

        centroid = (wMin + wMax) * 0.5f;
        halfExtents = (wMax - wMin) * 0.5f;
    }

    inline bool intersects(const AABB& other) const {
        return
            (wMin.x <= other.wMax.x && wMax.x >= other.wMin.x) &&
            (wMin.y <= other.wMax.y && wMax.y >= other.wMin.y) &&
            (wMin.z <= other.wMax.z && wMax.z >= other.wMin.z);
    }

    inline bool contains(const AABB& other) const {
        return 
            wMin.x <= other.wMin.x && wMin.y <= other.wMin.y && wMin.z <= other.wMin.z && 
            wMax.x >= other.wMax.x && wMax.y >= other.wMax.y && wMax.z >= other.wMax.z;
    }

    inline void growToInclude(const glm::vec3& p) {
        wMin = glm::min(wMin, p);
        wMax = glm::max(wMax, p);
    }

    inline void grow(float m) {
        glm::vec3 delta{ m, m, m };
        wMin -= delta;
        wMax += delta;
    }

private:
    void transform_noRotation(const glm::mat4& M, const glm::vec3& T, const glm::vec3 S) {
        wMin.x = lMin.x * S.x + T.x;
        wMin.y = lMin.y * S.y + T.y;
        wMin.z = lMin.z * S.z + T.z;

        wMax.x = lMax.x * S.x + T.x;
        wMax.y = lMax.y * S.y + T.y;
        wMax.z = lMax.z * S.z + T.z;

        Box.min.x.coord = wMin.x; Box.max.x.coord = wMax.x;
        Box.min.y.coord = wMin.y; Box.max.y.coord = wMax.y;
        Box.min.z.coord = wMin.z; Box.max.z.coord = wMax.z;
    }
    void transform_withRotation(const glm::mat3& M, const glm::vec3& T) {
        float  a, b;
        float  Amin[3], Amax[3];
        float  Bmin[3], Bmax[3];

        Amin[0] = lMin.x; Amax[0] = lMax.x;
        Amin[1] = lMin.y; Amax[1] = lMax.y;
        Amin[2] = lMin.z; Amax[2] = lMax.z;

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

        wMin.x = Bmin[0]; wMax.x = Bmax[0];
        wMin.y = Bmin[1]; wMax.y = Bmax[1];
        wMin.z = Bmin[2]; wMax.z = Bmax[2];

        Box.min.x.coord = wMin.x; Box.max.x.coord = wMax.x;
        Box.min.y.coord = wMin.y; Box.max.y.coord = wMax.y;
        Box.min.z.coord = wMin.z; Box.max.z.coord = wMax.z;
    }

    void computeFromVertices(const std::vector<glm::vec3>& vertices) {
        glm::vec3 mn(std::numeric_limits<float>::max());
        glm::vec3 mx(std::numeric_limits<float>::lowest());

        for (const auto& v : vertices) {
            mn = glm::min(mn, v);
            mx = glm::max(mx, v);
        }

        lMin = mn;
        lMax = mx;
    }

    void setLocalFaces() {
        localFaces.minX = {
            glm::vec3(lMin.x, lMin.y, lMin.z),
            glm::vec3(lMin.x, lMin.y, lMax.z),
            glm::vec3(lMin.x, lMax.y, lMax.z),
            glm::vec3(lMin.x, lMax.y, lMin.z)
        };

        localFaces.maxX = {
            glm::vec3(lMax.x, lMin.y, lMax.z),
            glm::vec3(lMax.x, lMin.y, lMin.z),
            glm::vec3(lMax.x, lMax.y, lMin.z),
            glm::vec3(lMax.x, lMax.y, lMax.z)
        };

        localFaces.minY = {
            glm::vec3(lMax.x, lMin.y, lMin.z),
            glm::vec3(lMax.x, lMin.y, lMax.z),
            glm::vec3(lMin.x, lMin.y, lMax.z),
            glm::vec3(lMin.x, lMin.y, lMin.z)
        };

        localFaces.maxY = {
            glm::vec3(lMin.x, lMax.y, lMin.z),
            glm::vec3(lMin.x, lMax.y, lMax.z),
            glm::vec3(lMax.x, lMax.y, lMax.z),
            glm::vec3(lMax.x, lMax.y, lMin.z)
        };

        localFaces.minZ = {
            glm::vec3(lMin.x, lMin.y, lMin.z),
            glm::vec3(lMin.x, lMax.y, lMin.z),
            glm::vec3(lMax.x, lMax.y, lMin.z),
            glm::vec3(lMax.x, lMin.y, lMin.z)
        };

        localFaces.maxZ = {
            glm::vec3(lMin.x, lMin.y, lMax.z),
            glm::vec3(lMax.x, lMin.y, lMax.z),
            glm::vec3(lMax.x, lMax.y, lMax.z),
            glm::vec3(lMin.x, lMax.y, lMax.z)
        };
    }
};