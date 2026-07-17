#include "sat.h"

//=======================================================
//  Box-Triangle
//=======================================================
bool SAT::boxTri(Collider& A, Tri& tri, Result& out) {
    const OOBB& box = std::get<OOBB>(A.shape);
    out = {};
    out.depth = FLT_MAX;
    out.tri_ptr = &tri;

    auto overlapIntervals = [](
        float minA, 
        float maxA, 
        float minB, 
        float maxB, 
        float& outOverlap) -> bool
        {
            if (maxA < minB || maxB < minA)
                return false;
            outOverlap = std::min(maxA, maxB) - std::max(minA, minB);
            return true;
        };

    auto projectTriOntoAxisLocal = [](
        const glm::vec3& p0, 
        const glm::vec3& p1, 
        const glm::vec3& p2, 
        const glm::vec3& axisLocal, 
        float& outMin, 
        float& outMax) -> void
        {
            float d0 = glm::dot(p0, axisLocal);
            float d1 = glm::dot(p1, axisLocal);
            float d2 = glm::dot(p2, axisLocal);
            outMin = std::min(d0, std::min(d1, d2));
            outMax = std::max(d0, std::max(d1, d2));
        };

    // Boxens world-space bas
    const glm::vec3 Au[3] = {
        box.worldAxes[0],
        box.worldAxes[1],
        box.worldAxes[2]
    };

    const glm::vec3 c = box.worldCenter;
    const glm::vec3 e = glm::abs(box.localHalfExtents * box.scale);

    // Transformera triangeln till boxens lokala rum
    const glm::vec3 w0 = tri.vertices[0] - c;
    const glm::vec3 w1 = tri.vertices[1] - c;
    const glm::vec3 w2 = tri.vertices[2] - c;

    glm::vec3 p0(
        glm::dot(w0, Au[0]),
        glm::dot(w0, Au[1]),
        glm::dot(w0, Au[2])
    );
    glm::vec3 p1(
        glm::dot(w1, Au[0]),
        glm::dot(w1, Au[1]),
        glm::dot(w1, Au[2])
    );
    glm::vec3 p2(
        glm::dot(w2, Au[0]),
        glm::dot(w2, Au[1]),
        glm::dot(w2, Au[2])
    );

    const glm::vec3 f0 = p1 - p0;
    const glm::vec3 f1 = p2 - p1;
    const glm::vec3 f2 = p0 - p2;

    const glm::vec3 triCenterLocal = (p0 + p1 + p2) / 3.0f;

    float bestDepth = FLT_MAX;
    glm::vec3 bestAxisLocal(0.0f);
    AxisType bestAxisType = AxisType::FaceA;
    int bestEdgeA = -1;
    int bestEdgeB = -1;

    auto testAxis = [&](const glm::vec3& axisLocal, AxisType axisType, int edgeA = -1, int edgeB = -1) -> bool
        {
            float len2 = glm::dot(axisLocal, axisLocal);
            if (len2 < 1e-12f)
                return true; // degenererad axel, skip

            float invLen = 1.0f / std::sqrt(len2);
            glm::vec3 n = axisLocal * invLen;

            float triMin, triMax;
            projectTriOntoAxisLocal(p0, p1, p2, n, triMin, triMax);

            // AABB/OBB-projektion i boxens lokala rum
            float r =
                e.x * std::abs(n.x) +
                e.y * std::abs(n.y) +
                e.z * std::abs(n.z);

            float overlap = 0.0f;
            if (!overlapIntervals(triMin, triMax, -r, r, overlap))
                return false;

            // Gör normalen konsekvent: från box -> tri
            if (glm::dot(triCenterLocal, n) < 0.0f)
                n = -n;

            if (overlap < bestDepth) {
                bestDepth = overlap;
                bestAxisLocal = n;
                bestAxisType = axisType;
                bestEdgeA = edgeA;
                bestEdgeB = edgeB;
            }

            return true;
        };

    // 1) Boxens tre face-axlar
    if (!testAxis(glm::vec3(1, 0, 0), AxisType::FaceA)) return false;
    if (!testAxis(glm::vec3(0, 1, 0), AxisType::FaceA)) return false;
    if (!testAxis(glm::vec3(0, 0, 1), AxisType::FaceA)) return false;

    // 2) Triangelns normal
    glm::vec3 triNormalLocal = glm::cross(p1 - p0, p2 - p0);
    if (!testAxis(triNormalLocal, AxisType::FaceB)) return false;

    // 3) 9 edge-edge axlar - I boxens lokala rum är boxaxlarna bara x/y/z
    const glm::vec3 ex(1, 0, 0);
    const glm::vec3 ey(0, 1, 0);
    const glm::vec3 ez(0, 0, 1);

    if (!testAxis(glm::cross(f0, ex), AxisType::EdgeEdge, 0, 0)) return false;
    if (!testAxis(glm::cross(f0, ey), AxisType::EdgeEdge, 0, 1)) return false;
    if (!testAxis(glm::cross(f0, ez), AxisType::EdgeEdge, 0, 2)) return false;

    if (!testAxis(glm::cross(f1, ex), AxisType::EdgeEdge, 1, 0)) return false;
    if (!testAxis(glm::cross(f1, ey), AxisType::EdgeEdge, 1, 1)) return false;
    if (!testAxis(glm::cross(f1, ez), AxisType::EdgeEdge, 1, 2)) return false;

    if (!testAxis(glm::cross(f2, ex), AxisType::EdgeEdge, 2, 0)) return false;
    if (!testAxis(glm::cross(f2, ey), AxisType::EdgeEdge, 2, 1)) return false;
    if (!testAxis(glm::cross(f2, ez), AxisType::EdgeEdge, 2, 2)) return false;

    // Kolliderar
    out.depth = bestDepth;
    out.feature.type = bestAxisType;
    out.feature.edgeIndexA = bestEdgeA;
    out.feature.edgeIndexB = bestEdgeB;


    // Local axis -> world axis
    out.normal =
        Au[0] * bestAxisLocal.x +
        Au[1] * bestAxisLocal.y +
        Au[2] * bestAxisLocal.z;

    out.normal = glm::normalize(out.normal);

    // Approx kontaktpunkt: clampad triangelcentroid i boxens lokala rum tillbaka till world
    glm::vec3 localPoint = glm::clamp(triCenterLocal, -e, e);
    out.point =
        c +
        Au[0] * localPoint.x +
        Au[1] * localPoint.y +
        Au[2] * localPoint.z;

    return true;
}