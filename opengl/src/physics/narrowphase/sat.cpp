#include "pch.h"
#include "SAT.h"

void SAT::reverseNormal(glm::vec3& posA, glm::vec3& posB, glm::vec3& normal) {
    glm::vec3 direction = posB - posA; 
    if (glm::dot(direction, normal) < 0) { 
        normal = -normal; 
    }
}

void SAT::findBestTriangles(std::vector<SAT::Result>& results) { 
    if (results.size() <= 4)
        return; 

    static std::vector<int> indices; 
    indices.clear();
    indices.reserve(4); 
    indices.push_back(0);

    // Sort results by depth
    std::sort(results.begin(), results.end(), [](const Result a, const Result b) { 
        return a.depth > b.depth;
        });  

    int loopRange = std::min(3, static_cast<int>(results.size()));
    for (int i = 0; i < loopRange; i++) {
        addFurthestTriangle(results, indices);
    }

    static std::vector<SAT::Result> temp;
    temp.clear();
    temp.reserve(indices.size());
    for (int idx : indices) {
        temp.push_back(results[idx]);
    } 

    results.swap(temp);
}

void SAT::addFurthestTriangle(std::vector<SAT::Result>& result, std::vector<int>& indices) {
    int bestIdx = -1;
    float bestDist = std::numeric_limits<float>::lowest();

    for (int i = 0; i < result.size(); i++)  
    { 
        // Skip if already added
        bool alreadyAdded = false;
        for (int j = 0; j < indices.size(); j++) { 
            if (i == indices[j]) {
                alreadyAdded = true;
                break;
            }
        }
        if (alreadyAdded) 
            continue; 

        float shortestDist = std::numeric_limits<float>::max();
        for (int j = 0; j < indices.size(); j++)
        { 
            glm::vec3 diff = result[i].tri_ptr->centroid - result[indices[j]].tri_ptr->centroid;
            float dist2 = glm::dot(diff, diff);

            if (dist2 < shortestDist) {
                shortestDist = dist2;
            } 
        }
        if (shortestDist > bestDist) {
            bestDist = shortestDist; 
            bestIdx = i; 
        }
    }

    indices.push_back(bestIdx);
}

bool SAT::boxBox(Collider& A, Collider& B, Result& out) {
    //OOBB& boxA = std::get<OOBB>(A.shape);
    //OOBB& boxB = std::get<OOBB>(B.shape);

    //return intersectPolygons(boxA.worldVertices, boxB.worldVertices, boxA.worldAxes, boxB.worldAxes, out);

    //return boxBoxOBB_NoPrecompute(A, B, out);
    return boxBoxOBB(A, B, out);

    // #TODO: Korrekt hantering av face vs edge-edge för BoxBox
    // 
    // ---- Välja face-face:
    // 
    // float s = dot(A.axis[k], normal);
    // int faceSign = (s >= 0.0f) ? +1 : -1;   // +1 = plus-face, -1 = minus-face
    // vec3 faceCenter = A.center + faceSign * A.axis[k] * A.halfExtents[k];
    // vec3 faceNormal = faceSign * A.axis[k];
    // 
    // Sen använda faceCenter och faceNormal för att välja rätt face för clipping
    //
    // ---- Välja edge-edge:
    // 
    //// inputs: center C, axes U[3], half extents he[3], edge axis i, contact normal n (A->B)
    //int a0 = (i + 1) % 3;
    //int a1 = (i + 2) % 3;

    //float s0 = (dot(n, U[a0]) >= 0.0f) ? 1.0f : -1.0f;
    //float s1 = (dot(n, U[a1]) >= 0.0f) ? 1.0f : -1.0f;

    //// base point on the "side" of the box (fixed a0/a1 signs)
    //glm::vec3 base = C
    //    + s0 * U[a0] * he[a0]
    //    + s1 * U[a1] * he[a1];

    //// edge endpoints (vary along i)
    //glm::vec3 e0 = base + U[i] * he[i];
    //glm::vec3 e1 = base - U[i] * he[i];
}

bool SAT::boxSphere(Collider& A, Collider& B, const ColliderPose& pose, Result& out) {
    // Plocka alltid ut cuboid i A och sphere i B
    OOBB*   box  = std::get_if<OOBB>(&A.shape);
    Sphere* sph  = std::get_if<Sphere>(&B.shape);

    // Matriser från GameObject (samma som du använder för rendering och OOBB/AABB)
    const glm::mat4& M  = pose.modelMatrix;
    const glm::mat4& iM = pose.invModelMatrix;

    // Sphere center i värld
    glm::vec3 worldC = sph->centerWorld;
    // Transformera world → box-local
    glm::vec3 localC = glm::vec3(iM * glm::vec4(worldC, 1.0f));

    // Clamp i lokal AABB
    glm::vec3 clamped;
    clamped.x = glm::clamp(localC.x, -box->localHalfExtents.x, +box->localHalfExtents.x);
    clamped.y = glm::clamp(localC.y, -box->localHalfExtents.y, +box->localHalfExtents.y);
    clamped.z = glm::clamp(localC.z, -box->localHalfExtents.z, +box->localHalfExtents.z);

    // Transformera local → world
    glm::vec3 closestWorld = glm::vec3(M * glm::vec4(clamped, 1.0f));

    // Beräkna delta och dist2
    glm::vec3 delta = worldC - closestWorld;
    float dist2     = glm::dot(delta, delta);
    float r2        = sph->radiusWorld * sph->radiusWorld;

    // Test mot radie
    if (dist2 > r2) {
        return false;
    }

    // Fyll ut resultat, logga penetration & normal
    float dist = std::sqrt(dist2);
    out.depth  = sph->radiusWorld - dist;
    out.normal = (dist > 1e-6f ? delta / dist : glm::vec3(0.0f,1.0f,0.0f));
    out.point = closestWorld;

    return true;
}

bool SAT::sphereSphere(Collider& A, Collider& B, Result& out) {
    Sphere& sphereA = std::get<Sphere>(A.shape);
    Sphere& sphereB = std::get<Sphere>(B.shape);

    glm::vec3 delta = sphereB.centerWorld - sphereA.centerWorld; 
    float dist2 = glm::dot(delta, delta); 
    float r2 = (sphereA.radiusWorld + sphereB.radiusWorld) * (sphereA.radiusWorld + sphereB.radiusWorld); 

    if (dist2 > r2) {
        return false; // No intersection
    }

    float dist = std::sqrt(dist2);
    out.depth = (sphereA.radiusWorld + sphereB.radiusWorld) - dist;
    out.normal = (dist > 1e-6f ? delta / dist : glm::vec3(0.0f, 1.0f, 0.0f));
    out.point = sphereB.centerWorld - out.normal * sphereB.radiusWorld; // Contact point on sphereB

    return true;
}

bool SAT::boxTri(Collider& A, Tri& tri, Result& out) {
    //OOBB& box = std::get<OOBB>(A.shape);
    //out.tri_ptr = &tri;

    //return intersectPolygons(box.worldVertices, tri.vertices, box.worldAxes, tri.axes, out);

    return boxTriSpecialized(A, tri, out);
}

bool SAT::boxTriSpecialized(Collider& A, Tri& tri, Result& out) {
    const OOBB& box = std::get<OOBB>(A.shape);
    out = {};
    out.depth = FLT_MAX;
    out.tri_ptr = &tri;

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

    // ---------------------------------------------------
    // 1) Boxens tre face-axlar
    // ---------------------------------------------------
    if (!testAxis(glm::vec3(1, 0, 0), AxisType::FaceA)) return false;
    if (!testAxis(glm::vec3(0, 1, 0), AxisType::FaceA)) return false;
    if (!testAxis(glm::vec3(0, 0, 1), AxisType::FaceA)) return false;

    // ---------------------------------------------------
    // 2) Triangelns normal
    // ---------------------------------------------------
    glm::vec3 triNormalLocal = glm::cross(p1 - p0, p2 - p0);
    if (!testAxis(triNormalLocal, AxisType::FaceB)) return false;

    // ---------------------------------------------------
    // 3) 9 edge-edge axlar
    //    I boxens lokala rum är boxaxlarna bara x/y/z
    // ---------------------------------------------------
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

    // ---------------------------------------------------
    // Kolliderar
    // ---------------------------------------------------
    out.depth = bestDepth;
    out.axisType = bestAxisType;
    out.edgeIndexA = bestEdgeA;
    out.edgeIndexB = bestEdgeB;

    // Local axis -> world axis
    out.normal =
        Au[0] * bestAxisLocal.x +
        Au[1] * bestAxisLocal.y +
        Au[2] * bestAxisLocal.z;

    out.normal = glm::normalize(out.normal);

    // Approx kontaktpunkt:
    // clampad triangelcentroid i boxens lokala rum tillbaka till world
    glm::vec3 localPoint = glm::clamp(triCenterLocal, -e, e);
    out.point =
        c +
        Au[0] * localPoint.x +
        Au[1] * localPoint.y +
        Au[2] * localPoint.z;

    return true;
}

bool SAT::sphereTri(Collider& A, Tri& tri, Result& out) {
    Sphere& sphere = std::get<Sphere>(A.shape);
    glm::vec3 P = sphere.centerWorld - tri.normal * glm::dot(sphere.centerWorld - tri.vertices[0], tri.normal); 

    // test vs edge planes
    bool isInside = true;
    for (int i = 0; i < 3; i++) {
        glm::vec3 planeNormal = (glm::normalize(glm::cross(tri.edgeDirs[i], tri.normal)));
        glm::vec3 planePoint = tri.vertices[i];

        if (glm::dot(planeNormal, P - planePoint) > 0.0f) {
            isInside = false;
            break;
        }
    }

    float depth = sphere.radiusWorld - glm::length(sphere.centerWorld - P);

    if (isInside and depth > 0.0f) {
        out.point = P;
        out.normal = tri.normal;
        out.depth = depth;
        out.tri_ptr = &tri;
        return true;
    }

    // not inside planes, project sphere center onto triangle edges
    float bestDist2 = std::numeric_limits<float>::infinity();
    glm::vec3 bestQ;
    for (int i = 0; i < 3; i++) {
        glm::vec3& edge = tri.edgeDirs[i];

        float t = glm::dot(sphere.centerWorld - tri.vertices[i], edge) / glm::dot(edge, edge); 
        t = glm::clamp(t, 0.0f, 1.0f);
        glm::vec3 Q = tri.vertices[i] + t * edge; 

        float dist2 = glm::dot(sphere.centerWorld - Q, sphere.centerWorld - Q);

        if (dist2 < bestDist2) { 
            bestDist2 = dist2;
            bestQ = Q; 
        }
    }

    if (bestDist2 <= sphere.radiusWorld * sphere.radiusWorld) { 
        out.point = bestQ; 
        out.normal = glm::normalize(sphere.centerWorld - bestQ); 
        out.depth = sphere.radiusWorld - std::sqrt(bestDist2); 
        out.tri_ptr = &tri;
        return true; 
    }

    return false;
}

std::pair<float, float> SAT::projectVertices(std::span<const glm::vec3> vertices, const glm::vec3& axis) {
    float min = std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::lowest();

    for (int i = 0; i < vertices.size(); i++)
    {
        const glm::vec3& v = vertices[i];
        float proj = glm::dot(v, axis);

        if (proj < min) { min = proj; }
        if (proj > max) { max = proj; }
    }

    return std::make_pair(min, max);
}

bool SAT::intersectPolygons(
    std::span<const glm::vec3> vertsA,
    std::span<const glm::vec3> vertsB,
    std::span<const glm::vec3> normalsA,
    std::span<const glm::vec3> normalsB,
    Result& satResult)
{
    static std::vector<glm::vec3> triedAxes; 
    triedAxes.clear(); 
    triedAxes.reserve(normalsA.size() + normalsB.size()); 

    // testa normaler från A
    for (const glm::vec3& A : normalsA) {
        auto [minA, maxA] = projectVertices(vertsA, A);
        auto [minB, maxB] = projectVertices(vertsB, A);

        if (minA >= maxB or minB >= maxA)
            return false;

        float overlapPlus = maxB - minA; 
        float overlapMinus = maxA - minB; 
        float axisDepth = std::min(overlapPlus, overlapMinus); 

        if (axisDepth < satResult.depth) {
            satResult.depth = axisDepth;
            satResult.normal = A;
            satResult.axisType = AxisType::FaceA;
        }

        float sepA_plus = minB - maxA;
        float sepA_minus = minA - maxB;
        satResult.separationA = std::max(satResult.separationA, std::max(sepA_plus, sepA_minus)); 
    }

    // testa normaler från B
    for (const glm::vec3& B : normalsB) {
        auto [minA, maxA] = projectVertices(vertsA, B);
        auto [minB, maxB] = projectVertices(vertsB, B);

        if (minA >= maxB or minB >= maxA)
            return false;

        float overlapPlus = maxB - minA; 
        float overlapMinus = maxA - minB; 
        float axisDepth = std::min(overlapPlus, overlapMinus); 

        if (axisDepth < satResult.depth) {
            satResult.depth = axisDepth;
            satResult.normal = B;
            satResult.axisType = AxisType::FaceB;
        }

        float sepB_plus = minA - maxB; 
        float sepB_minus = minB - maxA;
        satResult.separationB = std::max(satResult.separationB, std::max(sepB_plus, sepB_minus)); 
    }

    for (const glm::vec3& n : normalsA) triedAxes.push_back(n);
    for (const glm::vec3& n : normalsB) triedAxes.push_back(n);

    // testa korsprodukter mellan normaler
    for (int i = 0; i < normalsA.size(); i++) {
        for (int j = 0; j < normalsB.size(); j++) {
            const glm::vec3& A = normalsA[i]; 
            const glm::vec3& B = normalsB[j]; 

            glm::vec3 axis = glm::cross(A, B);

            if (glm::length2(axis) < 1e-6f)
                continue; // Skip parallel axes

            axis = glm::normalize(axis);

            bool dup = false; 
            for (auto& t : triedAxes) { 
                if (glm::abs(glm::dot(axis, t)) > 0.999f) { dup = true; break; }
            }
            if (dup) continue; 

            triedAxes.push_back(axis); 
            auto [minA, maxA] = projectVertices(vertsA, axis);
            auto [minB, maxB] = projectVertices(vertsB, axis);

            if (minA >= maxB or minB >= maxA)
                return false;

            float axisDepth = std::min(maxB - minA, maxA - minB);

            if (axisDepth < satResult.depth) {
                satResult.depth = axisDepth;
                satResult.normal = axis;
                satResult.axisType = AxisType::EdgeEdge;
                satResult.edgeIndexA = i; 
                satResult.edgeIndexB = j; 
            }
        }
    }

    return true;
}

bool SAT::boxBoxOBB_NoPrecompute(Collider& ACol, Collider& BCol, Result& out)
{
    const OOBB& A = std::get<OOBB>(ACol.shape);
    const OOBB& B = std::get<OOBB>(BCol.shape);

    const glm::vec3 Au[3] = { A.worldAxes[0], A.worldAxes[1], A.worldAxes[2] };
    const glm::vec3 Bu[3] = { B.worldAxes[0], B.worldAxes[1], B.worldAxes[2] };

    const glm::vec3 a = A.localHalfExtents * A.scale;
    const glm::vec3 b = B.localHalfExtents * B.scale;

    const glm::vec3 cA = A.worldCenter;
    const glm::vec3 cB = B.worldCenter;
    const glm::vec3 tWorld = cB - cA;

    constexpr float EPS = 1e-6f;

    out = {};
    out.depth = FLT_MAX;

    auto updateBest = [&](float overlap,
        const glm::vec3& axisWorld,
        float signedDistance,
        AxisType type,
        int edgeA = -1,
        int edgeB = -1)
        {
            if (overlap < out.depth) {
                out.depth = overlap;
                out.normal = (signedDistance < 0.0f) ? -axisWorld : axisWorld;
                out.axisType = type;
                out.edgeIndexA = edgeA;
                out.edgeIndexB = edgeB;
            }
        };

    // ---------------------------------------
    // 1) Face normals från A
    // ---------------------------------------
    for (int i = 0; i < 3; ++i) {
        glm::vec3 L = Au[i];

        float dist = glm::dot(tWorld, L);

        float ra = a[i];

        float rb =
            b[0] * (std::abs(glm::dot(Bu[0], L)) + EPS) +
            b[1] * (std::abs(glm::dot(Bu[1], L)) + EPS) +
            b[2] * (std::abs(glm::dot(Bu[2], L)) + EPS);

        float overlap = (ra + rb) - std::abs(dist);
        if (overlap < 0.0f) {
            return false;
        }

        updateBest(overlap, L, dist, AxisType::FaceA);
    }

    // ---------------------------------------
    // 2) Face normals från B
    // ---------------------------------------
    for (int j = 0; j < 3; ++j) {
        glm::vec3 L = Bu[j];

        float dist = glm::dot(tWorld, L);

        float ra =
            a[0] * (std::abs(glm::dot(Au[0], L)) + EPS) +
            a[1] * (std::abs(glm::dot(Au[1], L)) + EPS) +
            a[2] * (std::abs(glm::dot(Au[2], L)) + EPS);

        float rb = b[j];

        float overlap = (ra + rb) - std::abs(dist);
        if (overlap < 0.0f) {
            return false;
        }

        updateBest(overlap, L, dist, AxisType::FaceB);
    }

    // ---------------------------------------
    // 3) Edge-edge axlar: cross(A_i, B_j)
    // ---------------------------------------
    for (int i = 0; i < 3; ++i) {
        int i1 = (i + 1) % 3;
        int i2 = (i + 2) % 3;

        for (int j = 0; j < 3; ++j) {
            int j1 = (j + 1) % 3;
            int j2 = (j + 2) % 3;

            glm::vec3 axis = glm::cross(Au[i], Bu[j]);
            float axisLen2 = glm::dot(axis, axis);

            // parallella axlar => redundant edge-edge-axel
            if (axisLen2 < 1e-12f) {
                continue;
            }

            // Här använder vi samma standardformler som i precompute-versionen,
            // men räknar alla dot products direkt när de behövs.

            float R_i1_j = glm::dot(Au[i1], Bu[j]);
            float R_i2_j = glm::dot(Au[i2], Bu[j]);
            float R_i_j1 = glm::dot(Au[i], Bu[j1]);
            float R_i_j2 = glm::dot(Au[i], Bu[j2]);

            float t_i1 = glm::dot(tWorld, Au[i1]);
            float t_i2 = glm::dot(tWorld, Au[i2]);

            float ra =
                a[i1] * (std::abs(R_i2_j) + EPS) +
                a[i2] * (std::abs(R_i1_j) + EPS);

            float rb =
                b[j1] * (std::abs(R_i_j2) + EPS) +
                b[j2] * (std::abs(R_i_j1) + EPS);

            float dist = std::abs(t_i2 * R_i1_j - t_i1 * R_i2_j);

            float overlap = (ra + rb) - dist;
            if (overlap < 0.0f) {
                return false;
            }

            float axisLen = std::sqrt(axisLen2);
            glm::vec3 n = axis / axisLen;

            float overlapNormalized = overlap / axisLen;
            float signedDistance = glm::dot(tWorld, n);

            if (overlapNormalized < out.depth) {
                out.depth = overlapNormalized;
                out.normal = (signedDistance < 0.0f) ? -n : n;
                out.axisType = AxisType::EdgeEdge;
                out.edgeIndexA = i;
                out.edgeIndexB = j;
            }
        }
    }

    return true;
}

enum class AxisType {
    FaceA,
    FaceB,
    EdgeEdge
};

struct Result {
    float depth = FLT_MAX;
    glm::vec3 normal{ 0.0f };
    AxisType axisType = AxisType::FaceA;
    int edgeIndexA = -1;
    int edgeIndexB = -1;
};

bool SAT::boxBoxOBB(Collider& ACol, Collider& BCol, Result& out)
{
    const OOBB& A = std::get<OOBB>(ACol.shape);
    const OOBB& B = std::get<OOBB>(BCol.shape);

    // Boxarnas world-axlar (ska vara normaliserade)
    const glm::vec3 Au[3] = { A.worldAxes[0], A.worldAxes[1], A.worldAxes[2] };
    const glm::vec3 Bu[3] = { B.worldAxes[0], B.worldAxes[1], B.worldAxes[2] };

    // Half extents i world-scale men fortfarande längs boxarnas lokala/worldAxes-riktningar
    const glm::vec3 a = A.localHalfExtents * A.scale;
    const glm::vec3 b = B.localHalfExtents * B.scale;

    const glm::vec3 cA = A.worldCenter;
    const glm::vec3 cB = B.worldCenter;

    out = {};
    out.depth = FLT_MAX;

    constexpr float EPS = 1e-6f;

    // ---------------------------------------------------
    // 1) Relative rotation matrix:
    //    R[i][j] = dot(A_i, B_j)
    // ---------------------------------------------------
    float R[3][3];
    float AbsR[3][3];

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] = glm::dot(Au[i], Bu[j]);
            AbsR[i][j] = std::abs(R[i][j]) + EPS;
        }
    }

    // ---------------------------------------------------
    // 2) Translation från A till B, uttryckt i A:s bas
    // ---------------------------------------------------
    glm::vec3 tWorld = cB - cA;
    float t[3] = {
        glm::dot(tWorld, Au[0]),
        glm::dot(tWorld, Au[1]),
        glm::dot(tWorld, Au[2])
    };

    auto updateBestFaceAxis = [&](float overlap,
        const glm::vec3& axisWorld,
        float signedDistance,
        AxisType type,
        int edgeA = -1,
        int edgeB = -1)
        {
            if (overlap < out.depth) {
                out.depth = overlap;
                out.normal = (signedDistance < 0.0f) ? -axisWorld : axisWorld;
                out.axisType = type;
                out.edgeIndexA = edgeA;
                out.edgeIndexB = edgeB;
            }
        };

    auto testFaceAxis = [&](float dist, float ra, float rb,
        const glm::vec3& axisWorld,
        float signedDistance,
        AxisType type,
        int edgeA = -1,
        int edgeB = -1) -> bool
        {
            float overlap = (ra + rb) - std::abs(dist);
            if (overlap < 0.0f) {
                return false; // separating axis
            }

            updateBestFaceAxis(overlap, axisWorld, signedDistance, type, edgeA, edgeB);
            return true;
        };

    // ---------------------------------------------------
    // 3) Test A:s 3 face normals
    // ---------------------------------------------------
    for (int i = 0; i < 3; ++i) {
        float ra = a[i];
        float rb = b[0] * AbsR[i][0] + b[1] * AbsR[i][1] + b[2] * AbsR[i][2];
        float dist = t[i];

        if (!testFaceAxis(dist, ra, rb, Au[i], dist, AxisType::FaceA)) {
            return false;
        }
    }

    // ---------------------------------------------------
    // 4) Test B:s 3 face normals
    //    dist = t projicerat på B_j
    //         = dot(tWorld, B_j)
    //         = t[0]R[0][j] + t[1]R[1][j] + t[2]R[2][j]
    // ---------------------------------------------------
    for (int j = 0; j < 3; ++j) {
        float ra = a[0] * AbsR[0][j] + a[1] * AbsR[1][j] + a[2] * AbsR[2][j];
        float rb = b[j];
        float dist = t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j];

        if (!testFaceAxis(dist, ra, rb, Bu[j], dist, AxisType::FaceB)) {
            return false;
        }
    }

    // ---------------------------------------------------
    // 5) Test de 9 edge-edge-axlarna: A_i x B_j
    //
    // Obs:
    // Formlerna nedan använder OBB-vs-OBB SAT i standardform.
    // Själva separations-testet kan göras utan att explicit bygga axeln.
    //
    // Men om vi vill spara bästa normal/penetration för manifold,
    // behöver vi bygga axisWorld = cross(A_i, B_j).
    // ---------------------------------------------------
    for (int i = 0; i < 3; ++i) {
        int i1 = (i + 1) % 3;
        int i2 = (i + 2) % 3;

        for (int j = 0; j < 3; ++j) {
            int j1 = (j + 1) % 3;
            int j2 = (j + 2) % 3;

            float ra = a[i1] * AbsR[i2][j] + a[i2] * AbsR[i1][j];
            float rb = b[j1] * AbsR[i][j2] + b[j2] * AbsR[i][j1];

            float dist = std::abs(t[i2] * R[i1][j] - t[i1] * R[i2][j]);

            float overlap = (ra + rb) - dist;
            if (overlap < 0.0f) {
                return false; // separating axis
            }

            // För att spara normal och jämföra penetrationsdjup mellan olika edge-edge-axlar
            // behöver vi normalisera den geometriska axeln.
            glm::vec3 axisWorld = glm::cross(Au[i], Bu[j]);
            float axisLen2 = glm::dot(axisWorld, axisWorld);

            // Om axlarna är nästan parallella är denna cross-axis degenererad/redundant.
            // Då skippar vi att använda den som "best axis", men separations-testet ovan
            // har redan gjorts.
            if (axisLen2 > 1e-12f) {
                float axisLen = std::sqrt(axisLen2);
                glm::vec3 n = axisWorld / axisLen;

                // overlap från standardformeln är på en onormaliserad cross-axis,
                // så för att kunna jämföra depth rättvist med face-axlar delar vi med |axis|.
                float overlapNormalized = overlap / axisLen;

                float signedDistance = glm::dot(tWorld, n);

                if (overlapNormalized < out.depth) {
                    out.depth = overlapNormalized;
                    out.normal = (signedDistance < 0.0f) ? -n : n;
                    out.axisType = AxisType::EdgeEdge;
                    out.edgeIndexA = i;
                    out.edgeIndexB = j;
                }
            }
        }
    }

    return true;
}

static inline void SAT::projectTriOntoAxisLocal(
    const glm::vec3& p0,
    const glm::vec3& p1,
    const glm::vec3& p2,
    const glm::vec3& axis,
    float& outMin,
    float& outMax)
{
    float d0 = glm::dot(p0, axis);
    float d1 = glm::dot(p1, axis);
    float d2 = glm::dot(p2, axis);

    outMin = std::min(d0, std::min(d1, d2));
    outMax = std::max(d0, std::max(d1, d2));
}

static inline bool SAT::overlapIntervals(
    float minA, float maxA,
    float minB, float maxB,
    float& outOverlap)
{
    if (maxA < minB || maxB < minA)
        return false;

    outOverlap = std::min(maxA, maxB) - std::max(minA, minB);
    return true;
}