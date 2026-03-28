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
    OOBB& boxA = std::get<OOBB>(A.shape);
    OOBB& boxB = std::get<OOBB>(B.shape);

    return intersectPolygons(boxA.wVertices, boxB.wVertices, boxA.wAxes, boxB.wAxes, out);

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

bool SAT::boxSphere(Collider& A, Collider& B, const Transform& transformA, Result& out) {
    // Plocka alltid ut cuboid i A och sphere i B
    OOBB*   box  = std::get_if<OOBB>(&A.shape);
    Sphere* sph  = std::get_if<Sphere>(&B.shape);

    // Matriser från GameObject (samma som du använder för rendering och OOBB/AABB)
    const glm::mat4& M  = transformA.modelMatrix;
    const glm::mat4& iM = transformA.invModelMatrix;

    // Sphere center i värld
    glm::vec3 worldC = sph->wCenter;
    // Transformera world → box-local
    glm::vec3 localC = glm::vec3(iM * glm::vec4(worldC, 1.0f));

    // Clamp i lokal AABB
    glm::vec3 clamped;
    clamped.x = glm::clamp(localC.x, -box->lHalfExtents.x, +box->lHalfExtents.x);
    clamped.y = glm::clamp(localC.y, -box->lHalfExtents.y, +box->lHalfExtents.y);
    clamped.z = glm::clamp(localC.z, -box->lHalfExtents.z, +box->lHalfExtents.z);

    // Transformera local → world
    glm::vec3 closestWorld = glm::vec3(M * glm::vec4(clamped, 1.0f));

    // Beräkna delta och dist2
    glm::vec3 delta = worldC - closestWorld;
    float dist2     = glm::dot(delta, delta);
    float r2        = sph->radius * sph->radius;

    // Test mot radie
    if (dist2 > r2) {
        return false;
    }

    // Fyll ut resultat, logga penetration & normal
    float dist = std::sqrt(dist2);
    out.depth  = sph->radius - dist;
    out.normal = (dist > 1e-6f ? delta / dist : glm::vec3(0.0f,1.0f,0.0f));
    out.point = closestWorld;

    return true;
}

bool SAT::sphereSphere(Collider& A, Collider& B, Result& out) {
    Sphere& sphereA = std::get<Sphere>(A.shape);
    Sphere& sphereB = std::get<Sphere>(B.shape);

    glm::vec3 delta = sphereB.wCenter - sphereA.wCenter; 
    float dist2 = glm::dot(delta, delta); 
    float r2 = (sphereA.radius + sphereB.radius) * (sphereA.radius + sphereB.radius); 

    if (dist2 > r2) {
        return false; // No intersection
    }

    float dist = std::sqrt(dist2);
    out.depth = (sphereA.radius + sphereB.radius) - dist;
    out.normal = (dist > 1e-6f ? delta / dist : glm::vec3(0.0f, 1.0f, 0.0f));
    out.point = sphereB.wCenter - out.normal * sphereB.radius; // Contact point on sphereB

    return true;
}

bool SAT::boxTri(Collider& A, Tri& tri, Result& out) {
    OOBB& box = std::get<OOBB>(A.shape);
    out.tri_ptr = &tri;

    return intersectPolygons(box.wVertices, tri.vertices, box.wAxes, tri.axes, out);
}

bool SAT::sphereTri(Collider& A, Tri& tri, Result& out) {
    Sphere& sphere = std::get<Sphere>(A.shape);
    glm::vec3 P = sphere.wCenter - tri.normal * glm::dot(sphere.wCenter - tri.vertices[0], tri.normal); 

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

    float depth = sphere.radius - glm::length(sphere.wCenter - P);

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

        float t = glm::dot(sphere.wCenter - tri.vertices[i], edge) / glm::dot(edge, edge); 
        t = glm::clamp(t, 0.0f, 1.0f);
        glm::vec3 Q = tri.vertices[i] + t * edge; 

        float dist2 = glm::dot(sphere.wCenter - Q, sphere.wCenter - Q);

        if (dist2 < bestDist2) { 
            bestDist2 = dist2;
            bestQ = Q; 
        }
    }

    if (bestDist2 <= sphere.radius * sphere.radius) { 
        out.point = bestQ; 
        out.normal = glm::normalize(sphere.wCenter - bestQ); 
        out.depth = sphere.radius - std::sqrt(bestDist2); 
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

    for (auto& n : normalsA) triedAxes.push_back(glm::normalize(n)); 
    for (auto& n : normalsB) triedAxes.push_back(glm::normalize(n)); 

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