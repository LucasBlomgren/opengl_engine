#include "sat.h"

namespace physics::internal {

//=======================================================
//  Box-Box
//=======================================================
bool SAT::boxBox(Collider& colA, Collider& colB, Result& out)
{
    const OOBB& A = std::get<OOBB>(colA.shape);
    const OOBB& B = std::get<OOBB>(colB.shape);

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
                out.feature.type = type;
                out.feature.edgeIndexA = edgeA;
                out.feature.edgeIndexB = edgeB;
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
                    out.feature.type = AxisType::EdgeEdge;
                    out.feature.edgeIndexA = i;
                    out.feature.edgeIndexB = j;
                }
            }
        }
    }

    return true;
}

}
