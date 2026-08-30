#include <glm/vec3.hpp>
#include <limits>

namespace physics::internal {

class Tri;

namespace SAT {
    enum class AxisType {
        None,
        FaceA,
        FaceB,
        EdgeEdge,
        TriFace,
        SpherePoint
    };

    enum class HitType {
        Overlap,        // regular penetration
        Speculative     // swept hit
    };

    struct FeatureId {
        AxisType type = AxisType::None;

        int faceIndex = -1;
        int edgeIndexA = -1;
        int edgeIndexB = -1;
        int vertexIndex = -1;
    };

    struct Result {
        HitType hitType = HitType::Overlap;

        // Always from A to B after reverseNormal/canonicalization.
        glm::vec3 normal{ 0.0f };

        // Positive only for real penetration.
        float depth = 0.0f;

        // Positive only for speculative/non-overlapping contacts.
        float separation = 0.0f;

        // Time of impact within current dt.
        // Normal overlap contacts can keep this at 0.
        float toi = 0.0f;

        // Useful for sphere-box, sphere-tri, speculative hit point, etc.
        glm::vec3 point{ 0.0f };

        // Feature information for manifold generation.
        FeatureId feature{};

        // Terrain-specific. Could later become a more generic mesh feature pointer/id.
        Tri* tri_ptr = nullptr;

        // Optional debug / SAT internals.
        float separationA = -std::numeric_limits<float>::infinity();
        float separationB = -std::numeric_limits<float>::infinity(); 
    };
}

}
