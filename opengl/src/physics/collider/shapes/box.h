#pragma once

#include <array>
#include <vector>

#include <glm/vec3.hpp>

#include "physics/public/physics_types.h"

#include "physics/collider/collider_transform_cache.h"

namespace physics::internal {

enum class FaceId {
   MinX,
   MaxX,
   MinY,
   MaxY,
   MinZ,
   MaxZ
};

inline constexpr std::array<std::array<int, 4>, 6> FACE_INDICES = { {
    {{0, 4, 7, 3}}, // MinX
    {{5, 1, 2, 6}}, // MaxX
    {{1, 5, 4, 0}}, // MinY
    {{3, 7, 6, 2}}, // MaxY
    {{0, 3, 2, 1}}, // MinZ
    {{4, 5, 6, 7}}  // MaxZ
} };

inline const std::array<glm::vec3, 3> LOCAL_AXES = {
    glm::vec3(1.0f, 0.0f, 0.0f),
    glm::vec3(0.0f, 1.0f, 0.0f),
    glm::vec3(0.0f, 0.0f, 1.0f)
};

class Box {
public:
   // Physics-owned scale used to build the world-space box.
   glm::vec3 scale{ 1.0f };

   glm::vec3 worldCenter{ 0.0f };
   glm::vec3 localHalfExtents{ 0.5f };

   std::array<glm::vec3, 8> worldVertices{};
   std::array<glm::vec3, 3> worldAxes{
       glm::vec3(1.0f, 0.0f, 0.0f),
       glm::vec3(0.0f, 1.0f, 0.0f),
       glm::vec3(0.0f, 0.0f, 1.0f)
   };

   Box() = default;
   explicit Box(const std::vector<glm::vec3>& vertices);
   explicit Box(const glm::vec3& halfExtents, const glm::vec3& center = glm::vec3(0.0f));

   void init(const std::vector<glm::vec3>& vertices);
   void setLocalBounds(const glm::vec3& center, const glm::vec3& halfExtents);
   void update(const Pose& worldPose);

   std::array<glm::vec3, 4> getLocalFace(FaceId face) const;

private:
   glm::vec3 localCenter{ 0.0f };
   std::array<glm::vec3, 8> localVertices{};

   void rebuildLocalVertices();
};


struct BoxData {
   Box box;
   ColliderTransformCache transformCache;
};

}
