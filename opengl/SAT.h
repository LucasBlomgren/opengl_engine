#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/io.hpp>

#include "Mesh.h"

std::pair<float, float> ProjectVertices(const std::array<glm::vec3, 8>& vertices, const glm::vec3& axis);
bool IntersectPolygons(Mesh& objA, Mesh& objB, glm::vec3& normal, float& depth, int& collisionNormalOwner);
