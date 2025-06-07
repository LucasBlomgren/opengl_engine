#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/io.hpp>

#include "game_object.h"

bool SATQuery(GameObject& objA, GameObject& objB, glm::vec3& normal, float& depth, int& collisionNormalOwner);

bool cuboidVsCuboid(GameObject& a, GameObject& b, glm::vec3& n, float& d, int& o);
bool sphereVsCuboid(GameObject& a, GameObject& b, glm::vec3& n, float& d, int& o);
bool sphereVsSphere(GameObject& a, GameObject& b, glm::vec3& n, float& d, int& o);
bool triVsCuboid(GameObject& a, GameObject& b, glm::vec3& n, float& d, int& o);
bool triVsSphere(GameObject& a, GameObject& b, glm::vec3& n, float& d, int& o);
bool triVsTri(GameObject& a, GameObject& b, glm::vec3& n, float& d, int& o);

std::pair<float, float> projectVertices(const std::array<glm::vec3, 8>& vertices, const glm::vec3& axis);
bool intersectPolygons(GameObject& objA, GameObject& objB, glm::vec3& normal, float& depth, int& collisionNormalOwner);
