#include "collider.h"
#include "game_object.h"

AABB& Collider::getAABB() {
	return aabb;
}

void Collider::updateAABB(const Transform& t) {
	if (type == ColliderType::CUBOID) {
		aabb.update(t);
	}
	else if (type == ColliderType::SPHERE) {
		float radius = t.scale.x; // assuming uniform scaling, can also store the radius in the Sphere struct if needed
		aabb.wMin = t.position - glm::vec3(radius);
		aabb.wMax = t.position + glm::vec3(radius);
		aabb.centroid = (aabb.wMin + aabb.wMax) * 0.5f;
		aabb.halfExtents = (aabb.wMax - aabb.wMin) * 0.5f;
	}
}

void Collider::updateCollider(const Transform& t) {
	std::visit([&](auto& shape) {
		using T = std::decay_t<decltype(shape)>;

		// OOBB
		if constexpr (std::is_same_v<T, OOBB>) {
			shape.update(t);
		}
		// Sphere
		else if constexpr (std::is_same_v<T, Sphere>) {
			shape.update(t);
		}
		}, shape);
}