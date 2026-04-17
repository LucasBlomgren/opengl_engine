#include "collider.h"

AABB& Collider::getAABB() {
	return aabb;
}

void Collider::updateAABB(const ColliderPose& pose) {
	if (type == ColliderType::CUBOID) {
		aabb.update(pose);
	}
	else if (type == ColliderType::SPHERE) {
		float radius = pose.scale.x; // assuming uniform scaling, can also store the radius in the Sphere struct if needed
		aabb.worldMin = pose.position - glm::vec3(radius);
		aabb.worldMax = pose.position + glm::vec3(radius);
		aabb.worldCenter = (aabb.worldMin + aabb.worldMax) * 0.5f;
		aabb.worldHalfExtents = (aabb.worldMax - aabb.worldMin) * 0.5f;
	}
}

void Collider::updateCollider(ColliderPose& pose) {
	std::visit([&](auto& shape) {
		using T = std::decay_t<decltype(shape)>;

		// OOBB
		if constexpr (std::is_same_v<T, OOBB>) {
			pose.ensureRotationMatrix();
			shape.update(pose);
		}	
		// Sphere
		else if constexpr (std::is_same_v<T, Sphere>) {
			shape.update(pose);
		}
		}, shape);
}