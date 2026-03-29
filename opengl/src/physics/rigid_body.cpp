#include "pch.h"
#include "rigid_body.h"

void RigidBody::update(Transform& t, float dt) {
	t.position += linearVelocity * dt;
	updateOrientation(t.orientation, angularVelocity, dt);
}

void RigidBody::updateOrientation(glm::quat& orientation, const glm::vec3& angularVelocity, float dt) {
	glm::quat omegaQuat(0.0f, angularVelocity.x, angularVelocity.y, angularVelocity.z);
	orientation += 0.5f * dt * (omegaQuat * orientation);
	orientation = glm::normalize(orientation);
}

void RigidBody::updateInertiaWorld(Transform& t) {
	glm::mat3 R = glm::mat3_cast(t.orientation);
	invInertiaWorld = R * invInertiaLocal * glm::transpose(R);
}

void RigidBody::applyImpulseLinear(const glm::vec3& j) {
	linearVelocity += j;
}
void RigidBody::applyImpulseAngular(const glm::vec3& j) {
	angularVelocity += j;
}

void RigidBody::setAsleep(Transform& t) {
	if (type == BodyType::Static)
		return;

	linearVelocity = glm::vec3(0, 0, 0);
	angularVelocity = glm::vec3(0, 0, 0);

	asleep = true;
	sleepCounter = 0.0f;

	anchorTimer = 0.0f;
	anchorPoint = t.position;
}

void RigidBody::setAwake() {
	if (type == BodyType::Static)
		return;

	asleep = false;
	sleepCounter = 0.0f;
}

void RigidBody::setStatic() {
	type = BodyType::Static;
	mass = 0.0f;
	invMass = 0.0f;
}

void RigidBody::calculateInverseInertia(const ColliderType& colliderType, Transform& t) {
	if (type == BodyType::Static) {
		invInertiaLocal = glm::mat3(0.0f);
		return;
	}

	if (colliderType == ColliderType::CUBOID) {
		bool isUniform = approxEqual(t.scale.x, t.scale.y) and approxEqual(t.scale.y, t.scale.z);

		if (isUniform) {
			inertiaCube(t.scale.x);
		} else {
			inertiaCuboid(t.scale);
		}
	}
	else if (colliderType == ColliderType::SPHERE) {
		inertiaSphere(t.scale);
	}

	invInertiaWorld = invInertiaLocal;
	updateInertiaWorld(t);
}

void RigidBody::inertiaCube(float side) {
	float I = 6.0f / (mass * side * side);

	invInertiaLocal = glm::mat3(
		glm::vec3(I, 0.0f, 0.0f),
		glm::vec3(0.0f, I, 0.0f),
		glm::vec3(0.0f, 0.0f, I)
	);
}

void RigidBody::inertiaCuboid(const glm::vec3& s) {
	float I_x = (1.0f / 12.0f) * mass * (s.y * s.y + s.z * s.z);
	float I_y = (1.0f / 12.0f) * mass * (s.x * s.x + s.z * s.z);
	float I_z = (1.0f / 12.0f) * mass * (s.x * s.x + s.y * s.y);

	invInertiaLocal = glm::mat3(
		glm::vec3(1.0f / I_x, 0.0f, 0.0f),
		glm::vec3(0.0f, 1.0f / I_y, 0.0f),
		glm::vec3(0.0f, 0.0f, 1.0f / I_z));
}

void RigidBody::inertiaSphere(const glm::vec3& s) {
	float radius = s.x * 0.5f; // assuming uniform scaling
	float I = (2.0f / 5.0f) * (mass * radius * radius);
	float invI = 1.0f / I;

	invInertiaLocal = glm::mat3(
		glm::vec3(invI, 0.0f, 0.0f),
		glm::vec3(0.0f, invI, 0.0f),
		glm::vec3(0.0f, 0.0f, invI));
}

