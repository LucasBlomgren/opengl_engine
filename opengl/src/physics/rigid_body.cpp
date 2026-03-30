#include "pch.h"
#include "rigid_body.h"

void RigidBody::update(Transform& t, ColliderType colliderType, float dt) {
	if (type == BodyType::Static || type == BodyType::Kinematic || asleep || externalControl)
		return;

	if (allowGravity and !player)
		linearVelocity += g * dt;

	float aLin;
	float aAng;
	bool avgCollisions = collisionHistory.average() > 0;

	if (colliderType == ColliderType::SPHERE) {
		aLin = 0.1f; // konstant linjär retardation (m/s^2)
		aAng = 1.0f; // konstant angulär retardation (rad/s^2)

		float vMag = glm::length(linearVelocity);
		if (vMag > 0.0f and avgCollisions) {
			float newMag = vMag - aLin * dt;
			if (newMag < 0.0f) newMag = 0.0f;
			linearVelocity *= (newMag / vMag); // behåll riktningen
		}

		float wMag = glm::length(angularVelocity);
		if (wMag > 0.0f and avgCollisions) {
			float newMag = wMag - aAng * dt;
			if (newMag < 0.0f) newMag = 0.0f;
			angularVelocity *= (newMag / wMag);
		}
	}

	// anti stuck for objects with high collision counts
	avgCollisions = collisionHistory.average() > 3;
	if (avgCollisions) {
		linearVelocity = linearVelocity * std::pow(0.98f, dt);
		angularVelocity = angularVelocity * std::pow(0.98f, dt);
	}

	// fake constraints
	if (!canMoveLinearly) {
		linearVelocity = glm::vec3(0.0f);
		angularVelocity.x = 0.0f;
		angularVelocity.y = 0.0f;
	}

	// update position and orientation
	t.lastPosition = t.position;
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
	if (type == BodyType::Static || type == BodyType::Kinematic)
		return;

	linearVelocity = glm::vec3(0, 0, 0);
	angularVelocity = glm::vec3(0, 0, 0);

	asleep = true;
	sleepCounter = 0.0f;

	anchorTimer = 0.0f;
	anchorPoint = t.position;
}

void RigidBody::setAwake() {
	if (type == BodyType::Static || type == BodyType::Kinematic)
		return;

	asleep = false;
	sleepCounter = 0.0f;
}

void RigidBody::setStatic() {
	type = BodyType::Static;
	mass = 0.0f;
	invMass = 0.0f;
}

void RigidBody::setExternalControl(bool controlled) {
	externalControl = controlled;
	sleepCounter = 0.0f;
	anchorTimer = 0.0f;
}

void RigidBody::calculateInverseInertia(const ColliderType& colliderType, const Collider& collider, Transform& t) {
	if (type == BodyType::Static) {
		invInertiaLocal = glm::mat3(0.0f);
		return;
	}

	if (colliderType == ColliderType::CUBOID) {
		const OOBB& box = std::get<OOBB>(collider.shape);
		const glm::vec3 size = box.halfExtentsLocal * 2.0f * t.scale;
		bool isUniform = approxEqual(size.x, size.y) and approxEqual(size.y, size.z);

		if (isUniform) {
			inertiaCube(size.x);
		}
		else {
			inertiaCuboid(size);
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



//// player controls
//if (player) {
//	glm::vec3 v = linearVelocity;

//	if (!onGround) {
//		playerMoveImpulse.x *= 0.02f;
//		playerMoveImpulse.z *= 0.02f;
//	}

//	v.x += playerMoveImpulse.x;
//	v.z += playerMoveImpulse.z;

//	if (!onGround) {
//		v += g * 3.0f * dt;
//	}
//	else if (v.y < 0.0f) {
//		v.y = 0.0f;
//	}

//	if (playerJumpImpulse != 0.0f) {
//		v.y += playerJumpImpulse;
//		playerJumpImpulse = 0.0f;
//	}

//	float maxSpeed = 15.0f;
//	glm::vec2 vxz(v.x, v.z);
//	float spd = glm::length(vxz);
//	if (spd > maxSpeed) {
//		vxz *= (maxSpeed / spd); // skala båda komponenterna lika
//		v.x = vxz.x;
//		v.z = vxz.y;
//	}

//	//if (onGround and glm::length2(playerMoveImpulse) < 0.01f) {
//	//    v.x -= v.x * std::min(20.0f * dt, 3.0f);
//	//    v.z -= v.z * std::min(20.0f * dt, 3.0f);
//	//}

//	if (onGround and glm::length2(playerMoveImpulse) < 0.01f) {
//		v.x = 0.0f;
//		v.z = 0.0f;
//	}

//	linearVelocity = v;

//	if (onGround) {
//		hasJumped = false;
//	}

//	onGround = false;
//}