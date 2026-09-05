#include "pch.h"
#include "rigidbody.h"

namespace physics::internal {

//===============================================
//   Velocity integration
//===============================================
void RigidBody::integratePose(float dt) {
	if (type == BodyType::Static || 
		motionControl == MotionControl::External)
	{
		return;
	}

	pose.position += linearVelocity * dt;
	updateOrientation(pose.orientation, angularVelocity, dt);
}

//===============================================
//  Gravity and friction/damping
//===============================================
void RigidBody::applyGravity(float dt) {
	if (!allowGravity)
		return;

	static constexpr glm::vec3 g =
		glm::vec3(0.0f, -9.81f, 0.0f);

	linearVelocity += g * dt;
}

void RigidBody::applyVelocityDamping(float dt, SleepState& sleepState) {
    constexpr float epsilon = 0.01f;

    // only apply damping if there have been recent collisions,
    // no air resistance when flying freely
	if (sleepState.collisionHistory.average() > epsilon) {
        linearVelocity *= std::pow(0.98f, dt);
        angularVelocity *= std::pow(0.98f, dt);
    }
}

void RigidBody::applyRollingFriction(ColliderType colliderType, float dt, SleepState& sleepState) {
	if (colliderType != ColliderType::SPHERE)
		return;

	float aLin;
	float aAng;
	bool avgCollisions = sleepState.collisionHistory.average() > 0;

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

void RigidBody::applyAntistuckFriction(float dt, SleepState& sleepState) {
	// anti stuck for objects with high collision counts
	bool avgCollisions = sleepState.collisionHistory.average() > 3;
	if (avgCollisions) {
		linearVelocity = linearVelocity * std::pow(0.98f, dt);
		angularVelocity = angularVelocity * std::pow(0.98f, dt);
	}
}

//===============================================
//    Orientation and inertia updates
//===============================================
void RigidBody::updateOrientation(glm::quat& orientation, const glm::vec3& angularVelocity, float dt) {
	glm::quat omegaQuat(0.0f, angularVelocity.x, angularVelocity.y, angularVelocity.z);
	orientation += 0.5f * dt * (omegaQuat * orientation);
	orientation = glm::normalize(orientation);
}

void RigidBody::updateInertiaWorld() {
	glm::mat3 rotation = glm::mat3_cast(pose.orientation);
	invInertiaWorld = rotation * invInertiaLocal * glm::transpose(rotation);
}

//===============================================
//     Applying impulses
//===============================================
void RigidBody::applyImpulseLinear(const glm::vec3& j) {
    linearVelocity += j * invMass;
}
void RigidBody::applyImpulseAngular(const glm::vec3& j) {
    angularVelocity += j * invInertiaWorld;
}
void RigidBody::pushBiasImpulseLinear(const glm::vec3& j) {
    biasLinearVelocity += j * invMass;
}
void RigidBody::pushBiasImpulseAngular(const glm::vec3& j) {
    biasAngularVelocity += j * invInertiaWorld;
}
void RigidBody::commitBiasImpulses(float dt) {
	pose.position += biasLinearVelocity * dt;
	updateOrientation(pose.orientation, biasAngularVelocity, dt);

	biasLinearVelocity = glm::vec3(0.0f);
	biasAngularVelocity = glm::vec3(0.0f);
}

//===============================================
//     Asleep and awake
//===============================================
void RigidBody::setAsleep(SleepState& sleepState) {
	if (type == BodyType::Static || type == BodyType::Kinematic) {
		return;
	}

	linearVelocity = glm::vec3(0.0f);
	angularVelocity = glm::vec3(0.0f);
	biasLinearVelocity = glm::vec3(0.0f);
	biasAngularVelocity = glm::vec3(0.0f);

	sleepState.asleep = true;
	sleepState.counter = 0.0f;

	sleepState.anchorTimer = 0.0f;
	sleepState.anchorPoint = pose.position;
}


void RigidBody::setAwake(SleepState& sleepState) {
	if (type == BodyType::Static)
		return;

	sleepState.asleep = false;
	sleepState.counter = 0.0f;
}

//===============================================
//     State setters
//===============================================
void RigidBody::setStatic() {
	type = BodyType::Static;
	mass = 0.0f;
	invMass = 0.0f;
}

void RigidBody::setMotionControl(MotionControl mode) {
	motionControl = mode;
}

//===============================================
//     Inertia calculations
//===============================================
void RigidBody::calculateInverseInertia(
    const ColliderType& colliderType,
    const Collider& collider,
    const glm::vec3& inertiaScale) 
{
	if (type == BodyType::Static) {
		invInertiaLocal = glm::mat3(0.0f);
		return;
	}

	if (colliderType == ColliderType::CUBOID) {
		const OOBB& box = std::get<OOBB>(collider.shape);
		const glm::vec3 size = box.localHalfExtents * 2.0f * inertiaScale;
		bool isUniform = approxEqual(size.x, size.y) and approxEqual(size.y, size.z);

		if (isUniform) {
			inertiaCube(size.x);
		}
		else {
			inertiaCuboid(size);
		}
	}
	else if (colliderType == ColliderType::SPHERE) {
		inertiaSphere(inertiaScale);
	}

	invInertiaWorld = invInertiaLocal;
	updateInertiaWorld();
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


}