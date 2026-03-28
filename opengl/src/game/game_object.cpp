#include "pch.h"
#include "game_object.h"

//void GameObject::initMesh() {
//	verticesPositions.clear();
//	for (const Vertex& vertex : mesh->vertices) {
//		verticesPositions.push_back(vertex.position);
//	}
//
//	modelMatrixDirty = true;
//	setModelMatrix();
//}

void GameObject::updatePos(const float& dt) {
	if (selectedByEditor or selectedByPlayer)
		return;

	//if (allowGravity and !player and !isStatic)
	//	linearVelocity += g * dt;

	float aLin;
	float aAng;
	//bool avgCollisions = collisionHistory.average() > 0;

	//if (colliderType == ColliderType::SPHERE) {
	//	aLin = 0.1f; // konstant linjär retardation (m/s^2)
	//	aAng = 1.0f; // konstant angulär retardation (rad/s^2)

	//	float vMag = glm::length(linearVelocity);
	//	if (vMag > 0.0f and avgCollisions) {
	//		float newMag = vMag - aLin * dt;
	//		if (newMag < 0.0f) newMag = 0.0f;
	//		linearVelocity *= (newMag / vMag); // behåll riktningen
	//	}

	//	float wMag = glm::length(angularVelocity);
	//	if (wMag > 0.0f and avgCollisions) {
	//		float newMag = wMag - aAng * dt;
	//		if (newMag < 0.0f) newMag = 0.0f;
	//		angularVelocity *= (newMag / wMag);
	//	}
	//}

	//// anti stuck for objects with high collision counts
	//avgCollisions = collisionHistory.average() > 3;
	//if (avgCollisions) {
	//	linearVelocity = linearVelocity * std::pow(0.98f, dt);
	//	angularVelocity = angularVelocity * std::pow(0.98f, dt);
	//}

	//// fake constraints
	//if (!canMoveLinearly) {
	//	linearVelocity = glm::vec3(0.0f);
	//	angularVelocity.x = 0.0f;
	//	angularVelocity.y = 0.0f;
	//}

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

	// update position and orientation
	//lastPosition = position;
	//position += linearVelocity * dt;
	//updateOrientation(orientation, angularVelocity, dt);

	//setRotatedFlag();
}
