#include "physics.h"

void PhysicsEngine::setPointers(std::vector<GameObject>* gameObjectList, BVHTree* tree) {
    this->gameObjectList = gameObjectList;
    this->bvhTree = tree;
}

BVHTree* PhysicsEngine::getBvhTree() const {
    return bvhTree;
}

RaycastHit PhysicsEngine::performRaycast(Ray& r)
{
    RaycastHit hitData = raycast(r, this->gameObjectList, this->bvhTree);
    return hitData;
}

const std::unordered_map<size_t, Contact>& PhysicsEngine::GetContactCache() const {
    return contactCache;
}

void PhysicsEngine::clearPhysicsData() {
    contactCache.clear();
}

void PhysicsEngine::updatePositions(float deltaTime) 
{
    for (GameObject& object : *gameObjectList)
    {
        object.updatePos(deltaTime);
        object.updateAABB();

        if (!object.isStatic)
            object.OOBB_shouldUpdate = true;
    }
}

void PhysicsEngine::step(float deltaTime, bool showNormals, std::mt19937 rng)
{
   std::ofstream log("physics.log", std::ios::app);


    updatePositions(deltaTime);

    int totalCount = 0;

    // bvh collision query
    for (GameObject& objA : *gameObjectList) {

        int count = bvhTree->query(objA.AABB);

        for (int i = 0; i < count; i++) {
            GameObject& objB = *bvhTree->collisions[i];
            if (objB.id < objA.id) continue;
            if (objA.id == objB.id) continue;

            if (objA.isStatic and objB.isStatic) 
                continue;

            // set awake
            float velocityThreshold = 4;
            float angularVelocityThreshold = 4;

            float velocityThreshold2 = velocityThreshold * velocityThreshold;
            float angularVelocityThreshold2 = angularVelocityThreshold * angularVelocityThreshold;

            float linearVelocityLen;
            float angularVelocityLen;
            if (objA.asleep) {
                linearVelocityLen = glm::dot(objB.linearVelocity, objB.linearVelocity);
                angularVelocityLen = glm::dot(objB.angularVelocity, objB.angularVelocity);
                if (linearVelocityLen > velocityThreshold2 or angularVelocityLen > angularVelocityThreshold2) {
                    objA.setAwake();
                }
            }
            if (objB.asleep) {
                linearVelocityLen = glm::dot(objA.linearVelocity, objA.linearVelocity);
                angularVelocityLen = glm::dot(objA.angularVelocity, objA.angularVelocity);
                if (linearVelocityLen > velocityThreshold2 or angularVelocityLen > angularVelocityThreshold2) {
                    objB.setAwake();
                }
            }
            // sleep check
            if ((objA.asleep and (objB.asleep or objB.isStatic)) or (objB.asleep and (objA.asleep or objA.isStatic)))
            { 
                objA.setAsleep();
                objB.setAsleep();
                continue;
            }

            objA.updateOOBB();
            objB.updateOOBB();

            // Narrow phase
            glm::vec3 collisionNormal;
            float depth = std::numeric_limits<float>::max();
            int collisionNormalOwner = 0;
            if (IntersectPolygons(objA, objB, collisionNormal, depth, collisionNormalOwner))
            {
                glm::vec3 direction = objB.position - objA.position;
                if (glm::dot(direction, collisionNormal) < 0)
                    collisionNormal = -collisionNormal;

                // contactPoints
                Contact contact = createContact(contactCache, objA, objB, collisionNormal, collisionNormalOwner);

                //std::shuffle(contact.points.begin(), contact.points.begin() + contact.counter, rng);

                float slop = 0.01f;
                float baumgarteFactor = 0.1f;
                for (int j = 0; j < contact.counter; j++) {
                    ContactPoint& cp = contact.points[j];

                    float penetration = cp.depth;             
                    float allowed = penetration - slop;

                    if (allowed > 0.0f) {
                       cp.biasVelocity = glm::min(-(baumgarteFactor * allowed) / deltaTime, 0.0f); // Baumgarte-bias
                       //float rawBias = -(baumgarteFactor * allowed) / deltaTime;
                       //cp.biasVelocity = glm::clamp(rawBias, -999.0f, 0.0f);
                    }
                    else {
                       cp.biasVelocity = 0.0f;
                    }
                }

                float staticFriction = 0.8f; // Justera beroende på material
                float dynamicFriction = 0.5f; 

                // PGS solver
                int maxIterations = 20;
                for (int i = 0; i < maxIterations; i++) {
                    bool converged = true;

                    for (int j = 0; j < contact.counter; j++) {
                        ContactPoint& cp = contact.points[j];

                        glm::vec3 relativeVelocity = (objB.linearVelocity + glm::cross(objB.angularVelocity, cp.rB)) -
                            (objA.linearVelocity + glm::cross(objA.angularVelocity, cp.rA));

                        float normalVelocity = glm::dot(relativeVelocity, contact.normal);

                        // Baumgarte-bias inbyggd i impuls­beräkningen:
                        float v_target = cp.targetBounceVelocity;          // studs-mål
                        float v_bias = cp.biasVelocity;                    // penetrations-bias
                        // villkor: normalVelocity + v_bias = önskad hastighet vid kontakten
                        float J = -(normalVelocity - v_target + v_bias) * cp.m_eff;

                        // Clamp
                        float temp = cp.accumulatedImpulse;
                        cp.accumulatedImpulse = glm::max(temp + J, 0.0f);
                        float deltaImpulse = cp.accumulatedImpulse - temp;

                        // Add normal to impulse
                        glm::vec3 deltaNormalImpulse = deltaImpulse * contact.normal;

                        // Apply impulses
                        float deltaNormalImpulseLen = glm::dot(deltaNormalImpulse, deltaNormalImpulse);
                        if (deltaNormalImpulseLen > 1e-10f) {
                            objA.linearVelocity -= deltaNormalImpulse * objA.invMass;
                            objA.angularVelocity -= objA.inverseInertia * glm::cross(cp.rA, deltaNormalImpulse);

                            objB.linearVelocity += deltaNormalImpulse * objB.invMass;
                            objB.angularVelocity += objB.inverseInertia * glm::cross(cp.rB, deltaNormalImpulse);

                            // impulse was changed
                            converged = false;
                        }

                        //--------------- Pre-calculate for friction ----------------
                        relativeVelocity = (objB.linearVelocity + cross(objB.angularVelocity, cp.rB)) - (objA.linearVelocity + cross(objA.angularVelocity, cp.rA));
                        // Project tangential velocity to the tangent plane
                        float v_t1 = glm::dot(relativeVelocity, cp.t1);
                        float v_t2 = glm::dot(relativeVelocity, cp.t2);
                        // Combine the tangential velocities to a vector
                        float vtMagnitude = v_t1 * v_t1 + v_t2 * v_t2;

                        glm::vec3 rA_t1 = glm::cross(cp.rA, cp.t1);
                        glm::vec3 rB_t1 = glm::cross(cp.rB, cp.t1);
                        glm::vec3 rA_t2 = glm::cross(cp.rA, cp.t2);
                        glm::vec3 rB_t2 = glm::cross(cp.rB, cp.t2);

                        glm::vec3 invIA_rA_t1 = objA.inverseInertia * rA_t1;
                        glm::vec3 invIB_rB_t1 = objB.inverseInertia * rB_t1;
                        glm::vec3 invIA_rA_t2 = objA.inverseInertia * rA_t2;
                        glm::vec3 invIB_rB_t2 = objB.inverseInertia * rB_t2;

                        // Beräkna effektiv massa längs cp.t1 och cp.t2 (som tidigare)
                        float k_t1 = (objA.invMass + objB.invMass) + glm::dot(rA_t1, invIA_rA_t1) + glm::dot(rB_t1, invIB_rB_t1);
                        float invMassT1 = 1.0f / k_t1;

                        float k_t2 = (objA.invMass + objB.invMass) + glm::dot(rA_t2, invIA_rA_t2) + glm::dot(rB_t2, invIB_rB_t2);
                        float invMassT2 = 1.0f / k_t2;

                        // Beräkna preliminära impulser i varje riktning:
                        float J1 = -v_t1 * invMassT1;
                        float J2 = -v_t2 * invMassT2;

                        // ---------- Static friction ----------
                        glm::vec3 desiredFrictionImpulse = (J1 * cp.t1) + (J2 * cp.t2);
                        float JtLen2 = glm::dot(desiredFrictionImpulse, desiredFrictionImpulse);
                        float Jn = fabs(cp.accumulatedImpulse);
                        // Beräkna maximal statisk friktionsimpuls (mu_static * |accumulatedImpulse|)
                        float Jmax = staticFriction * Jn;
                        float max2 = Jmax * Jmax; // max2 = mu_static^2 * |accumulatedImpulse|^2

                        if (JtLen2 <= max2) {
                            float impulseMag = glm::length(desiredFrictionImpulse);
                            if (JtLen2 > max2) {
                                float invMag = 1.0f / glm::sqrt(JtLen2);
                                desiredFrictionImpulse *= (Jmax * invMag);
                            }

                            // Applicera den statiska friktionsimpulsen:
                            objA.linearVelocity -= desiredFrictionImpulse * objA.invMass;
                            objA.angularVelocity -= objA.inverseInertia * glm::cross(cp.rA, desiredFrictionImpulse);

                            objB.linearVelocity += desiredFrictionImpulse * objB.invMass;
                            objB.angularVelocity += objB.inverseInertia * glm::cross(cp.rB, desiredFrictionImpulse);
                        }

                        // ---------- Dynamisk friktion (när kontakt glider) ----------
                        else
                        {
                            // Uppdatera de ackumulerade friktionsimpulserna (använd samma klampningsmetod som du gör idag)
                            float newFrictionImpulse1 = cp.accumulatedFrictionImpulse1 + J1;
                            float newFrictionImpulse2 = cp.accumulatedFrictionImpulse2 + J2;

                            // Klampar impulsen så att total friktion inte överskrider mu_d * |accumulatedImpulse|
                            float mu_dynamic = dynamicFriction; 
                            float maxFriction = mu_dynamic * fabs(cp.accumulatedImpulse);
                            float lengthSq = newFrictionImpulse1 * newFrictionImpulse1 + newFrictionImpulse2 * newFrictionImpulse2;
                            float maxFrictionSq = maxFriction * maxFriction;
                            if (lengthSq > maxFrictionSq) {
                                float scale = maxFriction / sqrt(lengthSq);
                                newFrictionImpulse1 *= scale;
                                newFrictionImpulse2 *= scale;
                            }

                            float deltaFrictionImpulse1 = newFrictionImpulse1 - cp.accumulatedFrictionImpulse1;
                            float deltaFrictionImpulse2 = newFrictionImpulse2 - cp.accumulatedFrictionImpulse2;
                            cp.accumulatedFrictionImpulse1 = newFrictionImpulse1;
                            cp.accumulatedFrictionImpulse2 = newFrictionImpulse2;

                            // Bygg friktionsimpulsen i tangentplanet:
                            glm::vec3 frictionImpulse = (deltaFrictionImpulse1 * cp.t1) + (deltaFrictionImpulse2 * cp.t2);

                            // Applicera impulsen:
                            float frictionImpulseLen = glm::dot(frictionImpulse, frictionImpulse);
                            if (frictionImpulseLen > 1e-6f) {
                                objA.linearVelocity -= frictionImpulse * objA.invMass;
                                objA.angularVelocity -= objA.inverseInertia * glm::cross(cp.rA, frictionImpulse);

                                objB.linearVelocity += frictionImpulse * objB.invMass;
                                objB.angularVelocity += objB.inverseInertia * glm::cross(cp.rB, frictionImpulse);

                                converged = false;
                            }
                        }

                        // ---------- TWIST FRICTION ----------
                        // Beräkna den relativa rotationshastigheten kring kontaktnormalen
                        // Detta är hur snabbt de roterar relativt varandra kring "n"
                        float relativeAngularSpeed = glm::dot((objB.angularVelocity - objA.angularVelocity), contact.normal);

                        // Beräkna effektiv massa för twist. 
                        // Här använder vi kropparnas inverseInertia (i world space) projicerade på kontaktnormalen.
                        float effectiveMassTwist = 1.0f / (glm::dot(contact.normal, objA.inverseInertia * contact.normal) +
                            glm::dot(contact.normal, objB.inverseInertia * contact.normal));

                        // Beräkna preliminär twist impulse (i rotationsdomänen)
                        float twistImpulse = -relativeAngularSpeed * effectiveMassTwist;

                        // Klampa twistimpulsen baserat på en twist-friktionskoefficient. 
                        // Ofta används en formel liknande: maxTwistImpulse = mu_twist * fabs(cp.accumulatedImpulse)
                        // där cp.accumulatedImpulse är den totala normala impulsen.
                        float mu_twist = 0.2f; // Exempelvärde – justera efter behov
                        float maxTwistImpulse = mu_twist * fabs(cp.accumulatedImpulse);

                        // Ackumulera twistimpulsen
                        float oldTwistImpulse = cp.accumulatedTwistImpulse;
                        float newTwistImpulse = glm::clamp(oldTwistImpulse + twistImpulse, -maxTwistImpulse, maxTwistImpulse);
                        float deltaTwistImpulse = newTwistImpulse - oldTwistImpulse;
                        cp.accumulatedTwistImpulse = newTwistImpulse;

                        // Den twistimpuls vi applicerar är ett moment (angular impulse) kring kontaktnormalen
                        glm::vec3 twistImpulseVec = deltaTwistImpulse * contact.normal;

                        // Applicera twistimpulsen på kropparnas angulära hastigheter
                        float twistImpulseVecLen = glm::dot(twistImpulseVec, twistImpulseVec);
                        if (twistImpulseVecLen > 1e-6f) {
                            objA.angularVelocity -= objA.inverseInertia * twistImpulseVec;
                            objB.angularVelocity += objB.inverseInertia * twistImpulseVec;
                            converged = false;
                        }
                    }
                    // Om vi inte har någon impuls att applicera, är vi klara
                    if (converged)
                        break;
                }

                //slop = 0.01f;
                //const float percent = 0.4f;         // 60‑80 % brukar vara lagom
                //for (int j = 0; j < contact.counter; ++j) {
                //   ContactPoint& cp = contact.points[j];
                //   float penetration = -cp.depth - slop;
                //   if (penetration > 0.0f) {
                //      float invMassSum = objA.invMass + objB.invMass;
                //      glm::vec3 correction = percent * penetration / invMassSum * contact.normal;

                //      objA.position -= correction * objA.invMass;
                //      objB.position += correction * objB.invMass;
                //   }
                //}
            }
        }
    }

    int maxFramesWithoutCollision = 10; 
    for (auto it = contactCache.begin(); it != contactCache.end(); ) {
        if (!it->second.wasUsedThisFrame) {
            it->second.framesSinceUsed++;

            // Ta bort manifold efter X antal frames utan kollisionsmatch
            if (it->second.framesSinceUsed > maxFramesWithoutCollision) {
                it = contactCache.erase(it);
                continue;
            }
        }
        else {
            // Nollställ för nästa frame
            it->second.wasUsedThisFrame = false;
            it->second.framesSinceUsed = 0;  // Nollställ räknaren vid träff
        }
        ++it;
    }
}