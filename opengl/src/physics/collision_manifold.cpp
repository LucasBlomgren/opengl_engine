#include "collision_manifold.h"

static constexpr int N = 3;
using ContactFn = void(CollisionManifold::*)(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, GameObject& A, GameObject& B, glm::vec3 n, int& normalOwner);

static ContactFn dispatchTable[N][N] = {
    { &CollisionManifold::cuboidVsCuboid,   nullptr,                            nullptr                        },
    { &CollisionManifold::sphereVsCuboid,   &CollisionManifold::sphereVsSphere, nullptr                        },
    { &CollisionManifold::meshVsCuboid,     &CollisionManifold::meshVsSphere,   &CollisionManifold::meshVsMesh }
};

// skapa en struct som håller pekare till 3 olika cases: 1. gameobject 2. gameobject + chosenTri (från BVH som hittat rätt tri på mesh) 3. tri-primitive
// skicka in två av dessa Collider A och Collider B till createContact istället för GameObject A och GameObject B
// detta för att kunna skicka in collider som inte är GameObject, t.ex. tri-primitive
// 
// collider struct ska ha getter-metoder för t.ex. PreComputePointData(), så att antingen får man 0 angularVelocity/invMass etc. ->
// -> om det är en tri-primitive, annars får man GameObjects värde

void CollisionManifold::createContact(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, GameObject& A, GameObject& B, glm::vec3 n, int& normalOwner) {
    int i = int(A.collider.shape.index());
    int j = int(B.collider.shape.index());

    if (i < j)  std::swap(i, j); // Ensure i >= j for the dispatch table

    auto fn = dispatchTable[i][j];
    (this->*fn)(outContact, contactCache, A, B, n, normalOwner);
}

void CollisionManifold::cuboidVsCuboid(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, GameObject& A, GameObject& B, glm::vec3 n, int& normalOwner) {
    // välj vilken som är referensface och incidentface
    std::array<glm::vec3, 4> referenceFace;
    std::array<glm::vec3, 4> incidentFace;
    if (normalOwner == 0) {
        selectCollisionFace(A, n, referenceFace);
        selectCollisionFace(B, -n, incidentFace);
    }
    else {
        selectCollisionFace(B, -n, referenceFace);
        selectCollisionFace(A, n, incidentFace);
    }

    // räkna ut normalen för referensface
    glm::vec3 edgeA = referenceFace[0] - referenceFace[1];
    glm::vec3 edgeB = referenceFace[0] - referenceFace[2];
    glm::vec3 referenceFaceNormal = glm::normalize(glm::cross(edgeA, edgeB));

    InitialContact initialContact{ &A, &B, referenceFace, incidentFace, referenceFaceNormal };

    createContactPoints(initialContact);
    createLocalCoordinates(initialContact);
    if (initialContact.counter > 4) { contactPointReduction(initialContact); }
    computePenetrationDepth(initialContact);

    outContact.objA_ptr = &A;
    outContact.objB_ptr = &B;
    outContact.normal = n;

    integrateContact(contactCache, initialContact, outContact);
}

void CollisionManifold::sphereVsCuboid(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, GameObject& A, GameObject& B, glm::vec3 n, int& normalOwner) {
    
}
void CollisionManifold::sphereVsSphere(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, GameObject& A, GameObject& B, glm::vec3 n, int& normalOwner) {
    
}
void CollisionManifold::meshVsCuboid(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, GameObject& A, GameObject& B, glm::vec3 n, int& normalOwner) {
    
}
void CollisionManifold::meshVsSphere(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, GameObject& A, GameObject& B, glm::vec3 n, int& normalOwner) {
    
}
void CollisionManifold::meshVsMesh(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, GameObject& A, GameObject& B, glm::vec3 n, int& normalOwner) {
    
}

void CollisionManifold::selectCollisionFace(GameObject& obj, const glm::vec3& normal, std::array<glm::vec3, 4>& outFace) {
    glm::vec3 rotated = obj.invRotationMatrix * normal;
    glm::vec3 absN = glm::abs(rotated);

    if (absN.x >= absN.y and absN.x >= absN.z) 
        outFace = (rotated.x > 0) ? obj.aabb.wFaces.maxX : obj.aabb.wFaces.minX;
    else if (absN.y >= absN.x and absN.y >= absN.z)
        outFace = (rotated.y > 0) ? obj.aabb.wFaces.maxY : obj.aabb.wFaces.minY;
    else 
        outFace = (rotated.z > 0) ? obj.aabb.wFaces.maxZ : obj.aabb.wFaces.minZ;
}

void CollisionManifold::createClippingPlanes(const std::array<glm::vec3,4>& face, const glm::vec3& faceNormal) {
    this->clippingPlanes = {};
    // create clipping planes
    for (int i = 0; i < face.size(); i++) {
        glm::vec3 edge = face[i] - face[(i + 1) % face.size()];
        glm::vec3 planeNormal = (glm::normalize(glm::cross(faceNormal, edge)));

        this->clippingPlanes[i] = Plane(planeNormal, face[i]);
    }
}

void CollisionManifold::getIntersectionPoint(const glm::vec3& v1, const glm::vec3& v2, const Plane& plane, glm::vec3& outPoint, bool& outBool) {
    glm::vec3 lineDir = v2 - v1;
    float denominator = glm::dot(plane.normal, lineDir);

    if (std::abs(denominator) < 1e-6f) {
       // Linjen är parallell med planet
        outBool = false;
        return;
    }

    float t = glm::dot(plane.normal, plane.point - v1) / denominator;

    if (t < 0.0f || t > 1.0f) {
        outBool = false;
        return;
    }

    outPoint = v1 + t * lineDir;
}

bool CollisionManifold::isPointInsidePlane(const glm::vec3& point, const glm::vec3& planeNormal, const glm::vec3 planePoint) {
    if (glm::dot(planeNormal, point - planePoint) < 1e-2f)
        return true;

    return false;
}

void CollisionManifold::createContactPoints(InitialContact& initialContact) {
   std::array<glm::vec3, 4>& referenceFace = initialContact.referenceFace; 
   std::array<glm::vec3, 4>& incidentFace = initialContact.incidentFace;
   glm::vec3& referenceFaceNormal = initialContact.referenceFaceNormal; 

   createClippingPlanes(referenceFace, referenceFaceNormal);

   this->contactPoints = {};
   this->nextContactPoints = {};
   this->clippingStatus = {};

   for (int i = 0; i < incidentFace.size(); i++) {
      contactPoints[i] = incidentFace[i];
   }
   int counter = 4;
   int counter2 = 4;

   // clip alla punkter mot alla plan
   for (const Plane& plane : this->clippingPlanes) {
      for (int i = 0; i < counter; i++) {
         clippingStatus[i] = isPointInsidePlane(contactPoints[i], plane.normal, plane.point);  // håll koll på vilka punkter som är innanför detta specifika plan
      }

      counter2 = counter;
      counter = 0;

      // polygon clipping algorithm
      for (int i = 0; i < counter2; i++) {
         int nextIndex = (i + 1) % counter2;

         bool validClip = true;
         glm::vec3 clippedPoint;
         if (!clippingStatus[i] and clippingStatus[nextIndex]) {
            getIntersectionPoint(contactPoints[i], contactPoints[nextIndex], plane, clippedPoint, validClip);
            if (validClip) nextContactPoints[counter++] = clippedPoint;
            nextContactPoints[counter++] = contactPoints[nextIndex];
         }
         else if (clippingStatus[i] and clippingStatus[nextIndex]) {
            nextContactPoints[counter++] = contactPoints[nextIndex];
         }
         else if (clippingStatus[i] and !clippingStatus[nextIndex]) {
            getIntersectionPoint(contactPoints[i], contactPoints[nextIndex], plane, clippedPoint, validClip);
            if (validClip) nextContactPoints[counter++] = clippedPoint;
         }
      }
      contactPoints = nextContactPoints;
   }

   // behåll de punkter som är innanför referensface
   for (int i = 0; i < counter; i++) {
      if (isPointInsidePlane(contactPoints[i], referenceFaceNormal, referenceFace[0])) 
         initialContact.globalCoords[initialContact.counter++] = contactPoints[i];
   }
}

void CollisionManifold::createLocalCoordinates(InitialContact& initialContact) {
    glm::mat4& invM = initialContact.objA_ptr->invModelMatrix;

    for (int i = 0; i < initialContact.counter; i++) {
        glm::vec4 local = invM * glm::vec4(initialContact.globalCoords[i], 1.0f);
        initialContact.localCoords[i] = glm::vec3(local);
    }
}

void CollisionManifold::contactPointReduction(InitialContact& contactPoints) {
    glm::vec3 normal = contactPoints.referenceFaceNormal;
    std::array<glm::vec3, 4>  finalGlobalPoints{};
    std::array<glm::vec3, 4>  finalLocalPoints{};

    // hitta den punkt (supportPoint) som är längst bort i x-led
    glm::vec3 supportPoint;
    const glm::vec3 direction = glm::vec3(1.0f, 0.0f, 0.0f);
    float maxDot = std::numeric_limits<float>::lowest();
    int supportPointIndex = 0;
    for (int i = 0; i < contactPoints.counter; i++) {
        glm::vec3& point = contactPoints.localCoords[i];

        float dotValue = glm::dot(point, direction);
        if (dotValue > maxDot) {
            maxDot = dotValue;
            supportPoint = point;
            supportPointIndex = i;
        }
    }
    finalGlobalPoints[0] = contactPoints.globalCoords[supportPointIndex];
    finalLocalPoints[0] = contactPoints.localCoords[supportPointIndex];

    // hitta den punkt som är längst bort från supportPoint
    int farthestPointIndex = 0;
    glm::vec3 farthestPoint{};
    float maxDistance = std::numeric_limits<float>::lowest();
    for (int i = 0; i < contactPoints.counter; i++) {
        glm::vec3& point = contactPoints.localCoords[i];

        float distance = glm::distance2(supportPoint, point);
        if (distance > maxDistance) {
            maxDistance = distance;
            farthestPoint = point;
            farthestPointIndex = i;
        }
    }
    finalGlobalPoints[1] = contactPoints.globalCoords[farthestPointIndex];
    finalLocalPoints[1] = contactPoints.localCoords[farthestPointIndex];

    // hitta den punkt som bildar den största positiva triangeln med supportPoint och farthestPoint (pga crossproduct winding order)
    int trianglePointIndex = 0;
    float maxArea = std::numeric_limits<float>::lowest();
    for (int i = 0; i < contactPoints.counter; i++) {
        glm::vec3& point = contactPoints.localCoords[i];

        float area = 0.5f * glm::dot(glm::cross(supportPoint - farthestPoint, supportPoint - point), normal);
        if (area > maxArea) {
            maxArea = area;
            trianglePointIndex = i;
        }
    }
    finalGlobalPoints[2] = contactPoints.globalCoords[trianglePointIndex];
    finalLocalPoints[2] = contactPoints.localCoords[trianglePointIndex];

    // hitta den punkt som bildar den största negativa triangeln med supportPoint och farthestPoint (pga motsatt crossproduct winding order)
    int negTrianglePointIndex = 0;
    maxArea = std::numeric_limits<float>::max();
    for (int i = 0; i < contactPoints.counter; i++) {
        glm::vec3& point = contactPoints.localCoords[i];

        float area = 0.5f * glm::dot(glm::cross(supportPoint - farthestPoint, supportPoint - point), normal);
        if (area < 0 and area < maxArea) {
            maxArea = area;
            negTrianglePointIndex = i;
        }
    }
    finalGlobalPoints[3] = contactPoints.globalCoords[negTrianglePointIndex];
    finalLocalPoints[3] = contactPoints.localCoords[negTrianglePointIndex];

    contactPoints.globalCoords = { finalGlobalPoints[0], finalGlobalPoints[1], finalGlobalPoints[2], finalGlobalPoints[3], {}, {}, {}, {} };
    contactPoints.localCoords = { finalLocalPoints[0], finalLocalPoints[1], finalLocalPoints[2], finalLocalPoints[3], {}, {}, {}, {} };
    contactPoints.counter = 4;
}

void CollisionManifold::computePenetrationDepth(InitialContact& initialContact) {
    for (int i = 0; i < initialContact.counter; i++)
        initialContact.pointDepths[i] = -glm::dot(initialContact.globalCoords[i] - initialContact.referenceFace[0], initialContact.referenceFaceNormal);
}

void CollisionManifold::PreComputePointData(ContactPoint& cp, glm::vec3& normal, GameObject& objA, GameObject& objB) {
    constexpr float restitutionThreshold = 0.1f; // Minsta hastighet för att restitution ska aktiveras
    constexpr float restitution = 0.1f; // exempelmaterial

    // pre-calculate rA, rB, EffectiveMass
    cp.rA = cp.globalCoord - objA.position;
    cp.rB = cp.globalCoord - objB.position;

    glm::vec3 rA_cross_n = glm::cross(cp.rA, normal);
    glm::vec3 rB_cross_n = glm::cross(cp.rB, normal);
    cp.m_eff = 1.0f / (objA.invMass + objB.invMass +
        glm::dot(rA_cross_n, objA.inverseInertia * rA_cross_n) +
        glm::dot(rB_cross_n, objB.inverseInertia * rB_cross_n));

    // Räkna ut den relativa hastigheten vid kontaktpunkten, baserat på de aktuella kropparnas tillstånd
    glm::vec3 relativeVelocity = 
        (objB.linearVelocity + glm::cross(objB.angularVelocity, cp.rB)) -
        (objA.linearVelocity + glm::cross(objA.angularVelocity, cp.rA));

    // Dela upp i normal- och tangentiell komponent
    glm::vec3 v_normal = glm::dot(relativeVelocity, normal) * normal;
    glm::vec3 v_tangent = relativeVelocity - v_normal;
    float vt2 = glm::dot(v_tangent, v_tangent);

    glm::vec3 t1, t2;
    constexpr float epsilon = 1e-3f;
    constexpr float eps2 = epsilon * epsilon;

    // Om den tangentiella hastigheten är signifikant, använd den för att bestämma t1
    if (vt2 > eps2) {
        t1 = glm::normalize(v_tangent);
        // t2 är vinkelrät mot både kontaktnormalen och t1
        t2 = glm::cross(normal, t1);
    }
    else {
        // Om relativeVelocity är nära noll, välj en standardreferens som inte är parallell med kontaktnormalen
        glm::vec3 reference = (std::abs(normal.y) < 0.9f) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
        t1 = glm::cross(normal, reference);
        t2 = glm::cross(normal, t1);
    }
    cp.t1 = t1;
    cp.t2 = t2;

    float normalVelocity = glm::dot(relativeVelocity, normal);
    if (normalVelocity < -restitutionThreshold) {
        cp.targetBounceVelocity = -restitution * normalVelocity;
    }
    else {
        cp.targetBounceVelocity = 0.0f;
    }

    glm::vec3 rA_t1 = glm::cross(cp.rA, t1);
    glm::vec3 rB_t1 = glm::cross(cp.rB, t1);
    glm::vec3 rA_t2 = glm::cross(cp.rA, t2);
    glm::vec3 rB_t2 = glm::cross(cp.rB, t2);

    glm::vec3 invIA_rA_t1 = objA.inverseInertia * rA_t1;
    glm::vec3 invIB_rB_t1 = objB.inverseInertia * rB_t1;
    glm::vec3 invIA_rA_t2 = objA.inverseInertia * rA_t2;
    glm::vec3 invIB_rB_t2 = objB.inverseInertia * rB_t2;

    // Beräkna effektiv massa längs cp.t1 och cp.t2 
    float k_t1 = (objA.invMass + objB.invMass) + glm::dot(rA_t1, invIA_rA_t1) + glm::dot(rB_t1, invIB_rB_t1);
    cp.invMassT1 = 1.0f / k_t1;
    float k_t2 = (objA.invMass + objB.invMass) + glm::dot(rA_t2, invIA_rA_t2) + glm::dot(rB_t2, invIB_rB_t2);
    cp.invMassT2 = 1.0f / k_t2;

    cp.invMassTwist = 1.0f / (glm::dot(normal, objA.inverseInertia * normal) + glm::dot(normal, objB.inverseInertia * normal));
}

size_t CollisionManifold::generateKey(int idA, int idB) {
    return (uint64_t)std::min(idA, idB) << 32 | std::max(idA, idB);
}

void CollisionManifold::integrateContact(std::unordered_map<size_t, Contact>& contactCache, InitialContact& initialContact, Contact& finalContact) {
    // kopiera över punkterna från initialContact till finalContact
    for (int i = 0; i < initialContact.counter; i++) {
        finalContact.points[i].globalCoord = initialContact.globalCoords[i];
        finalContact.points[i].localCoord = initialContact.localCoords[i];
        finalContact.points[i].depth = initialContact.pointDepths[i];
    }
    finalContact.counter = initialContact.counter;

    for (int i = 0; i < finalContact.counter; i++) 
        PreComputePointData(finalContact.points[i], finalContact.normal, *finalContact.objA_ptr, *finalContact.objB_ptr);

    // kolla om det redan finns en contact mellan objA och objB
    int idA = initialContact.objA_ptr->id;
    int idB = initialContact.objB_ptr->id;
    size_t key = generateKey(idA, idB);
    auto it = contactCache.find(key);

    // ingen matchande cached contact
    if (it == contactCache.end()) {
        contactCache[key] = finalContact;
        return;
    }

    // matcha nya punkter med gamla punkter eftersom finalContact inte matchar med en cached contact
    Contact& cachedContact = it->second;
    cachedContact.wasUsedThisFrame = true;

    std::array<bool, 4> matchedFinalPoints{ false };
    std::array<bool, 4> matchedCachedPoints{ false };
    glm::vec3 referenceFaceNormal = initialContact.referenceFaceNormal;
    std::array<glm::vec3, 4> referenceFace = initialContact.referenceFace;

    // -------------- behöver ändras till att vara en faktor av objektens storlekar -------------
    const float threshold = 0.5f;

    // iterera över alla nya contact points och se om någon är nära en existerande
    for (int i = 0; i < finalContact.counter; i++) {
        ContactPoint& newPoint = finalContact.points[i];
        for (int j = 0; j < cachedContact.counter; j++) {

            if (matchedFinalPoints[i]) break;
            if (matchedCachedPoints[j]) continue;

            ContactPoint& cachedPoint = cachedContact.points[j];

            // nya punkten är nära en cached punkt = warm start
            float dist2 = glm::distance2(newPoint.localCoord, cachedPoint.localCoord);
            if (dist2 < threshold * threshold) {
                if (glm::dot(cachedPoint.t1, newPoint.t1) < 0) newPoint.t1 = -newPoint.t1;
                if (glm::dot(cachedPoint.t2, newPoint.t2) < 0) newPoint.t2 = -newPoint.t2;

                // 1) Gör om gamla skalärer till en världsrums‑vektor
                glm::vec3 oldImpulseWorld =
                    cachedPoint.accumulatedImpulse * cachedContact.normal +
                    cachedPoint.accumulatedFrictionImpulse1 * cachedPoint.t1 +
                    cachedPoint.accumulatedFrictionImpulse2 * cachedPoint.t2;

                // 2) Projicera på den NYA basen
                newPoint.accumulatedImpulse = glm::dot(oldImpulseWorld, finalContact.normal);
                newPoint.accumulatedFrictionImpulse1 = glm::dot(oldImpulseWorld, newPoint.t1);
                newPoint.accumulatedFrictionImpulse2 = glm::dot(oldImpulseWorld, newPoint.t2);

                // 3) Twist‑impulsen följer bara normala rotationsaxeln ⇒ kopiera
                newPoint.accumulatedTwistImpulse = cachedPoint.accumulatedTwistImpulse;

                matchedFinalPoints[i] = true;
                matchedCachedPoints[j] = true;
                break;
            }
        }
    }

    // fyll på med cachade punkter som inte blivit matchade med en ny punkt
    if (finalContact.counter < 4) {
        for (int i = 0; i < cachedContact.counter; i++) {
            if (matchedCachedPoints[i] == true)
                continue;

            ContactPoint& cachedPoint = cachedContact.points[i];
            if ((glm::length((cachedPoint.globalCoord - referenceFace[0]) - glm::dot(cachedPoint.globalCoord - referenceFace[0], referenceFaceNormal) * referenceFaceNormal) > threshold) || (glm::abs(glm::dot(referenceFaceNormal, cachedPoint.globalCoord - referenceFace[0]) > threshold))) {
                continue;
            }

            PreComputePointData(cachedPoint, finalContact.normal, *finalContact.objA_ptr, *finalContact.objB_ptr);
            finalContact.points[finalContact.counter++] = cachedPoint;
            
            if (finalContact.counter >= 4) {
                break;
            }
        }
    }

    contactCache[key] = finalContact;
}