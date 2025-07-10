#include "collision_manifold.h"

void CollisionManifold::boxBox(Contact& contact, std::unordered_map<size_t, Contact>& cache, SAT::Result& satResult) {
    // välj vilken som är referensface och incidentface
    std::vector<glm::vec3> referenceFace; 
    std::vector<glm::vec3> incidentFace;

    if (satResult.axisType == SAT::AxisType::FaceA or satResult.axisType == SAT::AxisType::EdgeEdge) {
        selectCollisionFace(*contact.objA_ptr, satResult.normal);
        referenceFace = this->selectedFace;
        selectCollisionFace(*contact.objB_ptr, -satResult.normal);
        incidentFace = this->selectedFace;
    }
    else if (satResult.axisType == SAT::AxisType::FaceB) {
        selectCollisionFace(*contact.objB_ptr, -satResult.normal);
        incidentFace = this->selectedFace;
        selectCollisionFace(*contact.objA_ptr, satResult.normal);
        referenceFace = this->selectedFace;
    }

    // räkna ut normalen för referensface
    glm::vec3 edgeA = referenceFace[0] - referenceFace[1]; 
    glm::vec3 edgeB = referenceFace[0] - referenceFace[2]; 
    glm::vec3 referenceFaceNormal = glm::normalize(glm::cross(edgeA, edgeB)); 

    contact.hashKey = generateKey(contact.objA_ptr->id, contact.objB_ptr->id); // skapa hash key för contact
    contact.normal = satResult.normal; 
    contact.referenceFace = referenceFace; 
    contact.incidentFace = incidentFace; 
    contact.referenceFaceNormal = referenceFaceNormal; 

    clipPoints(referenceFace, incidentFace, referenceFaceNormal);

    // skapa contact points från de klippta punkterna
    contact.points.resize(this->clippedPoints.size());
    for (int i = 0; i < clippedPoints.size(); i++) {
        contact.points[i].globalCoord = this->clippedPoints[i];
    }

    createLocalCoordinates(contact);  

    if (contact.points.size() > 4) { 
        contactPointReduction(contact);  
    } 

    computePenetrationDepth(contact);  
    integrateContact(cache, contact);  
}

void CollisionManifold::boxSphere(Contact& contact, std::unordered_map<size_t, Contact>& cache, SAT::Result& satResult) {
    contact.normal = satResult.normal;
    contact.points.resize(1); 
    contact.points[0].globalCoord = satResult.point; // point is the closest point on the cuboid to the sphere
    contact.points[0].depth = satResult.depth;       // penetration depth

    createLocalCoordinates(contact); 

    contact.hashKey = generateKey(contact.objA_ptr->id, contact.objB_ptr->id); 
    integrateContact(cache, contact); 
}

void CollisionManifold::sphereSphere(Contact& contact, std::unordered_map<size_t, Contact>& cache, SAT::Result& satResult) {
    contact.normal = satResult.normal;
    contact.points.resize(1);
    contact.points[0].globalCoord = satResult.point; // point is the closest point on the sphere to the sphere
    contact.points[0].depth = satResult.depth;       // penetration depth

    createLocalCoordinates(contact); 

    contact.hashKey = generateKey(contact.objA_ptr->id, contact.objB_ptr->id); 
    integrateContact(cache, contact); 
}
void CollisionManifold::sphereMesh(Contact& contact, std::unordered_map<size_t, Contact>& cache, std::vector<SAT::Result>& allResults) {
    this->allClippedPoints.clear();
    
    this->allClippedPoints.resize(allResults.size());
    for (int i = 0; i < allResults.size(); i++) {
        this->allClippedPoints[i] = allResults[i].point;
    }

    // --- pick furthest points from all "clipped points" ---
    pickFourFurthestPoints(); 

    // --- create contact points from furthest points ---
    contact.points.resize(this->furthestPoints.size()); 
    for (int i = 0; i < this->furthestPoints.size(); i++) { 
        contact.points[i].globalCoord = this->furthestPoints[i]; 
    } 

    // add depth for each cp
    contact.points[0].depth = allResults[0].depth;
    for (int i = 1; i < this->furthestPoints.size(); i++) {
        contact.points[i].depth = allResults[indices[i]].depth;
    }

    createLocalCoordinates(contact);  
    contact.hashKey = generateKey(contact.objA_ptr->id, allResults[0].tri_ptr->id);  
    integrateContact(cache, contact); 
}
void CollisionManifold::meshMesh(Contact& contact, std::unordered_map<size_t, Contact>& cache, SAT::Result& satResult) {
    
}

void CollisionManifold::boxMesh(Contact& contact, std::unordered_map<size_t, Contact>& cache, std::vector<SAT::Result>& allResults) {
    std::vector<glm::vec3> referenceFace; 
    std::vector<glm::vec3> incidentFace; 

    this->allClippedPoints.clear();
    this->allClippedPoints.reserve(8 * allResults.size());

    selectCollisionFace(*contact.objA_ptr, allResults[0].normal);
    referenceFace = this->selectedFace;
    contact.referenceFace = referenceFace;

    glm::vec3 edgeA = referenceFace[0] - referenceFace[1];
    glm::vec3 edgeB = referenceFace[0] - referenceFace[2];
    glm::vec3 refNormal = glm::normalize(glm::cross(edgeA, edgeB));
    contact.referenceFaceNormal = refNormal;

    // gameobject vs terrain triangles
    if (contact.objB_ptr == nullptr) { 

        // --- clip all triangles against the reference face ---
        for (const SAT::Result& satResult : allResults)   
        {
            incidentFace = satResult.tri_ptr->vertices; 

            clipPoints(referenceFace, incidentFace, refNormal);

            // save clipped points for furthest point selection
            this->allClippedPoints.insert(this->allClippedPoints.end(), this->clippedPoints.begin(), this->clippedPoints.end()); 
        }

        // --- pick furthest points from all clipped points ---
        pickFourFurthestPoints();

        // --- create contact points from furthest points ---
        contact.points.resize(this->furthestPoints.size());
        for (int i = 0; i < this->furthestPoints.size(); i++) {
            contact.points[i].globalCoord = this->furthestPoints[i];
        }

        createLocalCoordinates(contact);
        computePenetrationDepth(contact);

        contact.hashKey = generateKey(contact.objA_ptr->id, allResults[0].tri_ptr->id);
        integrateContact(cache, contact);
    }
}

void CollisionManifold::pickFourFurthestPoints() {
    int N = (int)this->allClippedPoints.size();
    if (N <= 4) {
        this->furthestPoints = this->allClippedPoints;
        this->indices.clear();
        this->indices.reserve(N);

        // for sphere vs tri depths index
        for (int i = 0; i < N; ++i) {
            this->indices.push_back(i);
        }

        return;
    }

    this->indices.clear();
    this->indices.reserve(4);
    this->indices.push_back(0);
    this->furthestPoints.resize(4);
    this->furthestPoints[0] = this->allClippedPoints[0];

    for (int i = 0; i < 3; i++) {
        addFurthestPoint(indices); 
    }

    std::vector<glm::vec3> temp;
    temp.reserve(indices.size()); 
    for (int idx : indices) { 
        temp.push_back(this->allClippedPoints[idx]); 
    }

    this->furthestPoints.swap(temp);
}

void CollisionManifold::addFurthestPoint(std::vector<int>& indices) {
    int bestIdx = -1;
    float bestDist = std::numeric_limits<float>::lowest();

    for (int i = 0; i < this->allClippedPoints.size(); i++)
    {
        // Skip if already added
        bool alreadyAdded = false;
        for (int j = 0; j < indices.size(); j++) {
            if (i == indices[j]) {
                alreadyAdded = true;
                break;
            }
        }
        if (alreadyAdded)
            continue;

        float shortestDist = std::numeric_limits<float>::max();
        for (int j = 0; j < indices.size(); j++)
        {
            glm::vec3 diff = this->allClippedPoints[i] - this->allClippedPoints[indices[j]];
            float dist2 = glm::dot(diff, diff);

            if (dist2 < shortestDist) {
                shortestDist = dist2;
            }
        }
        if (shortestDist > bestDist) {
            bestDist = shortestDist;
            bestIdx = i;
        }
    }

    indices.push_back(bestIdx);
}

void CollisionManifold::selectCollisionFace(GameObject& obj, const glm::vec3& normal) {
    glm::vec3 rotated = obj.invRotationMatrix * normal;
    glm::vec3 absN = glm::abs(rotated);

    this->selectedFace.clear();

    if (absN.x >= absN.y and absN.x >= absN.z) {
        for (const auto& face : (rotated.x > 0) ? obj.aabb.wFaces.maxX : obj.aabb.wFaces.minX) {
            this->selectedFace.push_back(face);
        }
    }

    else if (absN.y >= absN.x and absN.y >= absN.z) {
        for (const auto& face : (rotated.y > 0) ? obj.aabb.wFaces.maxY : obj.aabb.wFaces.minY) {
            this->selectedFace.push_back(face);
        }
    }
    else {
        for (const auto& face : (rotated.z > 0) ? obj.aabb.wFaces.maxZ : obj.aabb.wFaces.minZ) {
            this->selectedFace.push_back(face);
        }
    }
}

void CollisionManifold::createClippingPlanes(const std::vector<glm::vec3>& face, const glm::vec3& faceNormal) {
    this->clippingPlanes = {};
    this->clippingPlanes.resize(face.size());
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

    float epsilon = 1e-6f;
    if (t < -epsilon || t > 1.0f + epsilon) {
        outBool = false;
        return;
    }

    outPoint = v1 + t * lineDir;
    outBool = true;
}

bool CollisionManifold::isPointInsidePlane(const glm::vec3& point, const glm::vec3& planeNormal, const glm::vec3 planePoint) {
    if (glm::dot(planeNormal, point - planePoint) < 0.0f) {
        return true;
    }

    return false;
}

void CollisionManifold::clipPoints(std::vector<glm::vec3>& referenceFace, std::vector<glm::vec3>& incidentFace, glm::vec3& referenceFaceNormal) {

   createClippingPlanes(referenceFace, referenceFaceNormal);

   this->contactPoints = {};
   this->nextContactPoints = {};
   this->clippingStatus = {};

   this->clippedPoints.clear();
   this->clippedPoints.reserve(incidentFace.size()*2);

   for (int i = 0; i < incidentFace.size(); i++) {
      contactPoints[i] = incidentFace[i];
   }
   int counter = incidentFace.size(); 
   int counter2 = counter; 

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
          this->clippedPoints.push_back(contactPoints[i]);

   }
}

void CollisionManifold::createLocalCoordinates(Contact& contact) {
    glm::mat4& invM = contact.objA_ptr->invModelMatrix;

    for (int i = 0; i < contact.points.size(); i++) {
        glm::vec4 local = invM * glm::vec4(contact.points[i].globalCoord, 1.0f);
        contact.points[i].localCoord = glm::vec3(local);
    }
}

void CollisionManifold::contactPointReduction(Contact& contact) {
    glm::vec3 normal = contact.referenceFaceNormal;
    std::array<glm::vec3, 4>  finalGlobalPoints{};
    std::array<glm::vec3, 4>  finalLocalPoints{};

    // hitta den punkt (supportPoint) som är längst bort i x-led
    glm::vec3 supportPoint;
    const glm::vec3 direction = glm::vec3(1.0f, 0.0f, 0.0f);
    float maxDot = std::numeric_limits<float>::lowest();
    int supportPointIndex = 0;
    for (int i = 0; i < contact.points.size(); i++) {
        glm::vec3& point = contact.points[i].localCoord;

        float dotValue = glm::dot(point, direction);
        if (dotValue > maxDot) {
            maxDot = dotValue;
            supportPoint = point;
            supportPointIndex = i;
        }
    }
    finalGlobalPoints[0] = contact.points[supportPointIndex].globalCoord;
    finalLocalPoints[0] = contact.points[supportPointIndex].localCoord;

    // hitta den punkt som är längst bort från supportPointout.normal
    int farthestPointIndex = 0;
    glm::vec3 farthestPoint{};
    float maxDistance = std::numeric_limits<float>::lowest();
    for (int i = 0; i < contact.points.size(); i++) {
        glm::vec3& point = contact.points[i].localCoord;

        float distance = glm::distance2(supportPoint, point);
        if (distance > maxDistance) {
            maxDistance = distance;
            farthestPoint = point;
            farthestPointIndex = i;
        }
    }
    finalGlobalPoints[1] = contact.points[farthestPointIndex].globalCoord;
    finalLocalPoints[1] = contact.points[farthestPointIndex].localCoord;

    // hitta den punkt som bildar den största positiva triangeln med supportPoint och farthestPoint (pga crossproduct winding order)
    int trianglePointIndex = 0;
    float maxArea = std::numeric_limits<float>::lowest();
    for (int i = 0; i < contact.points.size(); i++) {
        glm::vec3& point = contact.points[i].localCoord;

        float area = 0.5f * glm::dot(glm::cross(supportPoint - farthestPoint, supportPoint - point), normal);
        if (area > maxArea) {
            maxArea = area;
            trianglePointIndex = i;
        }
    }
    finalGlobalPoints[2] = contact.points[trianglePointIndex].globalCoord;
    finalLocalPoints[2] = contact.points[trianglePointIndex].localCoord;

    // hitta den punkt som bildar den största negativa triangeln med supportPoint och farthestPoint (pga motsatt crossproduct winding order)
    int negTrianglePointIndex = 0;
    maxArea = std::numeric_limits<float>::max();
    for (int i = 0; i < contact.points.size(); i++) {
        glm::vec3& point = contact.points[i].localCoord;

        float area = 0.5f * glm::dot(glm::cross(supportPoint - farthestPoint, supportPoint - point), normal);
        if (area < 0 and area < maxArea) {
            maxArea = area;
            negTrianglePointIndex = i;
        }
    }
    finalGlobalPoints[3] = contact.points[negTrianglePointIndex].globalCoord;
    finalLocalPoints[3] = contact.points[negTrianglePointIndex].localCoord;

    for (int i = 0; i < 4; i++) {
        contact.points[i].globalCoord = finalGlobalPoints[i];
        contact.points[i].localCoord = finalLocalPoints[i];
    }
    contact.points.resize(4);
}

void CollisionManifold::computePenetrationDepth(Contact& contact) {
    for (int i = 0; i < contact.points.size(); i++)
        contact.points[i].depth = -glm::dot(contact.points[i].globalCoord - contact.referenceFace[0], contact.referenceFaceNormal);
}

void CollisionManifold::PreComputePointData(ContactPoint& cp, Contact& contact) {
    constexpr float restitutionThreshold = 0.1f; // Minsta hastighet för att restitution ska aktiveras
    constexpr float restitution = 0.1f; // exempelmaterial

    GameObject& objA = *contact.objA_ptr; 
    GameObject& objB = *contact.objB_ptr; 
    glm::vec3 normal = contact.normal; 

    glm::vec3 rA = cp.globalCoord - objA.position;
    float invMassA = objA.invMass;
    glm::mat3 invInertiaA = objA.inverseInertiaWorld;
    glm::vec3 linearVelocityA = objA.linearVelocity;
    glm::vec3 angularVelocityA = objA.angularVelocity;

    glm::vec3 rB; 
    float invMassB;
    glm::mat3 invInertiaB; 
    glm::vec3 linearVelocityB;
    glm::vec3 angularVelocityB;

    if (contact.objB_ptr == nullptr) {
        rB = glm::vec3(0.0f);
        invMassB = 0.0f;
        invInertiaB = glm::mat3(0.0f);
        linearVelocityB = glm::vec3(0.0f);
        angularVelocityB = glm::vec3(0.0f);
    }
    else {
        rB = cp.globalCoord - objB.position; 
        invMassB = objB.invMass; 
        invInertiaB = objB.inverseInertiaWorld; 
        linearVelocityB = objB.linearVelocity; 
        angularVelocityB = objB.angularVelocity; 
    }

    // pre-calculate rA, rB, EffectiveMass
    cp.rA = rA;
    cp.rB = rB;

    glm::vec3 rA_cross_n = glm::cross(rA, normal);
    glm::vec3 rB_cross_n = glm::cross(rB, normal);
    cp.m_eff = 1.0f / (invMassA + invMassB +
        glm::dot(rA_cross_n, invInertiaA * rA_cross_n) +
        glm::dot(rB_cross_n, invInertiaB * rB_cross_n));

    // Räkna ut den relativa hastigheten vid kontaktpunkten, baserat på de aktuella kropparnas tillstånd
    glm::vec3 relativeVelocity = 
        (linearVelocityB + glm::cross(angularVelocityB, rB)) -
        (linearVelocityA + glm::cross(angularVelocityA, rA));

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

    glm::vec3 rA_t1 = glm::cross(rA, t1);
    glm::vec3 rB_t1 = glm::cross(rB, t1);
    glm::vec3 rA_t2 = glm::cross(rA, t2);
    glm::vec3 rB_t2 = glm::cross(rB, t2);

    glm::vec3 invIA_rA_t1 = invInertiaA * rA_t1;
    glm::vec3 invIB_rB_t1 = invInertiaB * rB_t1;
    glm::vec3 invIA_rA_t2 = invInertiaA * rA_t2;
    glm::vec3 invIB_rB_t2 = invInertiaB * rB_t2;

    // Beräkna effektiv massa längs cp.t1 och cp.t2 
    float k_t1 = (invMassA + invMassB) + glm::dot(rA_t1, invIA_rA_t1) + glm::dot(rB_t1, invIB_rB_t1);
    cp.invMassT1 = 1.0f / k_t1;
    float k_t2 = (invMassA + invMassB) + glm::dot(rA_t2, invIA_rA_t2) + glm::dot(rB_t2, invIB_rB_t2);
    cp.invMassT2 = 1.0f / k_t2;

    cp.invMassTwist = 1.0f / (glm::dot(normal, invInertiaA * normal) + glm::dot(normal, invInertiaB * normal));
}

size_t CollisionManifold::generateKey(int idA, int idB) {
    return (uint64_t)std::min(idA, idB) << 32 | std::max(idA, idB);
}

void CollisionManifold::integrateContact(std::unordered_map<size_t, Contact>& contactCache, Contact& contact) {
    for (int i = 0; i < contact.points.size(); i++) {
        PreComputePointData(contact.points[i], contact);
        contact.points[i].wasUsedThisFrame = true;
    }

    auto it = contactCache.find(contact.hashKey);

    // ingen matchande cached contact
    if (it == contactCache.end()) {
        contactCache[contact.hashKey] = contact;
        return;
    }

    // matcha nya punkter med gamla punkter eftersom finalContact inte matchar med en cached contact
    Contact& cachedContact = it->second;
    cachedContact.wasUsedThisFrame = true;

    std::array<bool, 4> matchedFinalPoints{ false };
    std::array<bool, 4> matchedCachedPoints{ false };
    glm::vec3& referenceFaceNormal = contact.referenceFaceNormal;
    std::vector<glm::vec3>& referenceFace = contact.referenceFace;

    // -------------- behöver ändras till att vara en faktor av objektens storlekar -------------
    const float threshold = 0.5f;

    // iterera över alla nya contact points och se om någon är nära en existerande
    for (int i = 0; i < contact.points.size(); i++) {
        ContactPoint& newPoint = contact.points[i];
        for (int j = 0; j < cachedContact.points.size(); j++) {

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
                newPoint.accumulatedImpulse = glm::dot(oldImpulseWorld, contact.normal);
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

    // sphere vs cube
    if (contact.points.size() == 1) {
        contactCache[contact.hashKey] = contact;
        return;
    }

    // sphere vs tri mesh
    if (contact.objA_ptr->colliderType == ColliderType::SPHERE and contact.objB_ptr == nullptr) {
        contactCache[contact.hashKey] = contact;
        return;
    }

    // fyll på med cachade punkter som inte blivit matchade med en ny punkt
    if (contact.points.size() < 4) {
        for (int i = 0; i < cachedContact.points.size(); i++) {
            if (matchedCachedPoints[i] == true)
                continue;

            ContactPoint& cachedPoint = cachedContact.points[i];
            if ((glm::length((cachedPoint.globalCoord - referenceFace[0]) - glm::dot(cachedPoint.globalCoord - referenceFace[0], referenceFaceNormal) * referenceFaceNormal) > threshold) || (glm::abs(glm::dot(referenceFaceNormal, cachedPoint.globalCoord - referenceFace[0]) > threshold))) {
                continue;
            }

            PreComputePointData(cachedPoint, contact);
            cachedPoint.wasUsedThisFrame = true;
            contact.points.push_back(cachedPoint);
            
            if (contact.points.size() >= 4) {
                break;
            }
        }
    }

    contactCache[contact.hashKey] = contact;
}