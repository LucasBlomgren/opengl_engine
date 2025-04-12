#include "collision_manifold.h"

std::array<glm::vec3, 4> selectCollisionFace(GameObject& obj, const glm::vec3& normal)
{
    glm::mat4 modelMatrix = obj.modelMatrix;
    glm::mat3 rotationMatrix = glm::mat3(modelMatrix);
    glm::mat3 inverseRotation = glm::transpose(rotationMatrix);

    // rotera normalen till objektets lokala koordinatsystem
    glm::vec3 rotatedNormal = inverseRotation * normal;

    glm::vec3 absNormal = glm::abs(rotatedNormal);
    std::array<glm::vec3, 4> selectedFace;

    // välj vilket face som är mest parallellt med normalen
    if (absNormal.x >= absNormal.y && absNormal.x >= absNormal.z) {
        if (rotatedNormal.x > 0)
            selectedFace = obj.AABB.defaultFaces.maxX;
        else
            selectedFace = obj.AABB.defaultFaces.minX;
    }
    else if (absNormal.y >= absNormal.x && absNormal.y >= absNormal.z) {
        if (rotatedNormal.y > 0)
            selectedFace = obj.AABB.defaultFaces.maxY;
        else
            selectedFace = obj.AABB.defaultFaces.minY;
    }
    else {
        if (rotatedNormal.z > 0)
            selectedFace = obj.AABB.defaultFaces.maxZ;
        else
            selectedFace = obj.AABB.defaultFaces.minZ;
    }

    // transformera face till globala koordinater
    for (glm::vec3& point : selectedFace) {
        glm::vec4 transformedPoint = modelMatrix * glm::vec4(point, 1.0f);
        point = glm::vec3(transformedPoint);
    }

    return selectedFace;
}

std::array<Plane, 4> createClippingPlanes(const std::array<glm::vec3,4>& face, const glm::vec3& faceNormal)
{
    std::array<Plane, 4> clippingPlanes{};

    // create clipping planes
    for (int i = 0; i < face.size(); i++) {
        glm::vec3 edge = face[i] - face[(i + 1) % face.size()];
        glm::vec3 planeNormal = (glm::normalize(glm::cross(faceNormal, edge)));

        clippingPlanes[i] = Plane(planeNormal, face[i]);
    }

    return clippingPlanes;
}

glm::vec3 getIntersectionPoint(const glm::vec3& v1, const glm::vec3& v2, const Plane& plane) 
{
    glm::vec3 lineDir = v2 - v1;
    float denominator = glm::dot(plane.normal, lineDir);
    float t = glm::dot(plane.normal, plane.point - v1) / denominator;
    glm::vec3 intersectionPoint = v1 + t * lineDir;

    return intersectionPoint;
}

bool isPointInsidePlane(const glm::vec3& point, const glm::vec3& planeNormal, const glm::vec3 planePoint)
{
    if (glm::dot(planeNormal, point - planePoint) < 0)
        return true;

    return false;
}

void createContactPoints(InitialContact& initialContact)
{
    std::array<glm::vec3, 4> referenceFace = initialContact.referenceFace; 
    std::array<glm::vec3, 4> incidentFace = initialContact.incidentFace;
    glm::vec3 referenceFaceNormal = initialContact.referenceFaceNormal; 

    std::array<Plane, 4> clippingPlanes = createClippingPlanes(referenceFace, referenceFaceNormal);
    //drawClippingPlanes(shader, VAO, clippingPlanes, referenceFace);
    
    std::array<glm::vec3, 8> contactPoints{};
    for (int i = 0; i < incidentFace.size(); i++) {
        contactPoints[i] = incidentFace[i];
    }
    std::array<glm::vec3, 8> nextContactPoints{};
    std::array<bool, 8> clippingStatus{};
    int counter = 4;
    int counter2 = 4;

    // clip alla punkter mot alla plan
    for (const Plane& plane : clippingPlanes) {
        for (int i = 0; i < counter; i++) {
            clippingStatus[i] = isPointInsidePlane(contactPoints[i], plane.normal, plane.point);  // håll koll på vilka punkter som är innanför detta specifika plan
        }

        counter2 = counter;
        counter = 0;

        // polygon clipping algorithm
        for (int i = 0; i < counter2; i++) {
            int nextIndex = (i + 1) % counter2;

            if (!clippingStatus[i] and clippingStatus[nextIndex]) {
                nextContactPoints[counter++] = getIntersectionPoint(contactPoints[i], contactPoints[nextIndex], plane);
                nextContactPoints[counter++] = contactPoints[nextIndex];
            }
            else if (clippingStatus[i] and clippingStatus[nextIndex]) {
                nextContactPoints[counter++] = contactPoints[nextIndex];
            }
            else if (clippingStatus[i] and !clippingStatus[nextIndex]) {
                nextContactPoints[counter++] = getIntersectionPoint(contactPoints[i], contactPoints[nextIndex], plane);
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

void createLocalCoordinates(InitialContact& initialContact)
{
    glm::mat4 invM = glm::inverse(initialContact.objA_ptr->modelMatrix);
    for (int i = 0; i < initialContact.counter; i++) {
        glm::vec4 local = invM * glm::vec4(initialContact.globalCoords[i], 1.0f);
        initialContact.localCoords[i] = glm::vec3(local);
    }
}

void contactPointReduction(InitialContact& contactPoints)
{
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
        if (area < 0 && area < maxArea) {
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

void computePenetrationDepth(InitialContact& initialContact) {
    glm::vec3 referenceFacePoint = initialContact.referenceFace[0];
    glm::vec3 referenceFaceNormal = initialContact.referenceFaceNormal;

    for (int i = 0; i < initialContact.counter; i++)
        initialContact.pointDepths[i] = -glm::dot(initialContact.globalCoords[i] - referenceFacePoint, referenceFaceNormal);
}

void PreComputeContactData(Contact& contact) {

    GameObject& objA = *contact.objA_ptr;
    GameObject& objB = *contact.objB_ptr;

    // pre-calculate rA, rB, EffectiveMass
    for (int j = 0; j < contact.counter; j++) {
        ContactPoint& cp = contact.points[j];

        cp.rA = cp.globalCoord - objA.position;
        cp.rB = cp.globalCoord - objB.position;

        cp.m_eff = 1.0f / (objA.invMass + objB.invMass +
            glm::dot(glm::cross(cp.rA, contact.normal), objA.inverseInertia * glm::cross(cp.rA, contact.normal)) +
            glm::dot(glm::cross(cp.rB, contact.normal), objB.inverseInertia * glm::cross(cp.rB, contact.normal)));

        // Räkna ut den relativa hastigheten vid kontaktpunkten, baserat på de aktuella kropparnas tillstånd
        glm::vec3 relativeVelocity = (objB.linearVelocity + glm::cross(objB.angularVelocity, cp.rB)) -
            (objA.linearVelocity + glm::cross(objA.angularVelocity, cp.rA));

        // Dela upp i normal- och tangentiell komponent
        glm::vec3 v_normal = glm::dot(relativeVelocity, contact.normal) * contact.normal;
        glm::vec3 v_tangent = relativeVelocity - v_normal;
        float vtMagnitude = glm::length(v_tangent);

        glm::vec3 t1, t2;
        float epsilon = 1e-3f;

        // Om den tangentiella hastigheten är signifikant, använd den för att bestämma t1
        if (vtMagnitude > epsilon) {
            t1 = glm::normalize(v_tangent);
            // t2 är vinkelrät mot både kontaktnormalen och t1
            t2 = glm::normalize(glm::cross(contact.normal, t1));
        }
        else {
            // Om relativeVelocity är nära noll, välj en standardreferens som inte är parallell med kontaktnormalen
            glm::vec3 reference = (fabs(contact.normal.y) < 0.9f) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
            t1 = glm::normalize(glm::cross(contact.normal, reference));
            t2 = glm::normalize(glm::cross(contact.normal, t1));
        }
        cp.t1 = t1;
        cp.t2 = t2;

        const float restitutionThreshold = 0.5f; // Minsta hastighet för att restitution ska aktiveras
        float restitution = 0.2f; // exempelmaterial
        float normalVelocity = glm::dot(relativeVelocity, contact.normal);
        if (normalVelocity < -restitutionThreshold) {
            cp.targetBounceVelocity = -restitution * normalVelocity;
        }
        else {
            cp.targetBounceVelocity = 0.0f;
        }
    }
}

size_t generateKey(int idA, int idB) {
    if (idA > idB) std::swap(idA, idB);
    return static_cast<size_t>(idA) * 500 + static_cast<size_t>(idB);
}

void integrateContact(std::unordered_map<size_t, Contact>& contactCache, InitialContact& initialContact, Contact& finalContact)
{
    // kopiera över punkterna från initialContact till finalContact
    for (int i = 0; i < initialContact.counter; i++) {
        finalContact.points[i].globalCoord = initialContact.globalCoords[i];
        finalContact.points[i].localCoord = initialContact.localCoords[i];
        finalContact.points[i].depth = initialContact.pointDepths[i];
    }
    finalContact.counter = initialContact.counter;

    PreComputeContactData(finalContact);

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
    const float threshold = 0.2f;

    // iterera över alla nya contact points och se om någon är nära en existerande
    for (int i = 0; i < finalContact.counter; i++) {
        ContactPoint& newPoint = finalContact.points[i];
        for (int j = 0; j < cachedContact.counter; j++) {

            if (matchedFinalPoints[i]) break;
            if (matchedCachedPoints[j]) continue;

            ContactPoint& cachedPoint = cachedContact.points[j];

            // nya punkten är nära en cached punkt = warm start
            if (glm::distance(newPoint.localCoord, cachedPoint.localCoord) < threshold) {
                newPoint.accumulatedImpulse = cachedPoint.accumulatedImpulse;
                newPoint.accumulatedFrictionImpulse1 = cachedPoint.accumulatedFrictionImpulse1;
                newPoint.accumulatedFrictionImpulse2 = cachedPoint.accumulatedFrictionImpulse2;
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

            finalContact.points[finalContact.counter++] = cachedPoint;
            if (finalContact.counter >= 4) {
                break;
            }
        }
    }

    contactCache[key] = finalContact;
}

Contact createContact(std::unordered_map<size_t, Contact>& contactCache, GameObject& objA, GameObject& objB, glm::vec3 normal, int& collisionNormalOwner)
{
    std::array<glm::vec3, 4> referenceFace;
    std::array<glm::vec3, 4> incidentFace;

    // välj vilken som är referensface och incidentface
    if (collisionNormalOwner == 0) {
        referenceFace = selectCollisionFace(objA, normal); 
        incidentFace = selectCollisionFace(objB, -normal);
    }
    else {
        referenceFace = selectCollisionFace(objB, -normal);
        incidentFace = selectCollisionFace(objA, normal);
    }

    // räkna ut normalen för referensface
    glm::vec3 edgeA = referenceFace[0] - referenceFace[1];
    glm::vec3 edgeB = referenceFace[0] - referenceFace[2];
    glm::vec3 referenceFaceNormal = glm::normalize(glm::cross(edgeA, edgeB));

    InitialContact initialContact{ &objA, &objB, referenceFace, incidentFace, referenceFaceNormal };

    createContactPoints(initialContact);
    createLocalCoordinates(initialContact);     
    if (initialContact.counter > 4) { contactPointReduction(initialContact); }
    computePenetrationDepth(initialContact);

    Contact finalContact{ &objA, &objB, normal };
    integrateContact(contactCache, initialContact, finalContact);

    return finalContact;
}

void drawClippingPlanes(const Shader& shader, unsigned int& VAO, const std::array<Plane, 4>& clippingPlanes, const std::array<glm::vec3, 4>& referenceFace)
{
    for (int i = 0; i < clippingPlanes.size(); i++) {
        Plane plane = clippingPlanes[i];
        glm::vec3 color = glm::vec3(1, 0, 0);
        if (i == 1) color = glm::vec3(0, 1, 0);
        if (i == 2) color = glm::vec3(0, 0, 1);
        if (i == 3) color = glm::vec3(1, 1, 0);
        glm::vec3 edgeMidpoint = (referenceFace[i] + referenceFace[(i + 1) % referenceFace.size()]) / 2.0f;
        glm::vec3 lineEnd = edgeMidpoint + plane.normal * 10.0f; // Extend the line in the direction of the normal

        drawLine(shader, VAO, edgeMidpoint, lineEnd, color); // Call the drawLine function to draw the line
    }
}