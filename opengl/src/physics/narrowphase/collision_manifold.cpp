#include "pch.h"
#include "collision_manifold.h"

//=============================================
// Box-Box collision
//=============================================
void CollisionManifold::boxBox(Contact& contact, std::unordered_map<size_t, Contact>& cache, SAT::Result& satResult) {
    ContactRuntime& rt = contact.runtimeData;
    Collider* colliderA = rt.colliderA;
    Collider* colliderB = rt.colliderB;

    const float linearSlop = 0.001f;
    const float k_tol = 0.1f * linearSlop;
    contact.objBisReference = (satResult.separationB > satResult.separationA + k_tol);

    if (contact.objBisReference) {
        selectOOBBCollisionRefFaceAndNormal(colliderB, colliderB->pose, -satResult.normal, contact.referenceFace, contact.referenceFaceNormal);
        selectOOBBCollisionIncidentFace(colliderA, colliderA->pose, satResult.normal, contact.incidentFace);
    }
    else {
        selectOOBBCollisionRefFaceAndNormal(colliderA, colliderA->pose, satResult.normal, contact.referenceFace, contact.referenceFaceNormal);
        selectOOBBCollisionIncidentFace(colliderB, colliderB->pose, -satResult.normal, contact.incidentFace);
    }

    clipPoints(contact.referenceFace, contact.incidentFace, 4, contact.referenceFaceNormal);

    // create contact points from clipped points
    Transform* root = nullptr;
    if (contact.objBisReference && contact.partnerTypeB == ContactPartnerType::RigidBody)
        root = contact.runtimeData.bodyRootB;
    else
        root = contact.runtimeData.bodyRootA;

    contact.points.resize(clippedPoints.size());
    for (int i = 0; i < clippedPoints.size(); ++i) {
        auto& cp = contact.points[i];
        cp.worldPos = clippedPoints[i];
        cp.depth = -glm::dot(clippedPoints[i] - contact.referenceFace[0], contact.referenceFaceNormal);
        cp.localPos = root->worldToLocalPoint(cp.worldPos);
    }

    // if more than 4 contact points, perform contact point reduction to improve solver stability
    if (contact.points.size() > 4) {
        contactPointReduction(contact);
    }

    contact.hashKey = generateKey(colliderA->id, colliderB->id);
    contact.normal = satResult.normal;

    integrateContact(cache, contact);
}

//==================================================
// Box-Sphere collision
//==================================================
void CollisionManifold::boxSphere(Contact& contact, std::unordered_map<size_t, Contact>& cache, SAT::Result& satResult) {
    contact.normal = satResult.normal;
    contact.points.resize(1);
    contact.points[0].worldPos = satResult.point; // point is the closest point on the cuboid to the sphere
    contact.points[0].depth = satResult.depth;    // penetration depth

    Transform* root = nullptr;
    if (contact.objBisReference && contact.partnerTypeB == ContactPartnerType::RigidBody)
        root = contact.runtimeData.bodyRootB;
    else
        root = contact.runtimeData.bodyRootA;

    contact.points[0].localPos = root->worldToLocalPoint(contact.points[0].worldPos);

    contact.hashKey = generateKey(contact.runtimeData.colliderA->id, contact.runtimeData.colliderB->id);
    integrateContact(cache, contact);
}

//===================================================
// Box-Mesh collision
//===================================================
void CollisionManifold::boxMesh(Contact& contact, std::unordered_map<size_t, Contact>& cache, std::vector<SAT::Result>& allResults) {
    ContactRuntime& rt = contact.runtimeData;
    Collider* colliderA = rt.colliderA;

    this->allClippedPoints.clear();
    this->allClippedPoints.reserve(8 * allResults.size());

    // set ref face for contact (for penetration depth calculation etc.)
    selectOOBBCollisionRefFaceAndNormal(colliderA, colliderA->pose, allResults[0].normal, contact.referenceFace, contact.referenceFaceNormal);

    allPointsDepthsScratch.reserve(allResults.size() * 3); // 3 points per triangle 
    allPointsDepthsScratch.clear();

    for (const SAT::Result& satResult : allResults)
    {
        selectOOBBCollisionRefFaceAndNormal(colliderA, colliderA->pose, satResult.normal, contact.referenceFace, contact.referenceFaceNormal);

        contact.incidentFace = {
            satResult.tri_ptr->vertices[0],
            satResult.tri_ptr->vertices[1],
            satResult.tri_ptr->vertices[2],
            glm::vec3(0.0f)
        };

        clipPoints(contact.referenceFace, contact.incidentFace, 3, contact.referenceFaceNormal);
        computePenetrationDepth(clippedPoints, contact.referenceFace, contact.referenceFaceNormal, allPointsDepthsScratch);

        // save clipped points for furthest point selection
        this->allClippedPoints.insert(this->allClippedPoints.end(), this->clippedPoints.begin(), this->clippedPoints.end());
    }

    // --- pick furthest points from all clipped points ---
    pickFourFurthestPoints();

    Transform* root = contact.runtimeData.bodyRootA; // terrain => bodyA

    contact.points.resize(this->furthestPoints.size());
    for (int i = 0; i < this->furthestPoints.size(); ++i) {
        auto& cp = contact.points[i];
        cp.worldPos = this->furthestPoints[i];
        cp.depth = allPointsDepthsScratch[indices[i]];
        cp.localPos = root->worldToLocalPoint(cp.worldPos);
    }

    contact.hashKey = generateKey(colliderA->id, allResults[0].tri_ptr->id);
    integrateContact(cache, contact);
}

//===================================================
// Sphere-Sphere collision
//===================================================
void CollisionManifold::sphereSphere(Contact& contact, std::unordered_map<size_t, Contact>& cache, SAT::Result& satResult) {
    ContactRuntime& rt = contact.runtimeData;
    Collider* colliderA = rt.colliderA;
    Collider* colliderB = rt.colliderB;

    contact.normal = satResult.normal;
    contact.points.resize(1);
    contact.points[0].worldPos = satResult.point; // point is the closest point on the sphere to the sphere
    contact.points[0].depth = satResult.depth;    // penetration depth

    Transform* root = nullptr;
    if (contact.objBisReference && contact.partnerTypeB == ContactPartnerType::RigidBody)
        root = contact.runtimeData.bodyRootB;
    else
        root = contact.runtimeData.bodyRootA;

    contact.points[0].localPos = root->worldToLocalPoint(contact.points[0].worldPos);

    contact.hashKey = generateKey(colliderA->id, colliderB->id);
    integrateContact(cache, contact);
}

//===================================================
// Sphere-Mesh collision
//===================================================
void CollisionManifold::sphereMesh(Contact& contact, std::unordered_map<size_t, Contact>& cache, std::vector<SAT::Result>& allResults) {
    ContactRuntime& rt = contact.runtimeData;
    Collider* colliderA = rt.colliderA;

    this->allClippedPoints.clear();

    this->allClippedPoints.resize(allResults.size());
    for (int i = 0; i < allResults.size(); i++) {
        this->allClippedPoints[i] = allResults[i].point;
    }

    pickFourFurthestPoints();

    // create contact points from furthest points
    contact.points.resize(this->furthestPoints.size());
    for (int i = 0; i < this->furthestPoints.size(); i++) {
        contact.points[i].worldPos = this->furthestPoints[i];
        contact.points[i].localPos = contact.runtimeData.bodyRootA->worldToLocalPoint(contact.points[i].worldPos);
    }

    // add depth for each point
    contact.points[0].depth = allResults[0].depth;
    for (int i = 1; i < this->furthestPoints.size(); i++) {
        contact.points[i].depth = allResults[indices[i]].depth;
    }

    contact.hashKey = generateKey(colliderA->id, allResults[0].tri_ptr->id);
    integrateContact(cache, contact);
}

//===================================================
// Helper function for contact generation
//===================================================
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

    furthestPoints.resize(indices.size());
    for (int k = 0; k < indices.size(); ++k)
        furthestPoints[k] = allClippedPoints[indices[k]];
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

//============================================================================
//  Select OOBB Reference face & normal for clipping based on contact normal
//============================================================================
void CollisionManifold::selectOOBBCollisionRefFaceAndNormal(const Collider* collider, ColliderPose& pose, const glm::vec3& normal, std::array<glm::vec3, 4>& outFace, glm::vec3& outNormal) {
    pose.ensureInvRotationMatrix();
    pose.ensureModelMatrix();

    const OOBB& box = std::get<OOBB>(collider->shape);

    // rotate the contact normal into the box's local space
    glm::vec3 rotated = pose.invRotationMatrix * normal;
    glm::vec3 absN = glm::abs(rotated);

    // choose the face whose normal is most aligned with the contact normal (in local space)
    std::array<glm::vec3, 4> localFace;

    if (absN.x >= absN.y && absN.x >= absN.z) {
        localFace = (rotated.x > 0) ? box.getLocalFace(FaceId::MaxX)
            : box.getLocalFace(FaceId::MinX);
    }
    else if (absN.y >= absN.x && absN.y >= absN.z) {
        localFace = (rotated.y > 0) ? box.getLocalFace(FaceId::MaxY)
            : box.getLocalFace(FaceId::MinY);
    }
    else {
        localFace = (rotated.z > 0) ? box.getLocalFace(FaceId::MaxZ)
            : box.getLocalFace(FaceId::MinZ);
    }

    // transform the four local face vertices to world space using the box's model matrix (which includes rotation and scale)
    glm::mat3 M3 = glm::mat3(pose.modelMatrix);
    glm::vec3 T3 = glm::vec3(pose.modelMatrix[3]);

    for (int i = 0; i < localFace.size(); i++) {
        outFace[i] = M3 * localFace[i] + T3;
    }

    // compute the face normal in world space using the cross product of two edges of the face
    glm::vec3 e0 = outFace[1] - outFace[0];
    glm::vec3 e1 = outFace[2] - outFace[0];
    outNormal = glm::normalize(glm::cross(e0, e1));
}

//=================================================================
// Select OOBB Incident face for clipping based on contact normal
//=================================================================
void CollisionManifold::selectOOBBCollisionIncidentFace(const Collider* collider, ColliderPose& pose, const glm::vec3& normal, std::array<glm::vec3, 4>& outFace) {
    pose.ensureInvRotationMatrix();
    pose.ensureModelMatrix();

    const OOBB& box = std::get<OOBB>(collider->shape);

    // rotate the contact normal into the box's local space
    glm::vec3 rotated = pose.invRotationMatrix * normal;
    glm::vec3 absN = glm::abs(rotated);

    // choose the face whose normal is most aligned with the contact normal (in local space)
    std::array<glm::vec3, 4> localFace;

    if (absN.x >= absN.y && absN.x >= absN.z) {
        localFace = (rotated.x > 0) ? box.getLocalFace(FaceId::MaxX)
            : box.getLocalFace(FaceId::MinX);
    }
    else if (absN.y >= absN.x && absN.y >= absN.z) {
        localFace = (rotated.y > 0) ? box.getLocalFace(FaceId::MaxY)
            : box.getLocalFace(FaceId::MinY);
    }
    else {
        localFace = (rotated.z > 0) ? box.getLocalFace(FaceId::MaxZ)
            : box.getLocalFace(FaceId::MinZ);
    }

    // transform the four local face vertices to world space using the box's model matrix (which includes rotation and scale)
    glm::mat3 M3 = glm::mat3(pose.modelMatrix);
    glm::vec3 T3 = glm::vec3(pose.modelMatrix[3]);

    for (int i = 0; i < localFace.size(); i++) {
        outFace[i] = M3 * localFace[i] + T3;
    }
}

//===================================================================================
// Sutherland-Hodgman clipping of incident face against reference face's side planes
//===================================================================================
void CollisionManifold::clipPoints(const std::array<glm::vec3, 4>& referenceFace, const std::array<glm::vec3, 4>& incidentFace, int incidentCount, const glm::vec3& referenceFaceNormal) {
    createClippingPlanes(referenceFace, referenceFaceNormal);

    this->clippedPoints.clear();
    this->clippedPoints.reserve(incidentCount * 2);

    for (int i = 0; i < incidentCount; i++) {
        contactPoints[i] = incidentFace[i];
    }
    int counter = incidentCount;
    int counter2 = counter;

    // clip all points against each plane one by one, this will keep only the points that are inside all planes (i.e. inside the reference face)
    for (const Plane& plane : this->clippingPlanes) {
        for (int i = 0; i < counter; i++) {
            clippingStatus[i] = isPointInsidePlane(contactPoints[i], plane.normal, plane.point, 1e-7f);  // remember which points are inside/outside the plane for the clipping algorithm
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
        std::swap(contactPoints, nextContactPoints);
    }

    // keep the points that are inside the reference face
    for (int i = 0; i < counter; i++) {
        if (isPointInsidePlane(contactPoints[i], referenceFaceNormal, referenceFace[0], 1e-2f))
            this->clippedPoints.push_back(contactPoints[i]);
    }
}

//============================================================================
// Sutherland-Hodgman clipping helper functions
//============================================================================
void CollisionManifold::createClippingPlanes(const std::array<glm::vec3, 4>& face, const glm::vec3& faceNormal)
{
    // #TODO: Använda FaceCenter för point on plane i stället för face[i]. Pga precision.
    // T.ex: float depth = dot(n, contactPoint - planePoint); (depth per contact point)
    // Face center är mer stabilt än ett hörn för det swappar inte mellan frames.
    // Face center kommer också vara on average närmare fler antal contact points än ett hörn.
    // Detta minskar risken för jitter vid kontaktpunkter nära plan.

    // create clipping planes
    for (int i = 0; i < face.size(); i++) {
        glm::vec3 edge = face[i] - face[(i + 1) % face.size()];
        glm::vec3 planeNormal = glm::cross(faceNormal, edge);

        this->clippingPlanes[i] = Plane(planeNormal, face[i]);
    }
}

void CollisionManifold::getIntersectionPoint(const glm::vec3& v1, const glm::vec3& v2, const Plane& plane, glm::vec3& outPoint, bool& outBool) {
    glm::vec3 lineDir = v2 - v1;
    float denominator = glm::dot(plane.normal, lineDir);

    if (std::abs(denominator) < 1e-6f) {
        // the line is parallel to the plane, so we consider it as no valid intersection for clipping purposes
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

bool CollisionManifold::isPointInsidePlane(const glm::vec3& point, const glm::vec3& planeNormal, const glm::vec3& planePoint, const float tolerance) {
    return glm::dot(planeNormal, point - planePoint) <= tolerance;
}

//==========================================================================================
// Contact point reduction to max 4 points for box-box collisions with many clipped points
//==========================================================================================
void CollisionManifold::contactPointReduction(Contact& contact) {
    glm::vec3 normal = contact.referenceFaceNormal;
    std::array<glm::vec3, 4>  finalGlobalPoints{};
    std::array<glm::vec3, 4>  finalLocalPoints{};

    // find the point (supportPoint) that is furthest in the x direction (arbitrary choice to have a deterministic point to build the rest of the points around)
    glm::vec3 supportPoint;
    const glm::vec3 direction = glm::vec3(1.0f, 0.0f, 0.0f);
    float maxDot = std::numeric_limits<float>::lowest();
    int supportPointIndex = 0;
    for (int i = 0; i < contact.points.size(); i++) {
        glm::vec3& point = contact.points[i].localPos;

        float dotValue = glm::dot(point, direction);
        if (dotValue > maxDot) {
            maxDot = dotValue;
            supportPoint = point;
            supportPointIndex = i;
        }
    }
    finalGlobalPoints[0] = contact.points[supportPointIndex].worldPos;
    finalLocalPoints[0] = contact.points[supportPointIndex].localPos;

    // find the point that is furthest from supportPoint in the direction of the contact normal
    int farthestPointIndex = 0;
    glm::vec3 farthestPoint{};
    float maxDistance = std::numeric_limits<float>::lowest();
    for (int i = 0; i < contact.points.size(); i++) {
        glm::vec3& point = contact.points[i].localPos;

        float distance = glm::distance2(supportPoint, point);
        if (distance > maxDistance) {
            maxDistance = distance;
            farthestPoint = point;
            farthestPointIndex = i;
        }
    }
    finalGlobalPoints[1] = contact.points[farthestPointIndex].worldPos;
    finalLocalPoints[1] = contact.points[farthestPointIndex].localPos;

    // find the point that forms the triangle with the largest positive area with supportPoint and farthestPoint, using the contact normal to determine the sign of the area
    int trianglePointIndex = 0;
    float maxArea = std::numeric_limits<float>::lowest();
    for (int i = 0; i < contact.points.size(); i++) {
        glm::vec3& point = contact.points[i].localPos;

        float area = 0.5f * glm::dot(glm::cross(supportPoint - farthestPoint, supportPoint - point), normal);
        if (area > maxArea) {
            maxArea = area;
            trianglePointIndex = i;
        }
    }
    finalGlobalPoints[2] = contact.points[trianglePointIndex].worldPos;
    finalLocalPoints[2] = contact.points[trianglePointIndex].localPos;

    // find the point that forms the triangle with the largest negative area with supportPoint and farthestPoint, using the contact normal to determine the sign of the area
    int negTrianglePointIndex = 0;
    maxArea = std::numeric_limits<float>::max();
    for (int i = 0; i < contact.points.size(); i++) {
        glm::vec3& point = contact.points[i].localPos;

        float area = 0.5f * glm::dot(glm::cross(supportPoint - farthestPoint, supportPoint - point), normal);
        if (area < 0 and area < maxArea) {
            maxArea = area;
            negTrianglePointIndex = i;
        }
    }
    finalGlobalPoints[3] = contact.points[negTrianglePointIndex].worldPos;
    finalLocalPoints[3] = contact.points[negTrianglePointIndex].localPos;

    for (int i = 0; i < 4; i++) {
        contact.points[i].worldPos = finalGlobalPoints[i];
        contact.points[i].localPos = finalLocalPoints[i];
    }
    contact.points.resize(4);
}

void CollisionManifold::computePenetrationDepth(std::vector<glm::vec3>& points, std::array<glm::vec3, 4>& refFace, glm::vec3& refFaceNormal, std::vector<float>& out) {
    for (int i = 0; i < points.size(); i++)
        out.push_back(-glm::dot(points[i] - refFace[0], refFaceNormal));
}

//=====================================================
//  Pre-compute data for solver
//=====================================================
void CollisionManifold::PreComputePointData(ContactPoint& cp, Contact& contact) {
    constexpr float restitutionThreshold = 0.2f; // smallest normal velocity to allow restitution (bounce)
    float restitution = 0.0f; // example material

    glm::vec3& normal = contact.normal;

    ContactRuntime& rt = contact.runtimeData;
    RigidBody* bodyA = rt.bodyA;
    RigidBody* bodyB = rt.bodyB;
    Transform* tA = rt.bodyRootA;
    Transform* tB = rt.bodyRootB;

    glm::vec3 rA;
    glm::vec3 rB;
    float invMassA;
    float invMassB;
    glm::mat3& invInertiaA = contact.invInertiaA;
    glm::mat3& invInertiaB = contact.invInertiaB;
    glm::vec3 linearVelocityA;
    glm::vec3 linearVelocityB;
    glm::vec3 angularVelocityA;
    glm::vec3 angularVelocityB;

    // bodyA solver response behavior
    if (contact.noSolverResponseA) {
        invMassA = 0.0f;
        invInertiaA = glm::mat3(0.0f);
    }
    else {
        invMassA = bodyA->invMass;
        invInertiaA = bodyA->invInertiaWorld;
    }
    // bodyA motion behavior
    if (contact.contributesMotionA) {
        rA = cp.worldPos - tA->position; // #TODO: ska vara rA = contactPoint - body.comWorld;
        linearVelocityA = bodyA->linearVelocity;
        angularVelocityA = bodyA->angularVelocity;
    }
    else {
        rA = glm::vec3(0.0f);
        linearVelocityA = glm::vec3(0.0f);
        angularVelocityA = glm::vec3(0.0f);
    }

    // bodyB solver response behavior
    if (contact.partnerTypeB == ContactPartnerType::Terrain || contact.noSolverResponseB) {
        invMassB = 0.0f;
        invInertiaB = glm::mat3(0.0f);
    }
    else {
        invMassB = bodyB->invMass;
        invInertiaB = bodyB->invInertiaWorld;
    }
    // bodyB motion behavior
    if (contact.partnerTypeB == ContactPartnerType::Terrain || !contact.contributesMotionB) {
        rB = glm::vec3(0.0f);
        linearVelocityB = glm::vec3(0.0f);
        angularVelocityB = glm::vec3(0.0f);
    }
    else {
        rB = cp.worldPos - tB->position; // #TODO: ska vara rB = contactPoint - body.comWorld;
        linearVelocityB = bodyB->linearVelocity;
        angularVelocityB = bodyB->angularVelocity;
    }

    // pre-calculate rA, rB, EffectiveMass
    cp.rA = rA;
    cp.rB = rB;

    glm::vec3 rA_cross_n = glm::cross(rA, normal);
    glm::vec3 rB_cross_n = glm::cross(rB, normal);
    cp.m_eff = 1.0f / (invMassA + invMassB +
        glm::dot(rA_cross_n, invInertiaA * rA_cross_n) +
        glm::dot(rB_cross_n, invInertiaB * rB_cross_n));

    //if (cp.m_eff <= 1e-8f) {
    //    std::cout << "Warning: contact point with near-zero effective mass!" << std::endl;
    //}

    // compute relative velocity at contact point based on current body states
    glm::vec3 relativeVelocity =
        (linearVelocityB + glm::cross(angularVelocityB, rB)) -
        (linearVelocityA + glm::cross(angularVelocityA, rA));

    // if the contact is warm-started (i.e. "old"), we disable restitution to avoid bounce due to accumulated penetration correction impulses from previous frames, which can cause jitter. 
    // This also means that only new contacts with sufficient impact velocity will bounce, which is a common and stable approach in physics engines.
    bool allowRestitution = true;
    if (cp.wasWarmStarted or contact.framesSinceUsed > 0) {
        allowRestitution = false;
    }

    float normalVelocity = glm::dot(relativeVelocity, normal);
    if (allowRestitution and normalVelocity < -restitutionThreshold) {
        cp.targetBounceVelocity = -restitution * normalVelocity;
    }
    else {
        cp.targetBounceVelocity = 0.0f;
    }

    glm::vec3 rA_t1 = glm::cross(rA, contact.t1);
    glm::vec3 rB_t1 = glm::cross(rB, contact.t1);
    glm::vec3 rA_t2 = glm::cross(rA, contact.t2);
    glm::vec3 rB_t2 = glm::cross(rB, contact.t2);

    glm::vec3 invIA_rA_t1 = invInertiaA * rA_t1;
    glm::vec3 invIB_rB_t1 = invInertiaB * rB_t1;
    glm::vec3 invIA_rA_t2 = invInertiaA * rA_t2;
    glm::vec3 invIB_rB_t2 = invInertiaB * rB_t2;

    // Compute effective mass along cp.t1 and cp.t2 for friction calculations in the solver. 
    // This is needed to determine how much tangential impulse to apply for a given desired change in tangential velocity, similar to how cp.m_eff is used for normal impulses.
    float k_t1 = (invMassA + invMassB) + glm::dot(rA_t1, invIA_rA_t1) + glm::dot(rB_t1, invIB_rB_t1);
    cp.invMassT1 = 1.0f / k_t1;
    float k_t2 = (invMassA + invMassB) + glm::dot(rA_t2, invIA_rA_t2) + glm::dot(rB_t2, invIB_rB_t2);
    cp.invMassT2 = 1.0f / k_t2;

    //if (k_t1 <= 1e-8f || k_t2 <= 1e-8f) {
    //    std::cout << "Warning: contact point with near-zero tangential effective mass!" << std::endl;
    //}
}

//==========================================================================================
// Integration of new contact with cached contact for warm starting and temporal coherence
//==========================================================================================
void CollisionManifold::integrateContact(std::unordered_map<size_t, Contact>& contactCache, Contact& contact) {
    // #TODO: FaceCenter kan användas för att sortera contact points om:
    // 1. nya kontaktpunkter skapades
    // 2. antal ändrades
    // 3. cache-matchning var osäker
    // Ordningen av kontaktpunkter ska vara stabil mellan frames för att undvika jitter.


    auto it = contactCache.find(contact.hashKey);

    contact.wasUsedThisFrame = true;
    contact.framesSinceUsed = 0;

    glm::vec3 n = contact.normal;
    glm::vec3 t1;

    // reproject cached t1 to be perpendicular to the new contact normal n to increase the chance of cache match 
    // for tangential basis even when contact normal changes slightly between frames, which can help with warm starting stability of friction impulses.
    if (it != contactCache.end()) {
        t1 = it->second.t1 - n * glm::dot(it->second.t1, n);
    }

    // if no cache or t1 is near zero, we need to generate a tangential basis (t1 and t2) for the contact to be used in the solver for friction calculations.
    if (it == contactCache.end() or glm::length2(t1) < 1e-8f) {
        uint64_t h = contact.hashKey * 0x9E3779B97F4A7C15ull;
        float theta = float((h >> 33) & 0x7fffffff) * (2.0f * 3.1415926535f) / float(0x80000000);

        // generate an arbitrary tangential vector t1 that is perpendicular to the contact normal n, 
        // using the contact hash to have a deterministic but seemingly random choice of t1 for different contacts to improve stability
        // when we don't have a cached contact to warm start with.
        glm::vec3 seed = (std::abs(n.y) < 0.9f) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
        glm::vec3 b = glm::normalize(glm::cross(seed, n));
        glm::vec3 c = glm::normalize(glm::cross(n, b));
        t1 = glm::normalize(std::cos(theta) * b + std::sin(theta) * c);
    }

    glm::vec3 t2 = glm::normalize(glm::cross(n, t1));

    // if we have a cached contact, we want to make sure that the tangential basis (t1 and t2) of the new contact 
    // is oriented in the same general direction as the cached contact's tangential basis to improve warm starting stability.
    if (it != contactCache.end() and glm::dot(t1, it->second.t1) < 0) { 
        t1 = -t1; 
        t2 = -t2; 
    }
    contact.t1 = t1;
    contact.t2 = t2;

    // no matching contact in cache, so we just add the new contact to cache and return
    if (it == contactCache.end()) {
        for (int i = 0; i < contact.points.size(); i++) {
            PreComputePointData(contact.points[i], contact);
            contact.points[i].wasUsedThisFrame = true;
        }

        contactCache.emplace(contact.hashKey, std::move(contact));
        return;
    }

    // we have a matching contact in cache, so we try to match the new contact points with the cached contact points for warm starting
    Contact& cachedContact = it->second;
    cachedContact.wasUsedThisFrame = true;

    std::array<bool, 4> matchedFinalPoints{ false };
    std::array<bool, 4> matchedCachedPoints{ false };
    glm::vec3& referenceFaceNormal = contact.referenceFaceNormal;
    std::array<glm::vec3, 4>& referenceFace = contact.referenceFace;

    ContactRuntime& rt = contact.runtimeData;
    Collider* colliderA = rt.colliderA;
    Collider* colliderB = rt.colliderB;
    Transform* tA = rt.bodyRootA;
    Transform* tB = rt.bodyRootB;


    // Pre-transform cached contact points to world space for proximity comparison with new contact points.
    glm::mat3 M3; 
    glm::vec3 T3; 
    if (cachedContact.objBisReference and contact.partnerTypeB == ContactPartnerType::RigidBody) {
        M3 = glm::mat3(tB->modelMatrix);
        T3 = glm::vec3(tB->modelMatrix[3]);
    } else {
        M3 = glm::mat3(tA->modelMatrix);
        T3 = glm::vec3(tA->modelMatrix[3]);
    }
    glm::vec3 cachedWorld[4];
    for (int j = 0; j < cachedContact.points.size(); ++j) {
        cachedWorld[j] = M3 * cachedContact.points[j].localPos + T3;
    }

    //===========================================================================================================
    // Warm start matching of contact points based on proximity.
    // Transfer impulse data from matched cached points to new points after projecting to new contact basis
    const float warmstartMatchingThreshold = 0.05f; // #TODO: behöver ändras till att vara en faktor av objektens storlekar
    const float thresholdSq = warmstartMatchingThreshold * warmstartMatchingThreshold;
    
    for (int i = 0; i < contact.points.size(); i++) {
        ContactPoint& newPoint = contact.points[i];
        for (int j = 0; j < cachedContact.points.size(); j++) {

            if (matchedFinalPoints[i]) break;
            if (matchedCachedPoints[j]) continue;

            ContactPoint& cachedPoint = cachedContact.points[j];

            glm::vec3 transformedPoint = cachedWorld[j];

            // new point is close to a cached point = warm start
            float dist2 = glm::distance2(newPoint.worldPos, transformedPoint);

            if (dist2 < thresholdSq) {

                newPoint.wasWarmStarted = true;

                // Remake the contact point data to match the new contact normal and tangential basis
                glm::vec3 oldImpulseWorld =
                    cachedPoint.accumulatedImpulse * cachedContact.normal +
                    cachedPoint.accumulatedFrictionImpulse1 * cachedContact.t1 +
                    cachedPoint.accumulatedFrictionImpulse2 * cachedContact.t2;

                // Project the old impulse onto the new contact normal and tangential basis
                newPoint.accumulatedImpulse = glm::dot(oldImpulseWorld, contact.normal);
                newPoint.accumulatedFrictionImpulse1 = glm::dot(oldImpulseWorld, contact.t1);
                newPoint.accumulatedFrictionImpulse2 = glm::dot(oldImpulseWorld, contact.t2);

                matchedFinalPoints[i] = true;
                matchedCachedPoints[j] = true;
                break;
            }
        }
    }

    //====================================================================================================
    // Add extra cached points if there's less than 4 points in the new contact (only done for box-box)
    if (contact.points.size() < 4 &&
        contact.partnerTypeA == ContactPartnerType::RigidBody &&
        contact.partnerTypeB == ContactPartnerType::RigidBody &&
        colliderA->type == ColliderType::CUBOID &&
        colliderB->type == ColliderType::CUBOID)
    {
        // add cached points that were not matched with a new point, but only if they are still valid 
        // (i.e. still inside the reference face and not too far from the new contact points), 
        // and transform them to match the new contact normal and tangential basis
        const float clippingPlaneTolerance = 1e-7f;
        const float maxPlaneDistanceForWarmStart = 0.1f;
        const float maxBackfaceDistance = 0.01f;

        if (contact.partnerTypeB == ContactPartnerType::RigidBody && contact.points.size() < 4) {
            for (int i = 0; i < cachedContact.points.size(); i++) {
                if (matchedCachedPoints[i] == true)
                    continue;

                ContactPoint& cachedPoint = cachedContact.points[i];
                glm::vec3& transformedPoint = cachedWorld[i];

                // check if point is still inside reference face
                bool inside = true;
                for (auto& plane : this->clippingPlanes) {
                    if (glm::dot(plane.normal, transformedPoint - plane.point) >= clippingPlaneTolerance) {
                        inside = false;
                        break;
                    }
                }
                if (!inside) continue;  // skip if the point is outside the reference face

                // Check if the point is not too far from the reference face plane (i.e. still relevant for warm starting). 
                float dn = glm::dot(referenceFaceNormal, transformedPoint - referenceFace[0]);
                if (dn > maxPlaneDistanceForWarmStart) {
                    continue;
                }

                // also allow points that are slightly behind the reference face to be warm started, to avoid jitter for points near the edge of the reference face.
                if (dn < -maxBackfaceDistance) continue;

                glm::vec3 projP = transformedPoint - dn * referenceFaceNormal;
                cachedPoint.worldPos = projP;

                Transform* root;
                if (contact.objBisReference and contact.partnerTypeB == ContactPartnerType::RigidBody) {
                    root = contact.runtimeData.bodyRootB;
                }
                else {
                    root = contact.runtimeData.bodyRootA;
                }

                cachedPoint.localPos = root->worldToLocalPoint(cachedPoint.worldPos);
                cachedPoint.depth = -glm::dot(cachedPoint.worldPos - contact.referenceFace[0], contact.referenceFaceNormal);

                cachedPoint.wasWarmStarted = true;

                // Remake the contact point data to match the new contact normal and tangential basis
                glm::vec3 oldImpulseWorld =
                    cachedPoint.accumulatedImpulse * cachedContact.normal +
                    cachedPoint.accumulatedFrictionImpulse1 * cachedContact.t1 +
                    cachedPoint.accumulatedFrictionImpulse2 * cachedContact.t2;

                // Project the old impulse onto the new contact normal and tangential basis
                cachedPoint.accumulatedImpulse = glm::dot(oldImpulseWorld, contact.normal);
                cachedPoint.accumulatedFrictionImpulse1 = glm::dot(oldImpulseWorld, contact.t1);
                cachedPoint.accumulatedFrictionImpulse2 = glm::dot(oldImpulseWorld, contact.t2);

                contact.points.push_back(cachedPoint);

                if (contact.points.size() >= 4) {
                    break;
                }
            }
        }
    }

    // Finally, we copy the new contact (with updated impulse data for warm starting) into the cache, replacing the old contact.
    for (int i = 0; i < contact.points.size(); i++) {
        PreComputePointData(contact.points[i], contact);
        contact.points[i].wasUsedThisFrame = true;
    }

    it->second = std::move(contact);
}

//========================================
// Contact caching key generation
//========================================
size_t CollisionManifold::generateKey(int idA, int idB) {
    return (uint64_t)std::min(idA, idB) << 32 | std::max(idA, idB);

    // #TODO: en terrain-kontakt ovanifrån kan i värsta fall:
    // 1. hitta "fel" cached manifold
    // 2. skriva över en box-box manifold
    // 3. warm-starta med fel impulser
    // 4. eller bara byta kontakt-cache-innehåll mellan körningar beroende på vilken kontakt som råkade gå in sist

    // #TODO: Nytt objekt med samma slot(id) i SlotMap som ett gammalt objekt i contact cache kan orsaka hash-kollision.
    // Varar endast i 5 frames men behöver fixas. Kan orsaka jitter och/eller felaktiga kontaktpunkter.
}









////=============================================
//// Edge vs edge contact point calculation
//// =============================================
//std::array<glm::vec3, 2> CollisionManifold::edgeEdgePoints(glm::vec3& P0, glm::vec3& P1, glm::vec3& Q0, glm::vec3& Q1)
//{
//    glm::vec3 u = P1 - P0;
//    glm::vec3 v = Q1 - Q0;
//    glm::vec3 w = P0 - Q0;
//    float a = glm::dot(u, u);
//    float b = glm::dot(u, v);
//    float c = glm::dot(v, v);
//    float d = glm::dot(u, w);
//    float e = glm::dot(v, w);
//    float Delta = a * c - b * b;
//
//    float s = 0.0f;
//    float t = 0.0f;
//    if (Delta < 1e-6f) // parallella kanter
//    {
//        s = 0.0f;
//        t = glm::clamp(e / c, 0.0f, 1.0f);
//    }
//    else
//    {
//        float s_star = (b * e - c * d) / Delta;
//        float t_star = (a * e - b * d) / Delta;
//        s = glm::clamp(s_star, 0.0f, 1.0f);
//        t = glm::clamp(t_star, 0.0f, 1.0f);
//        // Hantera kantfall: om s kläms, räkna om t = (b*s + e)/c; vice versa
//    }
//
//    glm::vec3 C1 = P0 + s * u;
//    glm::vec3 C2 = Q0 + t * v;
//    std::array<glm::vec3, 2> contactPoints = { C1, C2 };
//
//    return contactPoints;
//}