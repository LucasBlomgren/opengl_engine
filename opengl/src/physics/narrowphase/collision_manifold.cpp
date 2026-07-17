#include "pch.h"
#include "collision_manifold.h"

// #TODO: implementera recycled contacts för att genom heuristik kunna återanvända gamla contacts 
// istället för att skapa nya varje frame, för att förbättra solver stabilitet och prestanda.
// Recycling kan ske redan innan SAT, genom att jämföra transforms och collider IDs
// Se Box3D.

// 1. key = generateKey(a, b), slå upp i cachen.
// 
// 2. Finns den + relativeTransformValid + recycling på : beräkna rörelse - bounden(vinkeländring + translation + rotationsbåge·maxExtent) 
//    mot en tolerans.Ingen normal - jämförelse.
// 
// 3. Under toleransen : rotera cachade ankare med delta - rotation, räkna ny separation per punkt via baseSeparation + dot(dp, normal), 
//    behåll impulserna(warm start), kör solverns active - check som vanligt.Hoppa över SAT + face - select + clipping + reduction + integrateContact - matchningen.
// 
// 4. Över toleransen(eller ingen cache) : full väg — SAT, contact generation — och cacha(qA, qB, relativ pose, baseSeparation per punkt) för nästa frame.


//=============================================
// Box-Box collision
//=============================================
Contact* CollisionManifold::boxBox(
    Contact& contact,
    std::unordered_map<size_t, Contact>& cache,
    SAT::Result& satResult
) {
    ContactRuntime& rt = contact.runtimeData;
    Collider* colliderA = rt.colliderA;
    Collider* colliderB = rt.colliderB;

    const float linearSlop = 0.001f;
    const float k_tol = 0.1f * linearSlop;

    contact.clearPoints();
    contact.objBisReference = (satResult.separationB > satResult.separationA + k_tol);

    if (contact.objBisReference) {
        selectOOBBCollisionRefFaceAndNormal(
            colliderB,
            colliderB->pose,
            -satResult.normal,
            contact.referenceFace,
            contact.referenceFaceNormal
        );

        selectOOBBCollisionIncidentFace(
            colliderA,
            colliderA->pose,
            satResult.normal,
            contact.incidentFace
        );
    }
    else {
        selectOOBBCollisionRefFaceAndNormal(
            colliderA,
            colliderA->pose,
            satResult.normal,
            contact.referenceFace,
            contact.referenceFaceNormal
        );

        selectOOBBCollisionIncidentFace(
            colliderB,
            colliderB->pose,
            -satResult.normal,
            contact.incidentFace
        );
    }

    clipPoints(contact.referenceFace, contact.incidentFace, 4, contact.referenceFaceNormal);

    Transform* root = nullptr;
    if (contact.objBisReference && contact.partnerTypeB == ContactPartnerType::RigidBody) {
        root = contact.runtimeData.bodyRootB;
    }
    else {
        root = contact.runtimeData.bodyRootA;
    }

    std::array<ContactPoint, MaxClippedPoints> candidates{};
    uint32_t candidateCount = 0;

    for (uint32_t i = 0; i < clippedPointCount; ++i) {
        assert(candidateCount < candidates.size());

        ContactPoint cp{};
        cp.worldPos = clippedPoints[i];
        cp.depth = -glm::dot(clippedPoints[i] - contact.referenceFace[0], contact.referenceFaceNormal);
        cp.localPos = root->worldToLocalPoint(cp.worldPos);

        candidates[candidateCount++] = cp;
    }

    contact.normal = satResult.normal;

    if (candidateCount <= MaxContactPoints) {
        for (uint32_t i = 0; i < candidateCount; ++i) {
            contact.addPoint(candidates[i]);
        }
    }
    else {
        // #TODO: different points are chosen different frames, causing solver instability, maybe add some temporal coherence to point selection?
// like remembering which points were chosen last frame and prefer those if they are still valid (within some tolerance), 
// or add some bias towards points with deeper penetration depth
        contactPointReduction(contact, candidates, candidateCount);
    }

    contact.hashKey = generateKey(colliderA->id, colliderB->id);

    return integrateContact(cache, contact);
}

//==================================================
// Box-Sphere collision
//==================================================
Contact* CollisionManifold::boxSphere(
    Contact& contact,
    std::unordered_map<size_t, Contact>& cache,
    SAT::Result& satResult
) {
    contact.clearPoints();
    contact.normal = satResult.normal;

    Transform* root = nullptr;
    if (contact.objBisReference && contact.partnerTypeB == ContactPartnerType::RigidBody) {
        root = contact.runtimeData.bodyRootB;
    }
    else {
        root = contact.runtimeData.bodyRootA;
    }

    ContactPoint cp{};
    cp.worldPos = satResult.point;
    cp.depth = satResult.depth;
    cp.localPos = root->worldToLocalPoint(cp.worldPos);

    contact.addPoint(cp);

    contact.hashKey = generateKey(
        contact.runtimeData.colliderA->id,
        contact.runtimeData.colliderB->id
    );

    return integrateContact(cache, contact);
}

//===================================================
// Box-Mesh collision
//===================================================
Contact* CollisionManifold::boxMesh(
    Contact& contact,
    std::unordered_map<size_t, Contact>& cache,
    std::vector<SAT::Result>& allResults
) {
    if (allResults.empty())
        return nullptr;

    ContactRuntime& rt = contact.runtimeData;
    Collider* colliderA = rt.colliderA;

    contact.clearPoints();

    meshCandidateCount = 0;
    furthestCandidateCount = 0;
    selectedCandidateCount = 0;

    for (int resultIndex = 0; resultIndex < static_cast<int>(allResults.size()); ++resultIndex) {
        const SAT::Result& satResult = allResults[resultIndex];

        selectOOBBCollisionRefFaceAndNormal(
            colliderA,
            colliderA->pose,
            satResult.normal,
            contact.referenceFace,
            contact.referenceFaceNormal
        );

        contact.incidentFace = {
            satResult.tri_ptr->vertices[0],
            satResult.tri_ptr->vertices[1],
            satResult.tri_ptr->vertices[2],
            glm::vec3(0.0f)
        };

        clipPoints(contact.referenceFace, contact.incidentFace, 3, contact.referenceFaceNormal);

        for (uint32_t i = 0; i < clippedPointCount; ++i) {
            if (meshCandidateCount >= meshCandidates.size()) {
                break;
            }

            MeshContactCandidate candidate{};
            candidate.worldPos = clippedPoints[i];
            candidate.depth = computePenetrationDepth(
                clippedPoints[i],
                contact.referenceFace,
                contact.referenceFaceNormal
            );
            candidate.sourceIndex = resultIndex;

            meshCandidates[meshCandidateCount++] = candidate;
        }

        if (meshCandidateCount >= meshCandidates.size()) {
            break;
        }
    }

    if (meshCandidateCount == 0)
        return nullptr;

    pickFourFurthestPoints();

    Transform* root = contact.runtimeData.bodyRootA; // terrain => bodyA

    for (uint32_t i = 0; i < furthestCandidateCount; ++i) {
        ContactPoint cp{};
        cp.worldPos = furthestCandidates[i].worldPos;
        cp.depth = furthestCandidates[i].depth;
        cp.localPos = root->worldToLocalPoint(cp.worldPos);

        contact.addPoint(cp);
    }

    contact.hashKey = generateKey(colliderA->id, allResults[0].tri_ptr->id);

    return integrateContact(cache, contact);
}

//===================================================
// Sphere-Sphere collision
//===================================================
Contact* CollisionManifold::sphereSphere(
    Contact& contact,
    std::unordered_map<size_t, Contact>& cache,
    SAT::Result& satResult
) {
    ContactRuntime& rt = contact.runtimeData;
    Collider* colliderA = rt.colliderA;
    Collider* colliderB = rt.colliderB;

    contact.clearPoints();
    contact.normal = satResult.normal;

    Transform* root = nullptr;
    if (contact.objBisReference && contact.partnerTypeB == ContactPartnerType::RigidBody) {
        root = contact.runtimeData.bodyRootB;
    }
    else {
        root = contact.runtimeData.bodyRootA;
    }

    ContactPoint cp{};
    cp.worldPos = satResult.point;
    cp.depth = satResult.depth;
    cp.localPos = root->worldToLocalPoint(cp.worldPos);

    cp.speculative = (satResult.hitType == SAT::HitType::Speculative);
    cp.separation = satResult.separation;
    cp.toi = satResult.toi;
    contact.addPoint(cp);

    contact.hashKey = generateKey(colliderA->id, colliderB->id);

    return integrateContact(cache, contact);
}

//===================================================
// Sphere-Mesh collision
//===================================================
Contact* CollisionManifold::sphereMesh(
    Contact& contact,
    std::unordered_map<size_t, Contact>& cache,
    std::vector<SAT::Result>& allResults) 
{
    if (allResults.empty())
        return nullptr;

    ContactRuntime& rt = contact.runtimeData;
    Collider* colliderA = rt.colliderA;

    contact.clearPoints();

    meshCandidateCount = 0;
    furthestCandidateCount = 0;
    selectedCandidateCount = 0;

    // Om caller inte redan sätter contact.normal, är detta säkrare.
    contact.normal = allResults[0].normal;

    for (int i = 0; i < static_cast<int>(allResults.size()); ++i) {
        if (meshCandidateCount >= meshCandidates.size()) {
            break;
        }

        MeshContactCandidate candidate{};
        candidate.worldPos = allResults[i].point;
        candidate.depth = allResults[i].depth;
        candidate.sourceIndex = i;

        meshCandidates[meshCandidateCount++] = candidate;
    }

    if (meshCandidateCount == 0)
        return nullptr;

    pickFourFurthestPoints();

    Transform* root = contact.runtimeData.bodyRootA;

    for (uint32_t i = 0; i < furthestCandidateCount; ++i) {
        ContactPoint cp{};
        cp.worldPos = furthestCandidates[i].worldPos;
        cp.depth = furthestCandidates[i].depth;
        cp.localPos = root->worldToLocalPoint(cp.worldPos);

        contact.addPoint(cp);
    }

    contact.hashKey = generateKey(colliderA->id, allResults[0].tri_ptr->id);

    return integrateContact(cache, contact);
}

//===================================================
// Helper function for contact generation
//===================================================
void CollisionManifold::pickFourFurthestPoints() {
    furthestCandidateCount = 0;

    if (meshCandidateCount == 0)
        return;

    if (meshCandidateCount <= MaxContactPoints) {
        for (uint32_t i = 0; i < meshCandidateCount; ++i) {
            furthestCandidates[furthestCandidateCount++] = meshCandidates[i];
        }
        return;
    }

    // seed deterministiskt med första kandidaten (se TODO om temporal koherens)
    selectedCandidateCount = 0;
    selectedCandidateIndices[selectedCandidateCount++] = 0;

    for (uint32_t i = 1; i < MaxContactPoints; ++i) {
        addFurthestPoint();
    }

    for (uint32_t k = 0; k < selectedCandidateCount; ++k) {
        furthestCandidates[furthestCandidateCount++] =
            meshCandidates[selectedCandidateIndices[k]];
    }
}

void CollisionManifold::addFurthestPoint() {
    int bestIdx = -1;
    float bestDist = std::numeric_limits<float>::lowest();

    for (uint32_t i = 0; i < meshCandidateCount; ++i) {
        bool alreadyAdded = false;

        for (uint32_t j = 0; j < selectedCandidateCount; ++j) {
            if (static_cast<int>(i) == selectedCandidateIndices[j]) {
                alreadyAdded = true;
                break;
            }
        }

        if (alreadyAdded)
            continue;

        float shortestDist = std::numeric_limits<float>::max();

        for (uint32_t j = 0; j < selectedCandidateCount; ++j) {
            int selectedIdx = selectedCandidateIndices[j];

            glm::vec3 diff =
                meshCandidates[i].worldPos -
                meshCandidates[selectedIdx].worldPos;

            float dist2 = glm::dot(diff, diff);

            if (dist2 < shortestDist) {
                shortestDist = dist2;
            }
        }

        if (shortestDist > bestDist) {
            bestDist = shortestDist;
            bestIdx = static_cast<int>(i);
        }
    }

    if (bestIdx >= 0) {
        assert(selectedCandidateCount < selectedCandidateIndices.size());
        selectedCandidateIndices[selectedCandidateCount++] = bestIdx;
    }
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
        localFace = (rotated.x > 0) 
            ? box.getLocalFace(FaceId::MaxX)
            : box.getLocalFace(FaceId::MinX);
    }
    else if (absN.y >= absN.x && absN.y >= absN.z) {
        localFace = (rotated.y > 0) 
            ? box.getLocalFace(FaceId::MaxY)
            : box.getLocalFace(FaceId::MinY);
    }
    else {
        localFace = (rotated.z > 0) 
            ? box.getLocalFace(FaceId::MaxZ)
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
void CollisionManifold::clipPoints(
    const std::array<glm::vec3, 4>& referenceFace,
    const std::array<glm::vec3, 4>& incidentFace,
    int incidentCount,
    const glm::vec3& referenceFaceNormal) 
{
    createClippingPlanes(referenceFace, referenceFaceNormal);

    clippedPointCount = 0;

    assert(incidentCount <= static_cast<int>(MaxClippedPoints));

    for (int i = 0; i < incidentCount; i++) {
        contactPoints[i] = incidentFace[i];
    }

    int counter = incidentCount;

    for (const Plane& plane : clippingPlanes) {
        for (int i = 0; i < counter; i++) {
            clippingStatus[i] = isPointInsidePlane(
                contactPoints[i],
                plane.normal,
                plane.point,
                1e-2f
            );
        }

        int oldCounter = counter;
        counter = 0;

        auto pushNext = [&](const glm::vec3& p) {
            assert(counter < static_cast<int>(MaxClippedPoints));
            nextContactPoints[counter++] = p;
            };

        for (int i = 0; i < oldCounter; i++) {
            int nextIndex = (i + 1) % oldCounter;

            bool validClip = true;
            glm::vec3 clippedPoint{};

            if (!clippingStatus[i] && clippingStatus[nextIndex]) {
                getIntersectionPoint(
                    contactPoints[i],
                    contactPoints[nextIndex],
                    plane,
                    clippedPoint,
                    validClip
                );

                if (validClip)
                    pushNext(clippedPoint);

                pushNext(contactPoints[nextIndex]);
            }
            else if (clippingStatus[i] && clippingStatus[nextIndex]) {
                pushNext(contactPoints[nextIndex]);
            }
            else if (clippingStatus[i] && !clippingStatus[nextIndex]) {
                getIntersectionPoint(
                    contactPoints[i],
                    contactPoints[nextIndex],
                    plane,
                    clippedPoint,
                    validClip
                );

                if (validClip)
                    pushNext(clippedPoint);
            }
        }

        std::swap(contactPoints, nextContactPoints);

        if (counter == 0)
            break;
    }

    for (int i = 0; i < counter; i++) {
        if (isPointInsidePlane(
            contactPoints[i],
            referenceFaceNormal,
            referenceFace[0],
            1e-2f
        )) {
            assert(clippedPointCount < clippedPoints.size());
            clippedPoints[clippedPointCount++] = contactPoints[i];
        }
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
void CollisionManifold::contactPointReduction(
    Contact& contact,
    const std::array<ContactPoint, MaxClippedPoints>& candidates,
    uint32_t candidateCount
) {
    assert(candidateCount > MaxContactPoints);

    glm::vec3 normal = contact.referenceFaceNormal;

    std::array<int, MaxContactPoints> chosen{};
    uint32_t chosenCount = 0;

    auto alreadyChosen = [&](int idx) {
        for (uint32_t i = 0; i < chosenCount; ++i) {
            if (chosen[i] == idx)
                return true;
        }
        return false;
        };

    auto addChosen = [&](int idx) {
        if (idx < 0)
            return;

        if (alreadyChosen(idx))
            return;

        assert(chosenCount < chosen.size());
        chosen[chosenCount++] = idx;
        };

    auto pickFarthestUnusedFrom = [&](const glm::vec3& fromPoint) {
        int bestIdx = -1;
        float bestDist = std::numeric_limits<float>::lowest();

        for (uint32_t i = 0; i < candidateCount; ++i) {
            if (alreadyChosen(static_cast<int>(i)))
                continue;

            float dist2 = glm::distance2(fromPoint, candidates[i].localPos);

            if (dist2 > bestDist) {
                bestDist = dist2;
                bestIdx = static_cast<int>(i);
            }
        }

        return bestIdx;
        };

    // 1. support point, deterministiskt i local X
    const glm::vec3 direction(1.0f, 0.0f, 0.0f);

    int supportPointIndex = 0;
    float maxDot = std::numeric_limits<float>::lowest();

    for (uint32_t i = 0; i < candidateCount; ++i) {
        float dotValue = glm::dot(candidates[i].localPos, direction);

        if (dotValue > maxDot) {
            maxDot = dotValue;
            supportPointIndex = static_cast<int>(i);
        }
    }

    addChosen(supportPointIndex);

    const glm::vec3 supportPoint = candidates[supportPointIndex].localPos;

    // 2. längst från support point
    int farthestPointIndex = pickFarthestUnusedFrom(supportPoint);
    addChosen(farthestPointIndex);

    const glm::vec3 farthestPoint = candidates[farthestPointIndex].localPos;

    // 3. största positiva area
    int positiveAreaIndex = -1;
    float maxArea = std::numeric_limits<float>::lowest();

    for (uint32_t i = 0; i < candidateCount; ++i) {
        if (alreadyChosen(static_cast<int>(i)))
            continue;

        const glm::vec3& point = candidates[i].localPos;

        float area = 0.5f * glm::dot(
            glm::cross(supportPoint - farthestPoint, supportPoint - point),
            normal
        );

        if (area > maxArea) {
            maxArea = area;
            positiveAreaIndex = static_cast<int>(i);
        }
    }

    addChosen(positiveAreaIndex);

    // 4. största negativa area
    int negativeAreaIndex = -1;
    float minArea = std::numeric_limits<float>::max();

    for (uint32_t i = 0; i < candidateCount; ++i) {
        if (alreadyChosen(static_cast<int>(i)))
            continue;

        const glm::vec3& point = candidates[i].localPos;

        float area = 0.5f * glm::dot(
            glm::cross(supportPoint - farthestPoint, supportPoint - point),
            normal
        );

        if (area < minArea) {
            minArea = area;
            negativeAreaIndex = static_cast<int>(i);
        }
    }

    addChosen(negativeAreaIndex);

    // fallback om något degenererat hände
    while (chosenCount < MaxContactPoints) {
        int fallbackIdx = pickFarthestUnusedFrom(supportPoint);
        if (fallbackIdx < 0)
            break;

        addChosen(fallbackIdx);
    }

    contact.clearPoints();

    for (uint32_t i = 0; i < chosenCount; ++i) {
        contact.addPoint(candidates[chosen[i]]);
    }
}

float CollisionManifold::computePenetrationDepth(
    const glm::vec3& point,
    const std::array<glm::vec3, 4>& refFace,
    const glm::vec3& refFaceNormal
) {
    return -glm::dot(point - refFace[0], refFaceNormal);
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
}

// #TODO: FaceCenter kan användas för att sortera contact points om:
// 1. nya kontaktpunkter skapades
// 2. antal ändrades
// 3. cache-matchning var osäker
// Ordningen av kontaktpunkter ska vara stabil mellan frames för att undvika jitter.
//==========================================================================================
// Integration of new contact with cached contact for warm starting and temporal coherence
//==========================================================================================
Contact* CollisionManifold::integrateContact(
    std::unordered_map<size_t, Contact>& contactCache,
    Contact& contact
) {
    contact.minY = std::numeric_limits<float>::max();

    for (uint32_t i = 0; i < contact.numPoints; ++i) {
        const ContactPoint& cp = contact.points[i];
        contact.minY = std::min(contact.minY, cp.worldPos.y);
    }

    auto it = contactCache.find(contact.hashKey);

    contact.wasUsedThisFrame = true;
    contact.framesSinceUsed = 0;

    glm::vec3 n = contact.normal;
    glm::vec3 t1{};

    if (it != contactCache.end()) {
        t1 = it->second.t1 - n * glm::dot(it->second.t1, n);

        if (glm::length2(t1) > 1e-8f) {
            t1 = glm::normalize(t1);
        }
    }

    if (it == contactCache.end() || glm::length2(t1) < 1e-8f) {
        uint64_t h = contact.hashKey * 0x9E3779B97F4A7C15ull;

        float theta =
            float((h >> 33) & 0x7fffffff) *
            (2.0f * 3.1415926535f) /
            float(0x80000000);

        glm::vec3 seed =
            (std::abs(n.y) < 0.9f)
            ? glm::vec3(0, 1, 0)
            : glm::vec3(1, 0, 0);

        glm::vec3 b = glm::normalize(glm::cross(seed, n));
        glm::vec3 c = glm::normalize(glm::cross(n, b));

        t1 = glm::normalize(std::cos(theta) * b + std::sin(theta) * c);
    }

    glm::vec3 t2 = glm::normalize(glm::cross(n, t1));

    if (it != contactCache.end() && glm::dot(t1, it->second.t1) < 0.0f) {
        t1 = -t1;
        t2 = -t2;
    }

    contact.t1 = t1;
    contact.t2 = t2;

    if (it == contactCache.end()) {
        for (uint32_t i = 0; i < contact.numPoints; ++i) {
            PreComputePointData(contact.points[i], contact);
            contact.points[i].wasUsedThisFrame = true;
        }

        size_t key = contact.hashKey;

        auto [insertedIt, inserted] =
            contactCache.emplace(key, std::move(contact));

        return &insertedIt->second;
    }

    Contact& cachedContact = it->second;
    cachedContact.wasUsedThisFrame = true;

    std::array<bool, MaxContactPoints> matchedFinalPoints{ false };
    std::array<bool, MaxContactPoints> matchedCachedPoints{ false };

    ContactRuntime& rt = contact.runtimeData;
    Transform* tA = rt.bodyRootA;
    Transform* tB = rt.bodyRootB;

    glm::mat3 M3;
    glm::vec3 T3;

    if (cachedContact.objBisReference && contact.partnerTypeB == ContactPartnerType::RigidBody) {
        M3 = glm::mat3(tB->modelMatrix);
        T3 = glm::vec3(tB->modelMatrix[3]);
    }
    else {
        M3 = glm::mat3(tA->modelMatrix);
        T3 = glm::vec3(tA->modelMatrix[3]);
    }

    std::array<glm::vec3, MaxContactPoints> cachedWorld{};

    for (uint32_t j = 0; j < cachedContact.numPoints; ++j) {
        cachedWorld[j] = M3 * cachedContact.points[j].localPos + T3;
    }

    struct MatchPair {
        int newIdx = -1;
        int cachedIdx = -1;
        float dist2 = 0.0f;
    };

    std::array<MatchPair, MaxContactPoints* MaxContactPoints> pairs{};
    uint32_t pairCount = 0;

    float thresholdSq = 0.05f * 0.05f;

    for (uint32_t i = 0; i < contact.numPoints; ++i) {
        for (uint32_t j = 0; j < cachedContact.numPoints; ++j) {
            float dist2 = glm::distance2(contact.points[i].worldPos, cachedWorld[j]);

            if (dist2 < thresholdSq) {
                assert(pairCount < pairs.size());

                pairs[pairCount++] = {
                    static_cast<int>(i),
                    static_cast<int>(j),
                    dist2
                };
            }
        }
    }

    std::sort(
        pairs.begin(),
        pairs.begin() + pairCount,
        [](const MatchPair& a, const MatchPair& b) {
            return a.dist2 < b.dist2;
        }
    );

    for (uint32_t k = 0; k < pairCount; ++k) {
        int i = pairs[k].newIdx;
        int j = pairs[k].cachedIdx;

        if (matchedFinalPoints[i])
            continue;

        if (matchedCachedPoints[j])
            continue;

        ContactPoint& newPoint = contact.points[i];
        ContactPoint& cachedPoint = cachedContact.points[j];

        glm::vec3 oldImpulseWorld =
            cachedPoint.accumulatedNormalImpulse * cachedContact.normal +
            cachedPoint.accumulatedFrictionImpulse1 * cachedContact.t1 +
            cachedPoint.accumulatedFrictionImpulse2 * cachedContact.t2;

        newPoint.accumulatedNormalImpulse =
            glm::max(glm::dot(oldImpulseWorld, contact.normal), 0.0f);

        newPoint.accumulatedFrictionImpulse1 =
            glm::dot(oldImpulseWorld, contact.t1);

        newPoint.accumulatedFrictionImpulse2 =
            glm::dot(oldImpulseWorld, contact.t2);

        float maxFriction = 0.6f * newPoint.accumulatedNormalImpulse;

        float f1 = newPoint.accumulatedFrictionImpulse1;
        float f2 = newPoint.accumulatedFrictionImpulse2;

        float len2 = f1 * f1 + f2 * f2;
        float max2 = maxFriction * maxFriction;

        if (len2 > max2) {
            float len = std::sqrt(len2);

            if (len > 1e-6f) {
                float s = maxFriction / len;
                newPoint.accumulatedFrictionImpulse1 *= s;
                newPoint.accumulatedFrictionImpulse2 *= s;
            }
        }

        newPoint.wasWarmStarted = true;

        matchedFinalPoints[i] = true;
        matchedCachedPoints[j] = true;
    }

    for (uint32_t i = 0; i < contact.numPoints; ++i) {
        ContactPoint& cp = contact.points[i];

        if (!cp.wasWarmStarted) {
            cp.accumulatedNormalImpulse = 0.0f;
            cp.accumulatedFrictionImpulse1 = 0.0f;
            cp.accumulatedFrictionImpulse2 = 0.0f;
        }
    }

    for (uint32_t i = 0; i < contact.numPoints; ++i) {
        PreComputePointData(contact.points[i], contact);
        contact.points[i].wasUsedThisFrame = true;
    }

    it->second = std::move(contact);

    return &it->second;
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