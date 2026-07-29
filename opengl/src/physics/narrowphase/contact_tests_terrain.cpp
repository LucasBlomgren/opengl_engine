#pragma once
#include "narrowphase_manager.h"

namespace physics::internal {

//===============================================
//     Tris vs Box
//===============================================
void NarrowphaseManager::processTerrainTriBox(ContactBatch& batch, BodyHandle bodyH, Collider* collider, RigidBody* body, const std::vector<Tri*>& candidates)
{
    SAT_resultsList.clear();

    glm::vec3 pushOutSum{ 0.0f };
    float maxDepth = 0.0f;

    // Box data once, since collider pose is same for all terrain triangles.
    const OOBB& box = std::get<OOBB>(collider->shape);

    glm::mat3 R = glm::mat3_cast(collider->worldPose.orientation);

    glm::vec3 boxAxes[3] = {
        glm::normalize(R[0]),
        glm::normalize(R[1]),
        glm::normalize(R[2])
    };

    // Anpassa om din scale sitter på annan plats.
    glm::vec3 half = box.localHalfExtents * box.scale;

    glm::vec3 boxCenter = collider->worldPose.position;

    // SAT for each tri
    for (Tri* tri : candidates) {
        SAT::Result SAT_result;

        // First: only use boxTri as "does this box overlap this triangle at all?"
        if (!SAT::boxTri(*collider, *tri, SAT_result)) {
            continue;
        }

        glm::vec3 triNormalRaw = glm::cross(
            tri->vertices[1] - tri->vertices[0],
            tri->vertices[2] - tri->vertices[0]
        );

        float triNormalLen2 = glm::dot(triNormalRaw, triNormalRaw);
        if (triNormalLen2 < 1e-12f) {
            continue;
        }

        glm::vec3 triNormal = triNormalRaw * (1.0f / std::sqrt(triNormalLen2));

        // Box radius projected onto the triangle normal.
        float boxRadiusOnTriNormal =
            half.x * std::abs(glm::dot(boxAxes[0], triNormal)) +
            half.y * std::abs(glm::dot(boxAxes[1], triNormal)) +
            half.z * std::abs(glm::dot(boxAxes[2], triNormal));

        // Signed distance from box center to triangle plane.
        // Positive = box center is on the side triNormal points toward.
        float signedDist = glm::dot(boxCenter - tri->vertices[0], triNormal);

        // Penetration measured along the terrain face normal.
        // This stays valid even if the box center has gone partly/fully through the plane.
        float faceDepth = boxRadiusOnTriNormal - signedDist;

        // For terrain-face-only character collision:
        // If there is no penetration through the face plane, ignore edge-only contacts.
        if (faceDepth <= 0.0f) {
            continue;
        }

        // Force terrain contact to use face normal.
        //
        // Solver convention:
        // A = body/box
        // B = terrain
        // normal = A -> B
        //
        // triNormal is terrain -> body / pushout direction.
        SAT_result.normal = -triNormal;
        SAT_result.depth = faceDepth;
        SAT_result.feature.type = SAT::AxisType::TriFace;
        SAT_result.tri_ptr = tri;

        SAT_resultsList.push_back(SAT_result);

        float w = std::max(faceDepth, 0.0001f);

        // Pushout direction for character controller:
        // terrain -> body.
        pushOutSum += triNormal * w;

        maxDepth = std::max(maxDepth, faceDepth);
    }

    if (SAT_resultsList.empty()) {
        return;
    }

    body->totalCollisionCount++;

    glm::vec3 avgSolverNormal = getAvgNormal(SAT_resultsList);

    // Export to character controller
    if (body->motionControl == MotionControl::External)
    {
        glm::vec3 avgPushOutNormal;
        if (glm::length2(pushOutSum) > 1e-10f) {
            avgPushOutNormal = glm::normalize(pushOutSum);
        }
        else {
            avgPushOutNormal = -avgSolverNormal;
        }

        externalContacts.emplace_back(
            collider->rigidBodyHandle,
            BodyHandle{},
            -avgPushOutNormal,
            maxDepth,
            true // terrainContact
        );

        return;
    }

    SAT::findBestTriangles(SAT_resultsList);

    // collision manifold generation
    glm::vec3 avgNormal = getAvgNormal(SAT_resultsList);
    if (glm::length2(avgNormal) < 1e-8f) {
        avgNormal = SAT_resultsList[0].normal; // temporary fallback for degenerate cases where all SAT normals cancel each other out, resulting in zero avg normal.
    }

    ContactRuntime runtimeData = makeRuntimeData(body, collider);
    Contact contact(bodyH, runtimeData, avgNormal);
    Contact* contactPtr = collisionManifold->boxMesh(contact, *contactCache, SAT_resultsList);
    batch.contacts.push_back(contactPtr);
}

//===============================================
//     Tris vs Sphere
//===============================================
void NarrowphaseManager::processTerrainTriSphere(
    ContactBatch& batch,
    BodyHandle bodyH,
    Collider* collider,
    RigidBody* body,
    const std::vector<Tri*>& candidates)
{
    SAT_resultsList.clear();

    glm::vec3 pushOutSum{ 0.0f };
    float maxDepth = 0.0f;

    // SAT for each tri
    for (Tri* tri : candidates) {
        SAT::Result SAT_result;
        if (!SAT::sphereTri(*collider, *tri, SAT_result)) {
            continue;
        }

        glm::vec3 triNormal = glm::normalize(glm::cross(
            tri->vertices[1] - tri->vertices[0],
            tri->vertices[2] - tri->vertices[0]
        ));

        // Solver convention:
        // A = body, B = terrain, normal = A -> B.
        SAT_result.normal = -triNormal;
        SAT_result.feature.type = SAT::AxisType::TriFace;

        SAT_resultsList.push_back(SAT_result);

        float depth = std::max(SAT_result.depth, 0.0f);
        float w = std::max(depth, 0.0001f);

        // Pushout direction is terrain -> body.
        pushOutSum += triNormal * w;
        maxDepth = std::max(maxDepth, depth);
    }

    if (SAT_resultsList.empty()) {
        return;
    }

    body->totalCollisionCount++;

    glm::vec3 avgSolverNormal = getAvgNormal(SAT_resultsList);

    // Export to character controller
    if (body->motionControl == MotionControl::External)
    {
        glm::vec3 avgPushOutNormal;
        if (glm::length2(pushOutSum) > 1e-10f) {
            avgPushOutNormal = glm::normalize(pushOutSum);
        }
        else {
            avgPushOutNormal = -avgSolverNormal;
        }

        externalContacts.emplace_back(
            collider->rigidBodyHandle,
            BodyHandle{},
            -avgPushOutNormal,
            maxDepth,
            true // terrainContact
        );

        return;
    }

    SAT::findBestTriangles(SAT_resultsList);

    ContactRuntime runtimeData = makeRuntimeData(body, collider);

    Contact contact(bodyH, runtimeData, avgSolverNormal);
    Contact* contactPtr =
        collisionManifold->sphereMesh(contact, *contactCache, SAT_resultsList);

    batch.contacts.push_back(contactPtr);
}

//===============================================
//     Helper functions
//===============================================
glm::vec3 NarrowphaseManager::getAvgNormal(const std::vector<SAT::Result>& results) const {
    glm::vec3 avgNormal(0.0f);

    for (const SAT::Result& res : results) {
        avgNormal += res.normal;
    }

    float len2 = glm::dot(avgNormal, avgNormal);
    if (len2 < 1e-8f) {
        return glm::vec3(0.0f);
    }

    return glm::normalize(avgNormal);
}


}
