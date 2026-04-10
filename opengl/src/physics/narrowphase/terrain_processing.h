#pragma once
#include "narrowphase_manager.h"

#define FUNC_NAME __FUNCTION__

void NarrowphaseManager::processTerrainTriBox(RigidBodyHandle bodyH, Collider* collider, RigidBody* body) {
    // SAT for each tri
    for (Tri* tri : terrainTriCandidates) {
        SAT::Result SAT_result;
        if (!SAT::boxTri(*collider, *tri, SAT_result)) {
            continue;
        }

        glm::vec3& centerBox = std::get<OOBB>(collider->shape).worldCenter;
        SAT::reverseNormal(centerBox, SAT_result.tri_ptr->centroid, SAT_result.normal);
        SAT_resultsList.push_back(SAT_result);
    }

    if (SAT_resultsList.size() == 0) return;

    // export to character controller
    if (body->motionControl == MotionControl::External) {
        externalContacts.emplace_back(
            collider->rigidBodyHandle,
            RigidBodyHandle{},
            SAT_resultsList[0].normal,
            SAT_resultsList[0].depth
        );
        return;
    }

    // collision manifold generation
    glm::vec3 avgNormal = getAvgNormal(SAT_resultsList);
    SAT::findBestTriangles(SAT_resultsList);

    Transform* bodyRootTransform = caches->transforms.get(body->rootTransformHandle, FUNC_NAME);
    ContactRuntime runtimeData = makeRuntimeData(body, collider, bodyRootTransform);
    Contact contact(bodyH, runtimeData, avgNormal);
    collisionManifold->boxMesh(contact, *contactCache, SAT_resultsList);
}

void NarrowphaseManager::processTerrainTriSphere(RigidBodyHandle bodyH, Collider* collider, RigidBody* body) {
    // SAT for each tri
    for (Tri* tri : terrainTriCandidates) {
        SAT::Result SAT_result;
        if (!SAT::sphereTri(*collider, *tri, SAT_result)) {
            continue;
        }

        SAT::reverseNormal(collider->globalTransform.position, tri->centroid, SAT_result.normal);
        SAT_resultsList.push_back(SAT_result);
    }

    if (SAT_resultsList.size() == 0) return;

    // export to character controller
    if (body->motionControl == MotionControl::External) {
        externalContacts.emplace_back(
            collider->rigidBodyHandle,
            RigidBodyHandle{},
            SAT_resultsList[0].normal,
            SAT_resultsList[0].depth
        );

        return;
    }

    // collision manifold generation
    glm::vec3 avgNormal = getAvgNormal(SAT_resultsList);
    SAT::findBestTriangles(SAT_resultsList);

    Transform* bodyRootTransform = caches->transforms.get(body->rootTransformHandle, FUNC_NAME);
    ContactRuntime runtimeData = makeRuntimeData(body, collider, bodyRootTransform);
    Contact contact(bodyH, runtimeData, avgNormal);
    collisionManifold->sphereMesh(contact, *contactCache, SAT_resultsList);
}

//-----------------------------------------------
//     Helper functions
//-----------------------------------------------
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