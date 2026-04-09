#include "pch.h"
#include "narrowphase_manager.h"

#define FUNC_NAME __FUNCTION__

void NarrowphaseManager::init(
    CollisionManifold* collisionManifold,
    std::unordered_map<size_t, Contact>* contactCache,
    RuntimeCaches* caches
) {
    this->collisionManifold = collisionManifold;
    this->contactCache = contactCache;
    this->caches = caches;
}

void NarrowphaseManager::narrowPhase(const std::vector<TerrainPair>& terrainHits, const std::vector<DynamicPair>& dynamicHits) {
    externalContacts.clear();

    for (const TerrainPair& pair : terrainHits) {
        processTerrainPair(pair);
    }

    for (const DynamicPair& pair : dynamicHits) {
        processDynamicPair(pair);
    }
}

void NarrowphaseManager::processTerrainPair(const TerrainPair& pair) {
    RigidBody* body = caches->bodies.get(pair.body, FUNC_NAME);
    if (body->asleep || body->type == BodyType::Kinematic) return;

    for (const ColliderHandle& colH : body->colliderHandles) {
        Collider* collider = caches->colliders.get(colH, FUNC_NAME);
        Transform* globalColliderTransform = &caches->globalColliderTransforms[colH.slot];

        terrainTriCandidates.clear();
        terrainTriCandidates.reserve(pair.tris.size());
        collectTerrainTriCandidates(collider, pair.tris, terrainTriCandidates);

        if (terrainTriCandidates.empty()) {
            continue;
        }

        SAT_resultsList.clear();
        SAT_resultsList.reserve(terrainTriCandidates.size());

        switch (collider->type) {
        case ColliderType::CUBOID:
            processTerrainTriBox(collider->rigidBodyHandle, collider, body, globalColliderTransform);
            break;

        case ColliderType::SPHERE:
            processTerrainTriSphere(collider->rigidBodyHandle, collider , body, globalColliderTransform);
            break;
        }
    }
}

void NarrowphaseManager::collectTerrainTriCandidates(
    Collider* collider, 
    const std::vector<Tri*>& inputTris,
    std::vector<Tri*>& outCandidates) 
{
    outCandidates.clear();

    const AABB& colAABB = collider->getAABB();
    for (Tri* tri : inputTris) {
        if (tri->getAABB().intersects(colAABB)) {
            outCandidates.push_back(tri);
        }
    }
}

void NarrowphaseManager::processDynamicPair(const DynamicPair& pair) {
    //RigidBody* bodyA = bodyCache->get(pair.bodyA, FUNC_NAME);
    //RigidBody* bodyB = bodyCache->get(pair.bodyB, FUNC_NAME);

    //Collider* colliderA = colliderCache->get(bodyA->colliderHandle, FUNC_NAME);
    //Collider* colliderB = colliderCache->get(bodyB->colliderHandle, FUNC_NAME);

    //if ((bodyA->type == BodyType::Static || bodyA->type == BodyType::Kinematic) and
    //    (bodyB->type == BodyType::Static || bodyB->type == BodyType::Kinematic)) 
    //{
    //    return;
    //}

    //GameObject* objA = gameObjectCache->get(bodyA->gameObjectHandle, FUNC_NAME);
    //GameObject* objB = gameObjectCache->get(bodyB->gameObjectHandle, FUNC_NAME);

    ////if (colliderA->type == ColliderType::CUBOID and colliderB->type == ColliderType::CUBOID) {
    ////    processBoxBox();
    ////}
    ////else if ((colliderA->type == ColliderType::CUBOID and colliderB->type == ColliderType::SPHERE) or
    ////    (colliderA->type == ColliderType::SPHERE and colliderB->type == ColliderType::CUBOID)) {
    ////    processBoxSphere();
    ////}
    ////else if (colliderA->type == ColliderType::SPHERE and colliderB->type == ColliderType::SPHERE) {
    ////    processSphereSphere();
    ////}
}

void NarrowphaseManager::processTerrainTriBox(RigidBodyHandle bodyH, Collider* collider, RigidBody* body, Transform* colliderWorldTransform) {
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
    ContactRuntime runtimeData = makeRuntimeData(body, collider, bodyRootTransform, colliderWorldTransform);
    Contact contact(bodyH, runtimeData, avgNormal);
    collisionManifold->boxMesh(contact, *contactCache, SAT_resultsList);
}

void NarrowphaseManager::processTerrainTriSphere(RigidBodyHandle bodyH, Collider* collider, RigidBody* body, Transform* colliderWorldTransform) {
    // SAT for each tri
    for (Tri* tri : terrainTriCandidates) {
        SAT::Result SAT_result;
        if (!SAT::sphereTri(*collider, *tri, SAT_result)) {
            continue;
        }

        SAT::reverseNormal(colliderWorldTransform->position, tri->centroid, SAT_result.normal);
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
    ContactRuntime runtimeData = makeRuntimeData(body, collider, bodyRootTransform, colliderWorldTransform);
    Contact contact(bodyH, runtimeData, avgNormal);
    collisionManifold->sphereMesh(contact, *contactCache, SAT_resultsList);
}

void NarrowphaseManager::processBoxBox(RigidBody* bodyA, RigidBody* bodyB, Collider* colliderA, Collider* colliderB, Transform* transformA, Transform* transformB) {
    // TODO
}

void NarrowphaseManager::processBoxSphere(RigidBody* bodyA, RigidBody* bodyB, Collider* colliderA, Collider* colliderB, Transform* transformA, Transform* transformB) {
    // TODO
}

void NarrowphaseManager::processSphereSphere(RigidBody* bodyA, RigidBody* bodyB, Collider* colliderA, Collider* colliderB, Transform* transformA, Transform* transformB) {
    // TODO
}

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


ContactRuntime NarrowphaseManager::makeRuntimeData(
    RigidBody* bodyA, RigidBody* bodyB,
    Collider* colliderA, Collider* colliderB,
    Transform* bodyRootA, Transform* bodyRootB,
    Transform* colliderWorldA, Transform* colliderWorldB) const
{
    ContactRuntime rt;
    rt.bodyA = bodyA;
    rt.bodyB = bodyB;
    rt.colliderA = colliderA;
    rt.colliderB = colliderB;
    rt.bodyRootA = bodyRootA;
    rt.bodyRootB = bodyRootB;
    rt.colliderWorldA = colliderWorldA;
    rt.colliderWorldB = colliderWorldB;
    return rt;
}

ContactRuntime NarrowphaseManager::makeRuntimeData(
    RigidBody* bodyA,
    Collider* colliderA,
    Transform* bodyRootA,
    Transform* colliderWorldA) const
{
    ContactRuntime rt;
    rt.bodyA = bodyA;
    rt.colliderA = colliderA;
    rt.bodyRootA = bodyRootA;
    rt.colliderWorldA = colliderWorldA;
    return rt;
}