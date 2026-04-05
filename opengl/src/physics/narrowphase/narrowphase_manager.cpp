#include "pch.h"
#include "narrowphase_manager.h"

#define FUNC_NAME __FUNCTION__

void NarrowphaseManager::init(
    CollisionManifold* collisionManifold,
    std::unordered_map<size_t, Contact>* contactCache,
    PointerCache<GameObject, GameObjectHandle>* gameObjectPtrCache,
    PointerCache<Collider, ColliderHandle>* colliderPtrCache,
    PointerCache<RigidBody, RigidBodyHandle>* bodyPtrCache
) {
    this->collisionManifold = collisionManifold;
    this->contactCache = contactCache;
    this->gameObjectCache = gameObjectPtrCache;
    this->colliderCache = colliderPtrCache;
    this->bodyCache = bodyPtrCache;
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
    RigidBody* body = bodyCache->get(pair.body, FUNC_NAME);
    Collider* collider = colliderCache->get(body->colliderHandle, FUNC_NAME);

    if (body->asleep || body->type == BodyType::Kinematic) {
        return;
    }

    GameObject* obj = gameObjectCache->get(body->gameObjectHandle, FUNC_NAME);

    int reserveSize = pair.tris.size() * 2; // worst case
    SAT_resultsList.clear();
    SAT_resultsList.reserve(reserveSize);

    switch (collider->type) {
    case ColliderType::CUBOID:
        processTerrainBox(pair, collider, body, obj);
        break;

    case ColliderType::SPHERE:
        processTerrainSphere(pair, collider, body, obj);
        break;

    default:
        break;
    }
}

void NarrowphaseManager::processDynamicPair(const DynamicPair& pair) {
    RigidBody* bodyA = bodyCache->get(pair.bodyA, FUNC_NAME);
    RigidBody* bodyB = bodyCache->get(pair.bodyB, FUNC_NAME);

    Collider* colliderA = colliderCache->get(bodyA->colliderHandle, FUNC_NAME);
    Collider* colliderB = colliderCache->get(bodyB->colliderHandle, FUNC_NAME);

    if ((bodyA->type == BodyType::Static || bodyA->type == BodyType::Kinematic) and
        (bodyB->type == BodyType::Static || bodyB->type == BodyType::Kinematic)) 
    {
        return;
    }

    GameObject* objA = gameObjectCache->get(bodyA->gameObjectHandle, FUNC_NAME);
    GameObject* objB = gameObjectCache->get(bodyB->gameObjectHandle, FUNC_NAME);

    //if (colliderA->type == ColliderType::CUBOID and colliderB->type == ColliderType::CUBOID) {
    //    processBoxBox();
    //}
    //else if ((colliderA->type == ColliderType::CUBOID and colliderB->type == ColliderType::SPHERE) or
    //    (colliderA->type == ColliderType::SPHERE and colliderB->type == ColliderType::CUBOID)) {
    //    processBoxSphere();
    //}
    //else if (colliderA->type == ColliderType::SPHERE and colliderB->type == ColliderType::SPHERE) {
    //    processSphereSphere();
    //}
}

void NarrowphaseManager::processTerrainBox(const TerrainPair& pair, Collider* collider, RigidBody* body, GameObject* obj) {
    // SAT for each tri
    for (Tri* tri : pair.tris) {
        SAT::Result SAT_result;
        if (!SAT::boxTri(*collider, *tri, SAT_result)) {
            continue;
        }

        glm::vec3& centerBox = std::get<OOBB>(collider->shape).centerWorld;
        SAT::reverseNormal(centerBox, SAT_result.tri_ptr->centroid, SAT_result.normal);
        SAT_resultsList.push_back(SAT_result);
    }

    if (SAT_resultsList.size() == 0) return;

    body->updateInertiaWorld(obj->transform);

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

    ContactRuntime runtimeData = makeRuntimeData(body, collider, obj);
    Contact contact(pair.body, runtimeData, avgNormal);
    collisionManifold->boxMesh(contact, *contactCache, SAT_resultsList);
}

void NarrowphaseManager::processTerrainSphere(const TerrainPair& pair, Collider* collider, RigidBody* body, GameObject* obj) {
    // SAT for each tri
    for (Tri* tri : pair.tris) {
        SAT::Result SAT_result;
        if (!SAT::sphereTri(*collider, *tri, SAT_result)) {
            continue;
        }

        SAT::reverseNormal(obj->transform.position, tri->centroid, SAT_result.normal);
        SAT_resultsList.push_back(SAT_result);
    }

    if (SAT_resultsList.size() == 0) return;

    body->updateInertiaWorld(obj->transform);

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

    ContactRuntime runtimeData = makeRuntimeData(body, collider, obj);
    Contact contact(pair.body, runtimeData, avgNormal);
    collisionManifold->sphereMesh(contact, *contactCache, SAT_resultsList);
}

void NarrowphaseManager::processBoxBox(RigidBody* bodyA, RigidBody* bodyB, Collider* colliderA, Collider* colliderB, GameObject* objA, GameObject* objB) {
    // TODO
}

void NarrowphaseManager::processBoxSphere(RigidBody* bodyA, RigidBody* bodyB, Collider* colliderA, Collider* colliderB, GameObject* objA, GameObject* objB) {
    // TODO
}

void NarrowphaseManager::processSphereSphere(RigidBody* bodyA, RigidBody* bodyB, Collider* colliderA, Collider* colliderB, GameObject* objA, GameObject* objB) {
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
    GameObject* objA, GameObject* objB) const
{
    ContactRuntime rt;
    rt.bodyA = bodyA;
    rt.bodyB = bodyB;
    rt.colliderA = colliderA;
    rt.colliderB = colliderB;
    rt.objA = objA;
    rt.objB = objB;
    return rt;
}

ContactRuntime NarrowphaseManager::makeRuntimeData(
    RigidBody* bodyA,
    Collider* colliderA,
    GameObject* objA) const
{
    ContactRuntime rt;
    rt.bodyA = bodyA;
    rt.colliderA = colliderA;
    rt.objA = objA;
    return rt;
}