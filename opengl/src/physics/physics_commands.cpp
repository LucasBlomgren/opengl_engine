#include "pch.h"
#include "physics_engine.h"

//====================================
//    BVH 
//====================================
void PhysicsEngine::setBVHDirty(RigidBodyHandle& handle) {
    broadphaseManager.setBVHDirty(handle);
}
void PhysicsEngine::updateBVHRenderData(const BVHType& type, bool update) {
    broadphaseManager.updateBVHRenderData(type, update);
}

//====================================
//     Add/Remove commands
//====================================
void PhysicsEngine::queueAdd(RigidBodyHandle& handle, BroadphaseBucket& target) {
    //std::cout << "[Physics::QueueAdd]: Add object with handle: slot " << handle.slot << ", gen " << handle.gen << " to bucket " << static_cast<int>(target) << "\n";
    pending.push_back({ PhysCmd::Type::Add, handle, target });
}
void PhysicsEngine::queueRemove(RigidBodyHandle& handle) {
    //std::cout << "[Physics::QueueRemove]: Remove object with handle: slot " << handle.slot << ", gen " << handle.gen << "\n";
    pending.push_back({ PhysCmd::Type::Remove, handle, BroadphaseBucket::None });
}
void PhysicsEngine::queueMove(RigidBodyHandle& handle, BroadphaseBucket& target) {
    //std::cout << "[Physics::QueueMove]: Move object with handle: slot " << handle.slot << ", gen " << handle.gen << " to bucket " << static_cast<int>(target) << "\n";
    pending.push_back({ PhysCmd::Type::Move, handle, target });
}

//======================================
//     Flush pending commands
//======================================
void PhysicsEngine::flushBroadphaseCommands() {
    for (auto& cmd : pending) {
        switch (cmd.type) {
        case PhysCmd::Type::Add:
            broadphaseManager.add(cmd.handle, cmd.dst);
            break;

        case PhysCmd::Type::Remove:
            broadphaseManager.remove(cmd.handle);
            break;

        case PhysCmd::Type::Move:
            switch (cmd.dst) {
            case BroadphaseBucket::Awake:  broadphaseManager.moveToAwake(cmd.handle);  break;
            case BroadphaseBucket::Asleep: broadphaseManager.moveToAsleep(cmd.handle); break;
            case BroadphaseBucket::Static: broadphaseManager.moveToStatic(cmd.handle); break;
            default: break;
            }
            break;
        }
    }
    pending.clear();
}

//====================================
//         Sleep Commands
//====================================
void PhysicsEngine::sleepAllObjects() {
    auto& bodyMap = physicsWorld.getRigidBodiesMap();
    auto& dense = bodyMap.dense();

    for (uint32_t i = 0; i < (uint32_t)dense.size(); ++i) {
        RigidBody& body = dense[i];

        if (body.asleep) continue;
        if (body.type == BodyType::Static) continue;
        if (body.type == BodyType::Kinematic) continue;
        if (body.motionControl == MotionControl::External) continue;

        RigidBodyHandle handle = bodyMap.handle_from_dense_index(i);
        broadphaseManager.moveToAsleep(handle);
    }
}
void PhysicsEngine::awakenAllObjects() {
    auto& bodyMap = physicsWorld.getRigidBodiesMap();
    auto& dense = bodyMap.dense();

    for (uint32_t i = 0; i < (uint32_t)dense.size(); ++i) {
        RigidBody& body = dense[i];

        if (!body.asleep) continue;
        if (body.type == BodyType::Static) continue;
        if (body.type == BodyType::Kinematic) continue;
        if (body.motionControl == MotionControl::External) continue;

        RigidBodyHandle handle = bodyMap.handle_from_dense_index(i);
        broadphaseManager.moveToAwake(handle);
    }
}