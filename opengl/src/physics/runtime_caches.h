#pragma once

#include "core/pointer_cache.h"
#include "colliders/collider.h"
#include "rigidbody.h"

struct RuntimeCaches { 
    PointerCache<Transform, TransformHandle> transforms; 
    PointerCache<Collider, ColliderHandle> colliders; 
    PointerCache<RigidBody, RigidBodyHandle> bodies; 
    
    void clear() { 
        transforms.clear(); 
        colliders.clear(); 
        bodies.clear(); 
    } 
};