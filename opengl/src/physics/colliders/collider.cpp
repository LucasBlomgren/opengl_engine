#include "collider.h"
#include "game_object.h"

AABB Collider::getAABB() const {
    return owner->aabb;
}