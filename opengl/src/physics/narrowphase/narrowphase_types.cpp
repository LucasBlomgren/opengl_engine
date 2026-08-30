#include "physics/narrowphase/narrowphase_types.h"
#include "physics/narrowphase/collision_manifold.h"

#include <algorithm>

namespace physics::internal {

    void ContactBatch::sortByMinY()
    {
        std::sort(
            contacts.begin(),
            contacts.end(),
            [](const Contact* a, const Contact* b) {
                if (a->minY < b->minY) return true;
                if (b->minY < a->minY) return false;
                return a->hashKey < b->hashKey;
            }
        );
    }

}