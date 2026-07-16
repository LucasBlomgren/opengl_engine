#include "pch.h"
#include "narrowphase_types.h"
#include "collision_manifold.h"

void ContactBatch::sortByMinY() {
    std::sort(contacts.begin(), contacts.end(),
        [](const Contact* a, const Contact* b) {
            if (a->minY < b->minY) return true;
            if (b->minY < a->minY) return false;
            return a->hashKey < b->hashKey;
        });
}