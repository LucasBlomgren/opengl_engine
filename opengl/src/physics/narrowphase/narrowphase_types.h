#pragma once

#include <vector>
#include "collision_manifold.h"


struct ContactBatch {
    std::vector<Contact*> contacts;

    void clear() {
        contacts.clear();
    }

    size_t size() {
        return contacts.size();
    }

    void sortByMinY() {
        std::sort(contacts.begin(), contacts.end(),
            [](const Contact* a, const Contact* b) {
                if (a->minY < b->minY) return true;
                if (b->minY < a->minY) return false;
                return a->hashKey < b->hashKey;
            });
    }
};

struct ExternalMotionContact {
    RigidBodyHandle bodyA;
    RigidBodyHandle bodyB;
    glm::vec3 normal{ 0.0f };
    float penetration = 0.0f;

    ExternalMotionContact(const RigidBodyHandle& bodyA,
        const RigidBodyHandle& bodyB,
        const glm::vec3& normal,
        float penetration)
        : bodyA(bodyA), bodyB(bodyB), normal(normal), penetration(penetration) {}
    ExternalMotionContact() = default;
};