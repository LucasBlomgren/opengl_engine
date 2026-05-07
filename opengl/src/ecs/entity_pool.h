#pragma once
#include <vector>
#include <cstdint>
#include "entity.h"

class EntityPool {
public:
    Entity create() {
        uint32_t id;

        // Reuse ID from free list if available, otherwise create a new one
        if (!freeList.empty()) {
            id = freeList.back();
            freeList.pop_back();

            alive[id] = true;
        }
        else {
            id = static_cast<uint32_t>(generations.size());

            generations.push_back(1);
            alive.push_back(true);
        }

        return Entity{
            .id = id,
            .generation = generations[id]
        };
    }

    void destroy(Entity e) {
        if (!isAlive(e))
            return;

        alive[e.id] = false;
        generations[e.id]++;

        freeList.push_back(e.id);
    }

    bool isAlive(Entity e) const {
        if (!e.isValid())
            return false;

        if (e.id >= generations.size())
            return false;

        return alive[e.id] &&
            generations[e.id] == e.generation;
    }

    uint32_t capacity() const {
        return static_cast<uint32_t>(generations.size());
    }

private:
    std::vector<uint32_t> generations;
    std::vector<bool> alive;
    std::vector<uint32_t> freeList;
};