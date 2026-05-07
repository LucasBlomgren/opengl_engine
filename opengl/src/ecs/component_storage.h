#pragma once

#include <vector>
#include <cstdint>
#include <utility>
#include <cassert>

#include "entity.h"

template<typename T>
class ComponentStorage {
public:
    static constexpr uint32_t INVALID = 0xFFFFFFFFu;

public:
    bool has(Entity e) const {
        if (!e.isValid())
            return false;

        if (e.id >= entityToDense.size())
            return false;

        uint32_t denseIndex = entityToDense[e.id];

        if (denseIndex == INVALID)
            return false;

        if (denseIndex >= dense.size())
            return false;

        // Viktigt: jämför både id och generation.
        return denseEntities[denseIndex] == e;
    }

    T* try_get(Entity e) {
        if (!has(e))
            return nullptr;

        return &dense[entityToDense[e.id]];
    }

    const T* try_get(Entity e) const {
        if (!has(e))
            return nullptr;

        return &dense[entityToDense[e.id]];
    }

    T& get(Entity e) {
        assert(has(e));
        return dense[entityToDense[e.id]];
    }

    const T& get(Entity e) const {
        assert(has(e));
        return dense[entityToDense[e.id]];
    }

    template<typename... Args>
    T& emplace(Entity e, Args&&... args) {
        assert(e.isValid());

        if (e.id >= entityToDense.size()) {
            entityToDense.resize(e.id + 1, INVALID);
        }

        // Om komponenten redan finns, returnera den.
        // Alternativt kan du assert:a här om du inte vill tillåta duplicate add.
        if (has(e)) {
            return get(e);
        }

        uint32_t denseIndex = static_cast<uint32_t>(dense.size());

        dense.emplace_back(std::forward<Args>(args)...);
        denseEntities.push_back(e);
        entityToDense[e.id] = denseIndex;

        return dense.back();
    }

    void remove(Entity e) {
        if (!has(e))
            return;

        uint32_t removeIndex = entityToDense[e.id];
        uint32_t lastIndex = static_cast<uint32_t>(dense.size() - 1);

        Entity movedEntity = denseEntities[lastIndex];

        if (removeIndex != lastIndex) {
            dense[removeIndex] = std::move(dense[lastIndex]);
            denseEntities[removeIndex] = movedEntity;

            entityToDense[movedEntity.id] = removeIndex;
        }

        dense.pop_back();
        denseEntities.pop_back();

        entityToDense[e.id] = INVALID;
    }

    void clear() {
        dense.clear();
        denseEntities.clear();
        entityToDense.clear();
    }

    uint32_t size() const {
        return static_cast<uint32_t>(dense.size());
    }

    bool empty() const {
        return dense.empty();
    }

    std::vector<T>& components() {
        return dense;
    }

    const std::vector<T>& components() const {
        return dense;
    }

    std::vector<Entity>& entities() {
        return denseEntities;
    }

    const std::vector<Entity>& entities() const {
        return denseEntities;
    }

private:
    std::vector<T> dense;
    std::vector<Entity> denseEntities;
    std::vector<uint32_t> entityToDense;
};