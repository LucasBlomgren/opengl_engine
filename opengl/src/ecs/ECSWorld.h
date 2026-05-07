#pragma once
#include <type_traits>

#include "entity_pool.h"
#include "component_storage.h"

#include "../components/transforcomponent.h"
#include "../components/render_component.h"
#include "../components/name_component.h"
#include "../components/parent_component.h"
#include "../components/physics/rigid_body_component.h"
#include "../components/physics/collider_component.h"
#include "../components/physics/physics_link.h"

class ECSWorld {
public:
    Entity createEntity() {
        Entity e = entities.create();

        if (e.id >= signatures.size()) {
            signatures.resize(e.id + 1, 0);
        }

        signatures[e.id] = 0;
        return e;
    }

    void destroyEntity(Entity e) {
        if (!entities.isAlive(e))
            return;

        removeAllComponents(e);

        signatures[e.id] = 0;
        entities.destroy(e);
    }

    bool isAlive(Entity e) const {
        return entities.isAlive(e);
    }

    template<typename T, typename... Args>
    T& add(Entity e, Args&&... args) {
        auto& s = storage<T>();
        T& component = s.add(e, T{ std::forward<Args>(args)... });

        // optional senare:
        // signatures[e.id] |= componentBit<T>();

        return component;
    }

    template<typename T>
    bool has(Entity e) const {
        if (!entities.isAlive(e))
            return false;

        return storage<T>().has(e);
    }

    template<typename T>
    T* try_get(Entity e) {
        if (!entities.isAlive(e))
            return nullptr;

        return storage<T>().try_get(e);
    }

    template<typename T>
    T& get(Entity e) {
        return *try_get<T>(e);
    }

    template<typename T>
    void remove(Entity e) {
        if (!entities.isAlive(e))
            return;

        storage<T>().remove(e);

        // optional senare:
        // signatures[e.id] &= ~componentBit<T>();
    }

    template<typename T>
    ComponentStorage<T>& storage() {
        if constexpr (std::is_same_v<T, TransformComponent>)
            return transforms;
        else if constexpr (std::is_same_v<T, RenderComponent>)
            return renders;
        else if constexpr (std::is_same_v<T, NameComponent>)
            return names;
        else if constexpr (std::is_same_v<T, ParentComponent>)
            return parents;
        else if constexpr (std::is_same_v<T, RigidBodyComponent>)
            return rigidBodies;
        else if constexpr (std::is_same_v<T, ColliderComponent>)
            return colliders;
        else if constexpr (std::is_same_v<T, PhysicsLink>)
            return physicsLinks;
    }

    template<typename T>
    const ComponentStorage<T>& storage() const {
        if constexpr (std::is_same_v<T, TransformComponent>)
            return transforms;
        else if constexpr (std::is_same_v<T, RenderComponent>)
            return renders;
        else if constexpr (std::is_same_v<T, NameComponent>)
            return names;
        else if constexpr (std::is_same_v<T, ParentComponent>)
            return parents;
        else if constexpr (std::is_same_v<T, RigidBodyComponent>)
            return rigidBodies;
        else if constexpr (std::is_same_v<T, ColliderComponent>)
            return colliders;
        else if constexpr (std::is_same_v<T, PhysicsLink>)
            return physicsLinks;
    }

private:
    void removeAllComponents(Entity e) {
        transforms.remove(e);
        renders.remove(e);
        names.remove(e);
        parents.remove(e);
        rigidBodies.remove(e);
        colliders.remove(e);
        physicsLinks.remove(e);
    }

private:
    EntityPool entities;

    ComponentStorage<TransformComponent> transforms;
    ComponentStorage<RenderComponent> renders;
    ComponentStorage<NameComponent> names;
    ComponentStorage<ParentComponent> parents;

    ComponentStorage<RigidBodyComponent> rigidBodies;
    ComponentStorage<ColliderComponent> colliders;
    ComponentStorage<PhysicsLink> physicsLinks;

    using ComponentMask = uint64_t;
    std::vector<ComponentMask> signatures;
};