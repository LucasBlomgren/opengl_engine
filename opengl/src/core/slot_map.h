#pragma once

#include <cstdint>
#include <utility>
#include <vector>

template<class T, class Id>
class SlotMap {
public:
    static constexpr uint32_t INVALID = 0xFFFFFFFFu;

    SlotMap() {
        slotGen.reserve(100000);
        slotToDense.reserve(100000);
        denseToSlot.reserve(100000);
        freeSlots.reserve(100000);
        denseStorage.reserve(100000);
    }

    bool alive(Id handle) const {
        return handle.slot < slotGen.size() && 
            slotGen[handle.slot] == handle.gen && 
            slotToDense[handle.slot] != INVALID;
    }

    T* get(Id handle) {
        return &denseStorage[slotToDense[handle.slot]];
    }

    const T* get(Id handle) const {
        return &denseStorage[slotToDense[handle.slot]];
    }

    T* try_get(Id handle) {
        if (!alive(handle)) return nullptr;
        return &denseStorage[slotToDense[handle.slot]];
    }

    const T* try_get(Id handle) const {
        if (!alive(handle)) return nullptr;
        return &denseStorage[slotToDense[handle.slot]];
    }

    Id handle_from_dense_index(uint32_t denseIndex) const {
        uint32_t slot = denseToSlot[denseIndex];
        return Id{ slot, slotGen[slot] };
    }

    uint32_t slot_capacity() const {
        return static_cast<uint32_t>(slotGen.size());
    }

    template<class... Args>
    Id create(Args&&... args) {
        Id handle = reserve();
        create_reserved(handle, std::forward<Args>(args)...);
        return handle;
    }

    Id reserve() {
        uint32_t slot = allocSlot();
        return Id{ slot, slotGen[slot] };
    }

    template<class... Args>
    T* create_reserved(Id handle, Args&&... args) {
        if (handle.slot >= slotGen.size() ||
            slotGen[handle.slot] != handle.gen ||
            slotToDense[handle.slot] != INVALID) {
            return nullptr;
        }

        uint32_t denseIndex = static_cast<uint32_t>(denseStorage.size());
        denseStorage.emplace_back(std::forward<Args>(args)...);

        denseToSlot.push_back(handle.slot);
        slotToDense[handle.slot] = denseIndex;

        return &denseStorage.back();
    }

    void release_reserved(Id handle) {
        if (handle.slot >= slotGen.size() ||
            slotGen[handle.slot] != handle.gen ||
            slotToDense[handle.slot] != INVALID) {
            return;
        }

        slotToDense[handle.slot] = INVALID;
        ++slotGen[handle.slot];
        freeSlots.push_back(handle.slot);
    }

    void destroy(Id handle) {
        if (!alive(handle)) return;

        uint32_t denseIndex = slotToDense[handle.slot];
        uint32_t lastDenseIndex = static_cast<uint32_t>(denseStorage.size()) - 1;

        if (denseIndex != lastDenseIndex) {
            denseStorage[denseIndex] = std::move(denseStorage[lastDenseIndex]);

            uint32_t movedSlot = denseToSlot[lastDenseIndex];
            denseToSlot[denseIndex] = movedSlot;
            slotToDense[movedSlot] = denseIndex;
        }

        denseStorage.pop_back();
        denseToSlot.pop_back();

        slotToDense[handle.slot] = INVALID;
        ++slotGen[handle.slot];
        freeSlots.push_back(handle.slot);
    }

    std::vector<T>& dense() {
        return denseStorage;
    }

    const std::vector<T>& dense() const {
        return denseStorage;
    }

    void clear() {
        slotGen.clear();
        slotToDense.clear();
        denseToSlot.clear();
        freeSlots.clear();
        denseStorage.clear();
    }

private:
    uint32_t allocSlot() {
        if (!freeSlots.empty()) {
            uint32_t slot = freeSlots.back();
            freeSlots.pop_back();
            return slot;
        }

        uint32_t slot = static_cast<uint32_t>(slotGen.size());

        slotGen.push_back(0);
        slotToDense.push_back(INVALID);

        return slot;
    }

    std::vector<uint32_t> slotGen;
    std::vector<uint32_t> slotToDense;
    std::vector<uint32_t> denseToSlot;
    std::vector<uint32_t> freeSlots;
    std::vector<T> denseStorage;
};
