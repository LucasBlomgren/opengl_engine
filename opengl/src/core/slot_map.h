#pragma once
#include <cstdint>
#include <vector>
#include <utility>

template<class Tag>
struct HandleT {
    uint32_t slot = 0;
    uint32_t gen = 0;
};

using GameObjectHandle = HandleT<struct GameObjectTag>;
using ColliderHandle = HandleT<struct ColliderTag>;

template<class T, class Id>
class SlotMap {
public:
    static constexpr uint32_t INVALID = 0xFFFFFFFFu;

    bool alive(Id h) const {
        return h.slot < slotGen_.size() && slotGen_[h.slot] == h.gen && slotToDense_[h.slot] != INVALID;
    }

    T* try_get(Id h) {
        if (!alive(h)) return nullptr;
        return &dense_[slotToDense_[h.slot]];
    }
    const T* try_get(Id h) const {
        if (!alive(h)) return nullptr;
        return &dense_[slotToDense_[h.slot]];
    }

    template<class... Args>
    Id create(Args&&... args) {
        uint32_t slot = alloc_slot_();
        uint32_t di = (uint32_t)dense_.size();

        dense_.emplace_back(std::forward<Args>(args)...);

        denseToSlot_.push_back(slot);
        slotToDense_[slot] = di;

        return Id{ slot, slotGen_[slot] };
    }

    void destroy(Id h) {
        if (!alive(h)) return;

        uint32_t di = slotToDense_[h.slot];
        uint32_t last = (uint32_t)dense_.size() - 1;

        if (di != last) {
            dense_[di] = std::move(dense_[last]);

            uint32_t movedSlot = denseToSlot_[last];
            denseToSlot_[di] = movedSlot;
            slotToDense_[movedSlot] = di;
        }

        dense_.pop_back();
        denseToSlot_.pop_back();

        slotToDense_[h.slot] = INVALID;
        slotGen_[h.slot]++;              // invalidate old handles (ABA-safe)
        freeSlots_.push_back(h.slot);
    }

    // Dense iteration (hot paths: broadphase, solver, etc.)
    std::vector<T>& dense() { return dense_; }
    const std::vector<T>& dense() const { return dense_; }

private:
    uint32_t alloc_slot_() {
        if (!freeSlots_.empty()) {
            uint32_t slot = freeSlots_.back();
            freeSlots_.pop_back();
            return slot;
        }
        uint32_t slot = (uint32_t)slotGen_.size();
        slotGen_.push_back(0);
        slotToDense_.push_back(INVALID);
        return slot;
    }

    std::vector<uint32_t> slotGen_;      // slot -> gen
    std::vector<uint32_t> slotToDense_;  // slot -> dense index
    std::vector<uint32_t> denseToSlot_;  // dense index -> slot (needed for swap-pop remap)
    std::vector<uint32_t> freeSlots_;    // reusable slots
    std::vector<T>        dense_;        // dense storage
};