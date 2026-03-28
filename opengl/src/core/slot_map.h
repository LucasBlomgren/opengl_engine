#pragma once
#include <cstdint>
#include <vector>
#include <utility>

template<class Tag>
struct HandleT {
    uint32_t slot = 0xFFFFFFFFu;
    uint32_t gen = 0xFFFFFFFFu;

    bool isValid() const {
        return slot != 0xFFFFFFFFu;
    }

    friend bool operator==(const HandleT& a, const HandleT& b) noexcept {
        return a.slot == b.slot && a.gen == b.gen;
    }
};

namespace std {
    template<class Tag>
    struct hash<HandleT<Tag>> {
        size_t operator()(const HandleT<Tag>& h) const noexcept {
            // kombinera slot+gen till 64-bit och hash:a
            uint64_t x = (uint64_t(h.gen) << 32) | uint64_t(h.slot);
            return std::hash<uint64_t>{}(x);
        }
    };
}

using GameObjectHandle = HandleT<struct GameObjectTag>;
using ColliderHandle = HandleT<struct ColliderTag>;
using RigidBodyHandle = HandleT<struct RigidBodyTag>;

template<class T, class Id>
class SlotMap {
public:
    static constexpr uint32_t INVALID = 0xFFFFFFFFu;

    SlotMap() {
        m_slotGen.reserve(100000);
        m_slotToDense.reserve(100000);
        m_denseToSlot.reserve(100000);
        m_freeSlots.reserve(100000);
        m_dense.reserve(100000);
    }

    bool alive(Id h) const {
        return h.slot < m_slotGen.size() && m_slotGen[h.slot] == h.gen && m_slotToDense[h.slot] != INVALID;
    }

    T* try_get(Id h) {
        if (!alive(h)) return nullptr;
        return &m_dense[m_slotToDense[h.slot]];
    }
    const T* try_get(Id h) const {
        if (!alive(h)) return nullptr;
        return &m_dense[m_slotToDense[h.slot]];
    }

    Id handle_from_dense_index(uint32_t denseIndex) const {
        uint32_t slot = m_denseToSlot[denseIndex];
        return Id{ slot, m_slotGen[slot] };
    }

    uint32_t slot_capacity() const {
        return (uint32_t)m_slotGen.size();
    }

    template<class... Args>
    Id create(Args&&... args) {
        uint32_t slot = alloc_slot_();
        uint32_t di = (uint32_t)m_dense.size();

        m_dense.emplace_back(std::forward<Args>(args)...);

        m_denseToSlot.push_back(slot);
        m_slotToDense[slot] = di;

        return Id{ slot, m_slotGen[slot] };
    }

    void destroy(Id h) {
        if (!alive(h)) return;

        uint32_t di = m_slotToDense[h.slot];
        uint32_t last = (uint32_t)m_dense.size() - 1;

        if (di != last) {
            m_dense[di] = std::move(m_dense[last]);

            uint32_t movedSlot = m_denseToSlot[last];
            m_denseToSlot[di] = movedSlot;
            m_slotToDense[movedSlot] = di;
        }

        m_dense.pop_back();
        m_denseToSlot.pop_back();

        m_slotToDense[h.slot] = INVALID;
        m_slotGen[h.slot]++;              // invalidate old handles (ABA-safe)
        m_freeSlots.push_back(h.slot);
    }

    // Dense iteration (hot paths: broadphase, solver, etc.)
    std::vector<T>& dense() { return m_dense; }
    const std::vector<T>& dense() const { return m_dense; }

private:
    uint32_t alloc_slot_() {
        if (!m_freeSlots.empty()) {
            uint32_t slot = m_freeSlots.back();
            m_freeSlots.pop_back();
            return slot;
        }
        uint32_t slot = (uint32_t)m_slotGen.size();
        m_slotGen.push_back(0);
        m_slotToDense.push_back(INVALID);
        return slot;
    }

    std::vector<uint32_t> m_slotGen;      // slot -> gen
    std::vector<uint32_t> m_slotToDense;  // slot -> dense index
    std::vector<uint32_t> m_denseToSlot;  // dense index -> slot (needed for swap-pop remap)
    std::vector<uint32_t> m_freeSlots;    // reusable slots
    std::vector<T>        m_dense;        // dense storage
};