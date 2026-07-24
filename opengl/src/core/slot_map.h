#pragma once

#include <cstdint>
#include <utility>
#include <vector>

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

    bool alive(Id handle) const {
        return handle.slot < m_slotGen.size() && m_slotGen[handle.slot] == handle.gen && m_slotToDense[handle.slot] != INVALID;
    }

    T* get(Id handle) {
        return &m_dense[m_slotToDense[handle.slot]];
    }

    const T* get(Id handle) const {
        return &m_dense[m_slotToDense[handle.slot]];
    }

    T* try_get(Id handle) {
        if (!alive(handle)) return nullptr;
        return &m_dense[m_slotToDense[handle.slot]];
    }

    const T* try_get(Id handle) const {
        if (!alive(handle)) return nullptr;
        return &m_dense[m_slotToDense[handle.slot]];
    }

    Id handle_from_dense_index(uint32_t denseIndex) const {
        uint32_t slot = m_denseToSlot[denseIndex];
        return Id{ slot, m_slotGen[slot] };
    }

    uint32_t slot_capacity() const {
        return static_cast<uint32_t>(m_slotGen.size());
    }

    template<class... Args>
    Id create(Args&&... args) {
        uint32_t slot = allocSlot();
        uint32_t denseIndex = static_cast<uint32_t>(m_dense.size());

        m_dense.emplace_back(std::forward<Args>(args)...);

        m_denseToSlot.push_back(slot);
        m_slotToDense[slot] = denseIndex;

        return Id{ slot, m_slotGen[slot] };
    }

    void destroy(Id handle) {
        if (!alive(handle)) return;

        uint32_t denseIndex = m_slotToDense[handle.slot];
        uint32_t lastDenseIndex = static_cast<uint32_t>(m_dense.size()) - 1;

        if (denseIndex != lastDenseIndex) {
            m_dense[denseIndex] = std::move(m_dense[lastDenseIndex]);

            uint32_t movedSlot = m_denseToSlot[lastDenseIndex];
            m_denseToSlot[denseIndex] = movedSlot;
            m_slotToDense[movedSlot] = denseIndex;
        }

        m_dense.pop_back();
        m_denseToSlot.pop_back();

        m_slotToDense[handle.slot] = INVALID;
        ++m_slotGen[handle.slot];
        m_freeSlots.push_back(handle.slot);
    }

    std::vector<T>& dense() {
        return m_dense;
    }

    const std::vector<T>& dense() const {
        return m_dense;
    }

    void clear() {
        m_slotGen.clear();
        m_slotToDense.clear();
        m_denseToSlot.clear();
        m_freeSlots.clear();
        m_dense.clear();
    }

private:
    uint32_t allocSlot() {
        if (!m_freeSlots.empty()) {
            uint32_t slot = m_freeSlots.back();
            m_freeSlots.pop_back();
            return slot;
        }

        uint32_t slot = static_cast<uint32_t>(m_slotGen.size());

        m_slotGen.push_back(0);
        m_slotToDense.push_back(INVALID);

        return slot;
    }

    std::vector<uint32_t> m_slotGen;
    std::vector<uint32_t> m_slotToDense;
    std::vector<uint32_t> m_denseToSlot;
    std::vector<uint32_t> m_freeSlots;
    std::vector<T> m_dense;
};