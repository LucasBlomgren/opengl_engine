#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

template<class Tag>
struct HandleT {
    static constexpr uint32_t INVALID = 0xFFFFFFFFu;

    uint32_t slot = INVALID;
    uint32_t gen = INVALID;

    bool isValid() const noexcept {
        return slot != INVALID;
    }

    explicit operator bool() const noexcept {
        return isValid();
    }

    friend bool operator==(const HandleT& a, const HandleT& b) noexcept {
        return a.slot == b.slot && a.gen == b.gen;
    }
};

namespace std {
    template<class Tag>
    struct hash<HandleT<Tag>> {
        size_t operator()(const HandleT<Tag>& handle) const noexcept {
            uint64_t value = (uint64_t(handle.gen) << 32) | uint64_t(handle.slot);
            return std::hash<uint64_t>{}(value);
        }
    };
}