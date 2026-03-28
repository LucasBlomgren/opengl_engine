#pragma once
#include "pch.h"
#include "core/slot_map.h";

template<typename T, typename Handle>
struct PointerCache {
    std::vector<T*> cache;
    SlotMap<T, Handle>* sm = nullptr;
    const char* name = "Unknown";

    void init(SlotMap<T, Handle>& slotMap, const char* name) {
        sm = &slotMap; 
        this->name = name;
        cache.assign(sm->slot_capacity(), nullptr);
    }
    void clear() { 
        cache.assign(sm->slot_capacity(), nullptr); 
    }

    T* get(Handle h, const char* func) {
        if (h.slot >= cache.size()) {
            std::cout << "[" << func << "] Error: Invalid " << name << " handle.\n";
            return nullptr;
        }

        T*& slot = cache[h.slot];
        if (slot) {
            return slot;
        }

        slot = sm->try_get(h);
        if (!slot) {
            std::cout << "[" << func << "] Error: Dead or missing " << name << " for valid-looking handle.\n";
        }

        return slot;
    }
};