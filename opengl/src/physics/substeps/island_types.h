#pragma once

#include <vector>
#include <algorithm>
#include <cstdint>

#include "physics_step_types.h"

struct Island {
    uint32_t id = 0;
    std::vector<RigidBodyHandle> bodies;
    int substeps = 1;
};

struct MotionRisk {
    bool risky = false;
    int wantedSubsteps = 1;
    AABB sweptAABB;
};

struct IslandDSU {
    std::vector<int> parent;
    std::vector<int> size;
    std::vector<int> wantedSubsteps;
    std::vector<RigidBodyHandle> bodies;

    std::vector<int> slotToIndex;
    std::vector<uint32_t> slotGen;

    void init(size_t slotCap) {
        parent.clear();
        size.clear();
        wantedSubsteps.clear();
        bodies.clear();

        slotToIndex.assign(slotCap, -1);
        slotGen.assign(slotCap, 0);
    }

    int find(int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }

    int add(RigidBodyHandle h, int wanted) {
        if (h.slot < slotToIndex.size()) {
            int existing = slotToIndex[h.slot];

            if (existing != -1 && slotGen[h.slot] == h.gen) {
                wantedSubsteps[existing] =
                    std::max(wantedSubsteps[existing], wanted);

                return existing;
            }
        }

        int idx = static_cast<int>(parent.size());

        parent.push_back(idx);
        size.push_back(1);
        wantedSubsteps.push_back(wanted);
        bodies.push_back(h);

        slotToIndex[h.slot] = idx;
        slotGen[h.slot] = h.gen;

        return idx;
    }

    void unite(int a, int b) {
        a = find(a);
        b = find(b);

        if (a == b) return;

        if (size[a] < size[b]) {
            std::swap(a, b);
        }

        parent[b] = a;
        size[a] += size[b];

        wantedSubsteps[a] =
            std::max(wantedSubsteps[a], wantedSubsteps[b]);
    }
};