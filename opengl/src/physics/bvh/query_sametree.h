#pragma once
#include <vector>
#include <cassert>
#include <algorithm>
#include <cstdint>
#include <utility>
#include <glm/ext/vector_float3.hpp>

#include "core/slot_map.h"
#include "physics/colliders/aabb.h"

inline uint32_t elementKey(RigidBodyHandle h) {
    return h.slot;
}

template<class BVH>
void treeVsSameTreeQuery(
    const BVH& tree,
    std::vector<std::pair<typename BVH::Element, typename BVH::Element>>& out
) {
    out.clear();

    if (tree.nodes.empty() || tree.rootIdx == -1)
        return;

    out.reserve(std::min(tree.nodes.size(), static_cast<size_t>(BVH::MaxCollisionBuf)));

    struct Entry { int a, b; };

    constexpr int MaxStack = BVH::MaxStackSize;
    Entry stack[MaxStack];
    int sp = 0;

    auto push = [&](int a, int b) {
        if (a == -1 || b == -1)
            return;

        assert(sp < MaxStack && "treeVsSameTreeQuery stack overflow");
        if (sp < MaxStack) {
            stack[sp++] = { a, b };
        }
        };

    auto sah2 = [](const AABB& box) {
        const glm::vec3 e = box.worldMax - box.worldMin;
        return e.x * e.y + e.y * e.z + e.z * e.x;
        };

    push(tree.rootIdx, tree.rootIdx);

    while (sp > 0) {
        Entry entry = stack[--sp];

        const int ai = entry.a;
        const int bi = entry.b;

        const auto& nA = tree.nodes[ai];
        const auto& nB = tree.nodes[bi];

        if (!nA.fatBox.intersects(nB.fatBox)) {
            continue;
        }

        // Leaf-leaf: fungerar även när ai == bi
        if (nA.isLeaf && nB.isLeaf) {
            const bool sameLeaf = (ai == bi);

            tree.forEachLeafElement(nA, [&](const auto& elemA, const AABB& boxA) {
                tree.forEachLeafElement(nB, [&](const auto& elemB, const AABB& boxB) {

                    const uint32_t keyA = elementKey(elemA);
                    const uint32_t keyB = elementKey(elemB);

                    if (sameLeaf) {
                        // Samma leaf mot sig själv:
                        // undvik A-A och dubletter B-A
                        if (keyA >= keyB) {
                            return;
                        }
                    }
                    else {
                        // Olika leaves:
                        // traversal har redan valt en riktning, så skippa bara exakt self-pair
                        if (keyA == keyB) {
                            return;
                        }
                    }

                    if (boxA.intersects(boxB)) {
                        out.emplace_back(elemA, elemB);
                    }
                    });
                });

            continue;
        }

        // Samma interna node mot sig själv
        if (ai == bi) {
            const int left = nA.childAIdx;
            const int right = nA.childBIdx;

            push(left, left);

            if (left != -1 && right != -1 &&
                tree.nodes[left].fatBox.intersects(tree.nodes[right].fatBox))
            {
                push(left, right);
            }

            push(right, right);

            continue;
        }

        // Vanlig expansion för olika noder
        const bool expandA =
            !nA.isLeaf &&
            (nB.isLeaf || sah2(nA.fatBox) >= sah2(nB.fatBox));

        if (expandA) {
            const int a0 = nA.childAIdx;
            const int a1 = nA.childBIdx;

            if (a0 != -1 && tree.nodes[a0].fatBox.intersects(nB.fatBox)) {
                push(a0, bi);
            }

            if (a1 != -1 && tree.nodes[a1].fatBox.intersects(nB.fatBox)) {
                push(a1, bi);
            }
        }
        else {
            const int b0 = nB.childAIdx;
            const int b1 = nB.childBIdx;

            if (b0 != -1 && tree.nodes[b0].fatBox.intersects(nA.fatBox)) {
                push(ai, b0);
            }

            if (b1 != -1 && tree.nodes[b1].fatBox.intersects(nA.fatBox)) {
                push(ai, b1);
            }
        }
    }
}