#pragma once
#include <vector>

template<class BVHA, class BVHB>
void treeVsTreeQuery(
    const BVHA& a,
    const BVHB& b,
    std::vector<std::pair<typename BVHA::Element, typename BVHB::Element>>& out
) {
    if (a.nodes.empty() || b.nodes.empty() || a.rootIdx == -1 || b.rootIdx == -1) return;

    out.clear();
    out.reserve((std::min)(a.nodes.size(), b.nodes.size()));

    // same-tree duplicate guard only if BVH type + element type match
    constexpr bool sameType = 
        std::is_same_v<typename BVHA::Element, typename BVHB::Element> && 
        std::is_same_v<BVHA, BVHB>;

    // cast to void* to compare addresses, since C++ doesn't allow comparing unrelated types
    const bool sameTree = 
        sameType && 
        (static_cast<const void*>(&a) == static_cast<const void*>(&b));

    const bool needOrderCheck = sameType && sameTree;

    // stack entry for traversal, holds pair of node indices (one from each tree)
    struct Entry { int ai; int bi; };
    constexpr int MaxStack = (BVHA::MaxStackSize > BVHB::MaxStackSize) ? BVHA::MaxStackSize : BVHB::MaxStackSize;

    // create stack and push root pair
    Entry stack[MaxStack];
    int sp = 0;
    stack[sp++] = { a.rootIdx, b.rootIdx };

    // SAH heuristic for deciding which node to expand, based on surface area of fat boxes. Expand the one with larger surface area to hopefully cull more pairs earlier.
    // Real area is 2 * (xy + yz + zx), but because we only need relative comparisons use (xy + yz + zx).
    auto sah2 = [](const AABB& box) {
        const glm::vec3 e = box.worldMax - box.worldMin; // extents
        return e.x * e.y + e.y * e.z + e.z * e.x;
        };

    // iterative traversal of both trees
    while (sp) {
        const auto [ai, bi] = stack[--sp]; // LIFO stack pop = depth-first traversal
        const auto& nA = a.nodes[ai];
        const auto& nB = b.nodes[bi];

        // if both are leaves, check tight boxes for intersection and add pair if they intersect
        if (nA.isLeaf && nB.isLeaf) {

            // In a same-tree query, avoid self-pairs (A,A) and duplicate reverse pairs (B,A).
            // Do this before the AABB test to avoid testing a leaf against itself.
            if (needOrderCheck && ai >= bi) {
                continue;
            }

            if (nA.tightBox.intersects(nB.tightBox)) {
                out.emplace_back(nA.element, nB.element);
            }

            continue;
        }
        else {
            if (!nA.fatBox.intersects(nB.fatBox)) continue;
        }

        // decide which node to expand based on SAH heuristic
        const bool expandA = !nA.isLeaf && (nB.isLeaf || sah2(nA.fatBox) >= sah2(nB.fatBox));
        if (expandA) {
            const int a0 = nA.childAIdx, a1 = nA.childBIdx;
            if (a0 != -1 && a.nodes[a0].fatBox.intersects(nB.fatBox)) if (sp < MaxStack) stack[sp++] = { a0, bi };
            if (a1 != -1 && a.nodes[a1].fatBox.intersects(nB.fatBox)) if (sp < MaxStack) stack[sp++] = { a1, bi };
        }
        else {
            const int b0 = nB.childAIdx, b1 = nB.childBIdx;
            if (b0 != -1 && b.nodes[b0].fatBox.intersects(nA.fatBox)) if (sp < MaxStack) stack[sp++] = { ai, b0 };
            if (b1 != -1 && b.nodes[b1].fatBox.intersects(nA.fatBox)) if (sp < MaxStack) stack[sp++] = { ai, b1 };
        }
    }
}