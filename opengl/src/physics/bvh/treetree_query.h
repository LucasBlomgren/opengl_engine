#pragma once
#include <vector>

template<class BVHA, class BVHB>
void treeVsTreeQuery(
    const BVHA& a,
    const BVHB& b,
    std::vector<std::pair<typename BVHA::Element, typename BVHB::Element>>& out)
{
    using EA = typename BVHA::Element;
    using EB = typename BVHB::Element;

    if (a.nodes.empty() || b.nodes.empty() || a.rootIdx == -1 || b.rootIdx == -1) return;

    out.clear();
    out.reserve((std::min)(a.nodes.size(), b.nodes.size()));

    // same-tree duplicate guard only if BVH type + element type match
    constexpr bool sameType = std::is_same_v<EA, EB>&& std::is_same_v<BVHA, BVHB>;
    const bool sameTree = sameType && (static_cast<const void*>(&a) == static_cast<const void*>(&b));
    const bool needOrderCheck = sameType && sameTree;

    struct Entry { int ai; int bi; };
    constexpr int MaxStack = (BVHA::MaxStackSize > BVHB::MaxStackSize) ? BVHA::MaxStackSize : BVHB::MaxStackSize;

    Entry stack[MaxStack];
    int sp = 0;
    stack[sp++] = { a.rootIdx, b.rootIdx };

    auto sah2 = [](const AABB& box) {
        const glm::vec3 e = box.worldMax - box.worldMin;
        return e.x * e.y + e.y * e.z + e.z * e.x;
        };

    while (sp) {
        const auto [ai, bi] = stack[--sp];
        const auto& nA = a.nodes[ai];
        const auto& nB = b.nodes[bi];

        if (nA.isLeaf && nB.isLeaf) {
            if (nA.tightBox.intersects(nB.tightBox)) {
                if constexpr (sameType) {
                    if (!needOrderCheck || ai < bi) out.emplace_back(nA.element, nB.element);
                }
                else {
                    out.emplace_back(nA.element, nB.element);
                }
            }
            continue;
        }
        else {
            if (!nA.fatBox.intersects(nB.fatBox)) continue;
        }

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