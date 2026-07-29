#pragma once
#include <vector>

namespace physics::internal {

template<class BVHA, class BVHB>
void treeVsTreeQuery(
    const BVHA& a,
    const BVHB& b,
    std::vector<std::pair<typename BVHA::Element, typename BVHB::Element>>& out
) {
    if (a.nodes.empty() || b.nodes.empty() || a.rootIdx == -1 || b.rootIdx == -1) {
        return;
    }

    out.clear();
    out.reserve((std::min)(a.nodes.size(), b.nodes.size()));

    struct Entry {
        int ai;
        int bi;
    };

    constexpr int MaxStack =
        (BVHA::MaxStackSize > BVHB::MaxStackSize)
        ? BVHA::MaxStackSize
        : BVHB::MaxStackSize;

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

        if (!nA.fatBox.intersects(nB.fatBox)) {
            continue;
        }

        if (nA.isLeaf && nB.isLeaf) {
            a.forEachLeafElement(nA, [&](const auto& elemA, const AABB& boxA) {
                b.forEachLeafElement(nB, [&](const auto& elemB, const AABB& boxB) {
                    if (boxA.intersects(boxB)) {
                        out.emplace_back(elemA, elemB);
                    }
                    });
                });

            continue;
        }

        // Decide which node to expand next based on surface area heuristic (SAH).
        const bool expandA =
            !nA.isLeaf &&
            (nB.isLeaf || sah2(nA.fatBox) >= sah2(nB.fatBox));

        // Expand the node with the larger surface area, or the non-leaf node if one is a leaf.
        if (expandA) {
            // Expand node A
            const int a0 = nA.childAIdx;
            const int a1 = nA.childBIdx;

            if (a0 != -1 && sp < MaxStack) {
                stack[sp++] = { a0, bi };
            }

            if (a1 != -1 && sp < MaxStack) {
                stack[sp++] = { a1, bi };
            }
        }
        else {
            // Expand node B
            const int b0 = nB.childAIdx;
            const int b1 = nB.childBIdx;

            if (b0 != -1 && sp < MaxStack) {
                stack[sp++] = { ai, b0 };
            }

            if (b1 != -1 && sp < MaxStack) {
                stack[sp++] = { ai, b1 };
            }
        }
    }
}

}
