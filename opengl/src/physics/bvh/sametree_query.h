#pragma once
#include <vector>
#include <cassert>

template<class BVH>
void treeVsSameTreeQuery(
    const BVH& tree,
    std::vector<std::pair<typename BVH::Element, typename BVH::Element>>& out
) {
    using Element = typename BVH::Element;

    out.clear();

    if (tree.nodes.empty() || tree.rootIdx == -1)
        return;

    out.reserve(std::min(tree.nodes.size(), static_cast<size_t>(BVH::MaxCollisionBuf)));

    struct Entry {
        int a;
        int b;
    };

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

        // Specialfall: samma node mot sig själv.
        // Här vill vi INTE göra vanlig traversal, för då får vi både L-R och R-L.
        if (ai == bi) {
            if (nA.isLeaf) {
                // leaf mot sig själv = self-pair, ska alltid bort
                continue;
            }

            const int left = nA.childAIdx;
            const int right = nA.childBIdx;

            // Par inom vänstra subträdet
            push(left, left);

            // Par mellan vänster och höger, bara EN riktning
            if (left != -1 && right != -1 &&
                tree.nodes[left].fatBox.intersects(tree.nodes[right].fatBox))
            {
                push(left, right);
            }

            // Par inom högra subträdet
            push(right, right);

            continue;
        }

        // Olika noder: vanlig pruning med fat boxes
        if (!nA.fatBox.intersects(nB.fatBox)) {
            continue;
        }

        // Leaf-leaf: riktig kandidat
        if (nA.isLeaf && nB.isLeaf) {
            if (nA.tightBox.intersects(nB.tightBox)) {
                out.emplace_back(nA.element, nB.element);
            }
            continue;
        }

        // Expandera den större noden, ungefär som din vanliga tree-vs-tree
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