#include <algorithm>
#include <cassert>

#include "pch.h"
#include "sweep_and_prune.h"

#include "physics/bodies/rigidbody.h"

namespace physics::internal {

namespace sap {

    //==================================================
    // Active set
    //==================================================

    void SweepAndPrune::ActiveSet::clear()
    {
        items.clear();
        pos.clear();
    }

    void SweepAndPrune::ActiveSet::reset(int itemCount)
    {
        items.clear();
        items.reserve(itemCount);

        pos.assign(itemCount, -1);
    }

    void SweepAndPrune::ActiveSet::add(int idx)
    {
        assert(idx >= 0);
        assert(idx < static_cast<int>(pos.size()));
        assert(pos[idx] == -1 && "SAP active item added twice");

        pos[idx] = static_cast<int>(items.size());
        items.push_back(idx);
    }

    void SweepAndPrune::ActiveSet::remove(int idx)
    {
        assert(idx >= 0);
        assert(idx < static_cast<int>(pos.size()));

        const int itemPos = pos[idx];

        if (itemPos == -1) {
            return;
        }

        const int lastItem = items.back();

        items[itemPos] = lastItem;
        pos[lastItem] = itemPos;

        items.pop_back();
        pos[idx] = -1;
    }

    //==================================================
    // General state
    //==================================================

    void SweepAndPrune::clear()
    {
        mode = Mode::Empty;
        sweepAxis = 0;
        edgesSorted = false;

        aItems.clear();
        bItems.clear();
        edges.clear();

        activeA.clear();
        activeB.clear();
    }

    //==================================================
    // Build
    //==================================================

    void SweepAndPrune::build(
        RuntimeCaches* caches,
        const std::vector<BodyHandle>& handles)
    {
        clear();

        mode = Mode::SameSet;

        buildItems(caches, handles, aItems);
        sweepAxis = chooseLargestExtentAxis(aItems);

        buildSameSetEdges();

        // The edge array has not yet been sorted.
        edgesSorted = false;
    }

    void SweepAndPrune::build(
        RuntimeCaches* caches,
        const std::vector<BodyHandle>& aHandles,
        const std::vector<BodyHandle>& bHandles)
    {
        clear();

        mode = Mode::TwoSets;

        buildItems(caches, aHandles, aItems);
        buildItems(caches, bHandles, bItems);

        sweepAxis = chooseLargestExtentAxis(aItems, bItems);

        buildTwoSetEdges();

        // The edge array has not yet been sorted.
        edgesSorted = false;
    }

    void SweepAndPrune::buildItems(
        RuntimeCaches* caches,
        const std::vector<BodyHandle>& handles,
        std::vector<SapItem>& out)
    {
        out.clear();
        out.reserve(handles.size());

        for (BodyHandle handle : handles) {
            RigidBody* body = caches->bodies.get(handle, FUNC_NAME);

            out.emplace_back(SapItem{
                handle,
                body->aabb
                });
        }
    }

    void SweepAndPrune::buildSameSetEdges()
    {
        edges.clear();
        edges.reserve(aItems.size() * 2);

        for (int i = 0; i < static_cast<int>(aItems.size()); ++i) {
            edges.emplace_back(Edge{
                aItems[i].box.worldMin[sweepAxis],
                i,
                true,
                true
                });

            edges.emplace_back(Edge{
                aItems[i].box.worldMax[sweepAxis],
                i,
                true,
                false
                });
        }
    }

    void SweepAndPrune::buildTwoSetEdges()
    {
        edges.clear();
        edges.reserve((aItems.size() + bItems.size()) * 2);

        for (int i = 0; i < static_cast<int>(aItems.size()); ++i) {
            edges.emplace_back(Edge{
                aItems[i].box.worldMin[sweepAxis],
                i,
                true,
                true
                });

            edges.emplace_back(Edge{
                aItems[i].box.worldMax[sweepAxis],
                i,
                true,
                false
                });
        }

        for (int i = 0; i < static_cast<int>(bItems.size()); ++i) {
            edges.emplace_back(Edge{
                bItems[i].box.worldMin[sweepAxis],
                i,
                false,
                true
                });

            edges.emplace_back(Edge{
                bItems[i].box.worldMax[sweepAxis],
                i,
                false,
                false
                });
        }
    }

    //==================================================
    // Query
    //==================================================

    void SweepAndPrune::query(
        RuntimeCaches* caches,
        std::vector<SpeculativeDynamicPair>& out)
    {
        if (mode == Mode::Empty) {
            return;
        }

        // Only refresh the data that can change between substeps.
        updateItems(caches, aItems);

        if (mode == Mode::TwoSets) {
            updateItems(caches, bItems);
        }

        updateEdgeValues();
        sortEdges();

        if (mode == Mode::SameSet) {
            querySameSet(out);
        }
        else {
            queryTwoSets(out);
        }
    }

    void SweepAndPrune::updateItems(
        RuntimeCaches* caches,
        std::vector<SapItem>& items)
    {
        for (SapItem& item : items) {
            RigidBody* body = caches->bodies.get(item.handle, FUNC_NAME);
            item.box = body->aabb;
        }
    }

    void SweepAndPrune::updateEdgeValues()
    {
        for (Edge& edge : edges) {
            const SapItem& item = edge.isA
                ? aItems[edge.itemIdx]
                : bItems[edge.itemIdx];

            edge.value = edge.isMin
                ? item.box.worldMin[sweepAxis]
                : item.box.worldMax[sweepAxis];
        }
    }

    void SweepAndPrune::sortEdges()
    {
        if (!edgesSorted) {
            // First query after build: edges may be completely unsorted.
            std::sort(edges.begin(), edges.end(), edgeLess);
            edgesSorted = true;
            return;
        }

        // Following substeps:
        // endpoints normally moved only a short distance, so the array
        // should already be almost sorted.
        for (int i = 1; i < static_cast<int>(edges.size()); ++i) {
            Edge edge = edges[i];
            int j = i;

            while (j > 0 && edgeLess(edge, edges[j - 1])) {
                edges[j] = edges[j - 1];
                --j;
            }

            edges[j] = edge;
        }
    }

    void SweepAndPrune::querySameSet(
        std::vector<SpeculativeDynamicPair>& out)
    {
        const int count = static_cast<int>(aItems.size());

        if (count <= 1) {
            return;
        }

        activeA.reset(count);

        for (const Edge& edge : edges) {
            const int itemIdx = edge.itemIdx;

            if (edge.isMin) {
                const AABB& boxA = aItems[itemIdx].box;

                for (int otherIdx : activeA.items) {
                    const AABB& boxB = aItems[otherIdx].box;

                    if (!overlapsOtherTwoAxes(
                        boxA,
                        boxB,
                        sweepAxis))
                    {
                        continue;
                    }

                    out.emplace_back(
                        aItems[itemIdx].handle,
                        aItems[otherIdx].handle
                    );

                    // Also add the reverse pair for for speculative sweep ownership.
                    out.emplace_back(
                        aItems[otherIdx].handle,
                        aItems[itemIdx].handle
                    );
                }

                activeA.add(itemIdx);
            }
            else {
                activeA.remove(itemIdx);
            }
        }
    }

    void SweepAndPrune::queryTwoSets(
        std::vector<SpeculativeDynamicPair>& out)
    {
        if (aItems.empty() || bItems.empty()) {
            return;
        }

        activeA.reset(static_cast<int>(aItems.size()));
        activeB.reset(static_cast<int>(bItems.size()));

        for (const Edge& edge : edges) {
            if (edge.isA) {
                const int aIdx = edge.itemIdx;

                if (edge.isMin) {
                    const AABB& boxA = aItems[aIdx].box;

                    for (int bIdx : activeB.items) {
                        const AABB& boxB = bItems[bIdx].box;

                        if (!overlapsOtherTwoAxes(
                            boxA,
                            boxB,
                            sweepAxis))
                        {
                            continue;
                        }

                        out.emplace_back(
                            aItems[aIdx].handle,
                            bItems[bIdx].handle
                        );
                    }

                    activeA.add(aIdx);
                }
                else {
                    activeA.remove(aIdx);
                }
            }
            else {
                const int bIdx = edge.itemIdx;

                if (edge.isMin) {
                    const AABB& boxB = bItems[bIdx].box;

                    for (int aIdx : activeA.items) {
                        const AABB& boxA = aItems[aIdx].box;

                        if (!overlapsOtherTwoAxes(
                            boxA,
                            boxB,
                            sweepAxis))
                        {
                            continue;
                        }

                        // Set A remains the first body.
                        out.emplace_back(
                            aItems[aIdx].handle,
                            bItems[bIdx].handle
                        );
                    }

                    activeB.add(bIdx);
                }
                else {
                    activeB.remove(bIdx);
                }
            }
        }
    }

    //==================================================
    // Sorting
    //==================================================

    bool SweepAndPrune::edgeLess(
        const Edge& a,
        const Edge& b)
    {
        if (a.value != b.value) {
            return a.value < b.value;
        }

        // Min before max means touching intervals count as overlap.
        if (a.isMin != b.isMin) {
            return a.isMin && !b.isMin;
        }

        // Deterministic order between sets.
        if (a.isA != b.isA) {
            return a.isA && !b.isA;
        }

        return a.itemIdx < b.itemIdx;
    }

    //==================================================
    // Overlap tests
    //==================================================

    bool SweepAndPrune::overlapsOtherTwoAxes(
        const AABB& a,
        const AABB& b,
        int axis)
    {
        switch (axis) {
        case 0:
            // Swept X; test Y and Z.
            return
                a.worldMin.y <= b.worldMax.y &&
                a.worldMax.y >= b.worldMin.y &&
                a.worldMin.z <= b.worldMax.z &&
                a.worldMax.z >= b.worldMin.z;

        case 1:
            // Swept Y; test X and Z.
            return
                a.worldMin.x <= b.worldMax.x &&
                a.worldMax.x >= b.worldMin.x &&
                a.worldMin.z <= b.worldMax.z &&
                a.worldMax.z >= b.worldMin.z;

        default:
            // Swept Z; test X and Y.
            return
                a.worldMin.x <= b.worldMax.x &&
                a.worldMax.x >= b.worldMin.x &&
                a.worldMin.y <= b.worldMax.y &&
                a.worldMax.y >= b.worldMin.y;
        }
    }

    //==================================================
    // Sweep-axis selection
    //==================================================

    int SweepAndPrune::chooseLargestExtentAxis(
        const std::vector<SapItem>& items)
    {
        if (items.empty()) {
            return 0;
        }

        glm::vec3 minP = items.front().box.worldMin;
        glm::vec3 maxP = items.front().box.worldMax;

        for (int i = 1; i < static_cast<int>(items.size()); ++i) {
            minP = glm::min(minP, items[i].box.worldMin);
            maxP = glm::max(maxP, items[i].box.worldMax);
        }

        const glm::vec3 extent = maxP - minP;

        if (extent.x > extent.y) {
            return extent.x > extent.z ? 0 : 2;
        }

        return extent.y > extent.z ? 1 : 2;
    }

    int SweepAndPrune::chooseLargestExtentAxis(
        const std::vector<SapItem>& aItems,
        const std::vector<SapItem>& bItems)
    {
        bool first = true;

        glm::vec3 minP(0.0f);
        glm::vec3 maxP(0.0f);

        auto include = [&](const AABB& box) {
            if (first) {
                minP = box.worldMin;
                maxP = box.worldMax;
                first = false;
                return;
            }

            minP = glm::min(minP, box.worldMin);
            maxP = glm::max(maxP, box.worldMax);
            };

        for (const SapItem& item : aItems) {
            include(item.box);
        }

        for (const SapItem& item : bItems) {
            include(item.box);
        }

        if (first) {
            return 0;
        }

        const glm::vec3 extent = maxP - minP;

        if (extent.x > extent.y) {
            return extent.x > extent.z ? 0 : 2;
        }

        return extent.y > extent.z ? 1 : 2;
    }

} // namespace sap

}
