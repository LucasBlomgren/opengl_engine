#include "pch.h"
#include "sweep_and_prune.h"

#include <algorithm>

#include "tri.h"

namespace sap {

    namespace {

        struct SapEdge {
            float value = 0.0f;
            int itemIdx = -1;
            bool isMin = false;
        };

        bool edgeLess(const SapEdge& a, const SapEdge& b) {
            if (a.value != b.value) {
                return a.value < b.value;
            }

            // Min före max så touching intervals räknas som overlap.
            if (a.isMin != b.isMin) {
                return a.isMin && !b.isMin;
            }

            return a.itemIdx < b.itemIdx;
        }

        void removeActiveIndex(std::vector<int>& active, int idx) {
            for (int i = 0; i < static_cast<int>(active.size()); ++i) {
                if (active[i] == idx) {
                    active[i] = active.back();
                    active.pop_back();
                    return;
                }
            }
        }

        int chooseLargestExtentAxis(const std::vector<SapItem>& items) {
            if (items.empty()) {
                return 0;
            }

            glm::vec3 minP = items[0].box.worldMin;
            glm::vec3 maxP = items[0].box.worldMax;

            for (int i = 1; i < static_cast<int>(items.size()); ++i) {
                minP = glm::min(minP, items[i].box.worldMin);
                maxP = glm::max(maxP, items[i].box.worldMax);
            }

            glm::vec3 extent = maxP - minP;

            if (extent.x > extent.y) {
                return extent.x > extent.z ? 0 : 2;
            }

            return extent.y > extent.z ? 1 : 2;
        }

        int chooseLargestExtentAxis(
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
                }
                else {
                    minP = glm::min(minP, box.worldMin);
                    maxP = glm::max(maxP, box.worldMax);
                }
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

            glm::vec3 extent = maxP - minP;

            if (extent.x > extent.y) {
                return extent.x > extent.z ? 0 : 2;
            }

            return extent.y > extent.z ? 1 : 2;
        }

    } // anonymous namespace

    void buildItems(
        RuntimeCaches* caches,
        const std::vector<RigidBodyHandle>& handles,
        std::vector<SapItem>& out)
    {
        out.clear();
        out.reserve(handles.size());

        for (RigidBodyHandle h : handles) {
            RigidBody* body = caches->bodies.get(h, FUNC_NAME);

            SapItem item;
            item.handle = h;
            item.box = body->aabb;

            out.push_back(item);
        }
    }

    void querySameSet(
        const std::vector<SapItem>& items,
        std::vector<std::pair<RigidBodyHandle, RigidBodyHandle>>& out)
    {
        const int count = static_cast<int>(items.size());

        if (count <= 1) {
            return;
        }

        const int axis = chooseLargestExtentAxis(items);

        static std::vector<SapEdge> edges;
        static std::vector<int> active;

        edges.clear();
        edges.reserve(count * 2);

        for (int i = 0; i < count; ++i) {
            edges.push_back(SapEdge{
                items[i].box.worldMin[axis],
                i,
                true
                });

            edges.push_back(SapEdge{
                items[i].box.worldMax[axis],
                i,
                false
                });
        }

        std::sort(edges.begin(), edges.end(), edgeLess);

        active.clear();
        active.reserve(count);

        for (const SapEdge& edge : edges) {
            const int aIdx = edge.itemIdx;

            if (edge.isMin) {
                const AABB& boxA = items[aIdx].box;

                for (int bIdx : active) {
                    const AABB& boxB = items[bIdx].box;

                    // Sweep axis gav kandidat. Detta gör full 3D AABB-test.
                    if (boxA.intersects(boxB)) {
                        out.emplace_back(items[aIdx].handle, items[bIdx].handle);
                    }
                }

                active.push_back(aIdx);
            }
            else {
                removeActiveIndex(active, aIdx);
            }
        }
    }

    void queryTwoSets(
        const std::vector<SapItem>& aItems,
        const std::vector<SapItem>& bItems,
        std::vector<std::pair<RigidBodyHandle, RigidBodyHandle>>& out)
    {
        if (aItems.empty() || bItems.empty()) {
            return;
        }

        struct SapEdge2 {
            float value = 0.0f;
            int itemIdx = -1;
            bool isA = false;
            bool isMin = false;
        };

        auto edge2Less = [](const SapEdge2& a, const SapEdge2& b) {
            if (a.value != b.value) {
                return a.value < b.value;
            }

            if (a.isMin != b.isMin) {
                return a.isMin && !b.isMin;
            }

            if (a.isA != b.isA) {
                return a.isA && !b.isA;
            }

            return a.itemIdx < b.itemIdx;
            };

        const int axis = chooseLargestExtentAxis(aItems, bItems);

        static std::vector<SapEdge2> edges;
        static std::vector<int> activeA;
        static std::vector<int> activeB;

        edges.clear();
        edges.reserve((aItems.size() + bItems.size()) * 2);

        for (int i = 0; i < static_cast<int>(aItems.size()); ++i) {
            edges.push_back(SapEdge2{ aItems[i].box.worldMin[axis], i, true, true });
            edges.push_back(SapEdge2{ aItems[i].box.worldMax[axis], i, true, false });
        }

        for (int i = 0; i < static_cast<int>(bItems.size()); ++i) {
            edges.push_back(SapEdge2{ bItems[i].box.worldMin[axis], i, false, true });
            edges.push_back(SapEdge2{ bItems[i].box.worldMax[axis], i, false, false });
        }

        std::sort(edges.begin(), edges.end(), edge2Less);

        activeA.clear();
        activeB.clear();

        activeA.reserve(aItems.size());
        activeB.reserve(bItems.size());

        for (const SapEdge2& edge : edges) {
            if (edge.isA) {
                const int aIdx = edge.itemIdx;

                if (edge.isMin) {
                    const AABB& boxA = aItems[aIdx].box;

                    for (int bIdx : activeB) {
                        const AABB& boxB = bItems[bIdx].box;

                        if (boxA.intersects(boxB)) {
                            // A är alltid awake/dynamic.
                            out.emplace_back(aItems[aIdx].handle, bItems[bIdx].handle);
                        }
                    }

                    activeA.push_back(aIdx);
                }
                else {
                    removeActiveIndex(activeA, aIdx);
                }
            }
            else {
                const int bIdx = edge.itemIdx;

                if (edge.isMin) {
                    const AABB& boxB = bItems[bIdx].box;

                    for (int aIdx : activeA) {
                        const AABB& boxA = aItems[aIdx].box;

                        if (boxA.intersects(boxB)) {
                            // A är alltid awake/dynamic.
                            out.emplace_back(aItems[aIdx].handle, bItems[bIdx].handle);
                        }
                    }

                    activeB.push_back(bIdx);
                }
                else {
                    removeActiveIndex(activeB, bIdx);
                }
            }
        }
    }

    void queryTerrain(
        const TerrainBVH& terrainBvh,
        const std::vector<SapItem>& dynamicItems,
        std::vector<TerrainPair>& out)
    {
        out.clear();

        if (terrainBvh.rootIdx == -1 || dynamicItems.empty()) {
            return;
        }

        static std::vector<Tri*> triHits;

        out.reserve(dynamicItems.size());

        for (const SapItem& item : dynamicItems) {
            triHits.clear();

            terrainBvh.singleQuery(item.box, triHits);

            if (!triHits.empty()) {
                out.emplace_back(TerrainPair{
                    item.handle,
                    triHits
                    });
            }
        }
    }

    void computePairs(
        RuntimeCaches* caches,
        const TerrainBVH& terrainBvh,
        const std::vector<RigidBodyHandle>& awakeHandles,
        const std::vector<RigidBodyHandle>& asleepHandles,
        const std::vector<RigidBodyHandle>& staticHandles,
        std::vector<TerrainPair>& terrainPairs,
        std::vector<DynamicPair>& dynamicPairs)
    {
        static std::vector<SapItem> awakeItems;
        static std::vector<SapItem> asleepItems;
        static std::vector<SapItem> staticItems;
        static std::vector<std::pair<RigidBodyHandle, RigidBodyHandle>> pairBuffer;

        buildItems(caches, awakeHandles, awakeItems);
        buildItems(caches, asleepHandles, asleepItems);
        buildItems(caches, staticHandles, staticItems);

        terrainPairs.clear();
        dynamicPairs.clear();
        pairBuffer.clear();

        // Dynamic vs terrain.
        queryTerrain(terrainBvh, awakeItems, terrainPairs);

        // Dynamic vs dynamic.
        querySameSet(awakeItems, pairBuffer);

        // Dynamic vs asleep.
        queryTwoSets(awakeItems, asleepItems, pairBuffer);

        // Dynamic vs static.
        queryTwoSets(awakeItems, staticItems, pairBuffer);

        dynamicPairs.reserve(pairBuffer.size());

        for (const auto& p : pairBuffer) {
            dynamicPairs.emplace_back(DynamicPair{
                p.first,
                p.second
                });
        }
    }

} // namespace sap