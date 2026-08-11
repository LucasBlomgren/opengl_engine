#pragma once

#include <vector>

#include "physics/world/runtime_caches.h"
#include "physics/colliders/aabb.h"
#include "physics/broadphase/contact_types.h"

namespace physics::internal {

struct RuntimeCaches;

namespace sap {

    struct SapItem {
        BodyHandle handle;
        AABB box;
    };

    class SweepAndPrune {
    public:

        enum class Mode {
            Empty,
            SameSet,
            TwoSets
        };

        void clear();

        // Build one persistent SAP set.
        void build(
            RuntimeCaches* caches,
            const std::vector<BodyHandle>& handles
        );

        // Build two persistent SAP sets.
        void build(
            RuntimeCaches* caches,
            const std::vector<BodyHandle>& aHandles,
            const std::vector<BodyHandle>& bHandles
        );

        // Refresh AABBs, update edge positions, sort and emit pairs.
        // Does not clear out.
        void query(
            RuntimeCaches* caches,
            std::vector<SpeculativeDynamicPair>& out
        );

        Mode getMode() const {
            return mode;
        }

    private:

        struct Edge {
            float value = 0.0f;
            int itemIdx = -1;

            // Always true for SameSet mode.
            bool isA = true;
            bool isMin = false;
        };

        struct ActiveSet {
            std::vector<int> items;
            std::vector<int> pos;

            void clear();
            void reset(int itemCount);
            void add(int idx);
            void remove(int idx);
        };

        Mode mode = Mode::Empty;
        int sweepAxis = 0;

        // False immediately after build().
        // First query uses std::sort; later queries use insertion sort.
        bool edgesSorted = false;

        std::vector<SapItem> aItems;
        std::vector<SapItem> bItems;
        std::vector<Edge> edges;

        ActiveSet activeA;
        ActiveSet activeB;

        static bool edgeLess(const Edge& a, const Edge& b);

        static bool overlapsOtherTwoAxes(
            const AABB& a,
            const AABB& b,
            int sweepAxis
        );

        static int chooseLargestExtentAxis(
            const std::vector<SapItem>& items
        );

        static int chooseLargestExtentAxis(
            const std::vector<SapItem>& aItems,
            const std::vector<SapItem>& bItems
        );

        static void buildItems(
            RuntimeCaches* caches,
            const std::vector<BodyHandle>& handles,
            std::vector<SapItem>& out
        );

        static void updateItems(
            RuntimeCaches* caches,
            std::vector<SapItem>& items
        );

        void buildSameSetEdges();
        void buildTwoSetEdges();

        void updateEdgeValues();
        void sortEdges();

        void querySameSet(std::vector<SpeculativeDynamicPair>& out);
        void queryTwoSets(std::vector<SpeculativeDynamicPair>& out);
    };

} // namespace sap

}
