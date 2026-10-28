#pragma once

#include "physics/storage/physics_storage.h"
#include "physics/bodies/rigidbody.h"
#include "physics/collider/aabb.h"

namespace physics::internal {

// #TODO: optimize nodes for cache size:
// Het array — bara det traverseringen rör. Indexeras parallellt med cold[].
//struct BVHNodeHot {
//    float fatMin[3];   // 12
//    float fatMax[3];   // 12
//    int   childA;      // 4   (-1 => leaf)
//    int   childB;      // 4
//    BodyHandle element; // 8  (giltig bara för löv)
//};                      // 40 B → ryms i en cache-line, ~1,5 nod/line
//
//// Kall array — allt övrigt, rörs bara i build/update/refit, aldrig i query.
//struct BVHNodeCold {
//    AABB tightBox;
//    int parentIdx, start, count, selfIdx;
//    bool alive, dirty;
//    // local-koordinater etc.
//};

class BVHTree {
public:
   BVHTree() = default;

   using Element = BodyHandle;

   void init(
      PhysicsStorage* storage,
      size_t allocSize,
      bool writeBodyLeafIndices
   );
   void clear();

   bool dirty = false;
   int rootIdx = -1;
   bool shouldUpdateRenderData = false;

   // public för att rendera AABBs
   struct Node {
      bool alive = true;
      bool isLeaf = false;

      int selfIdx = -1;
      int parentIdx = -1;
      int childAIdx = -1;
      int childBIdx = -1;

      int   start = -1;
      int   count = -1;
      bool  dirty = false;

      Element element;

      // #TODO: probably better with physicsStorage.getCollider().getAABB() instead
      // and remove tightBox
      AABB  tightBox;     // only for leaf nodes

      AABB  fatBox;       // for all nodes
   };
   std::vector<Node> nodes;

   // tree vs tree query
   static constexpr int MaxStackSize = 512;
   static constexpr int MaxCollisionBuf = 25000;

   void buildFromBodies(const std::vector<BodyHandle>& handles);

   // for speculative pairs, using speculative AABBs
   void buildFromExternalAABBs(
      const std::vector<BodyHandle>& handles,
      const std::vector<AABB>& boxes
   );

   void update();
   void singleQuery(const AABB& qBox, std::vector<BodyHandle>& out) const;
   bool queryAny(const AABB& qBox, BodyHandle ignoreBody) const;

   int insertLeaf(BodyHandle handle);

   void removeLeaf(int leafIdx);

   template<class Func>
   void forEachLeafElement(const Node& leaf, Func&& fn) const {
      fn(leaf.element, leaf.tightBox);
   }

private:
   PhysicsStorage* physicsStorage = nullptr;

   bool writeBodyLeafIndices = true;

   int rebuildCooldown = 5;
   int rebuildCooldownCounter = 0;
   int numRefits = 0;
   int rebuildThreshold = 0; // recalculated in build() as log2(n) * rebuildRatio

   // min refits before rebuild, to avoid rebuilding too early when n is small
   const int   minRebuildThreshold = 5;

   const float rebuildRatio = 0.40f; // % of leaf corrections before rebuild
   const glm::vec3 fatBoxMargin{ 0.2f };

   struct BVHPrimitive {
      glm::vec3 min{ 0.0f };
      glm::vec3 max{ 0.0f };
      glm::vec3 centroid{ 0.0f };
      Element element;
   };
   std::vector<BVHPrimitive> prims;

   void initChild(int parentIdx, int nodeIdx, bool isLeft, int start, int end);
   void finalizeLeaf(int leafIdx);
   void splitNode(int parentIdx, int depth);
   void updateLeaves();
   void refitDirtySubtree(int nodeIdx);
   void refitAncestors(int leafIdx);
   void updateRenderData(Node& n);

   void collectPrimitivesFromBodies(const std::vector<BodyHandle>& handles);
   void createPrimitivesFromExternalAABBs(
      const std::vector<BodyHandle>& handles,
      const std::vector<AABB>& boxes
   );

   void constructTree();
   void rebuildFromLeaves();

   int findBestSibling(AABB& box);

   int createLeaf(BodyHandle bodyH);
};


}
