#include "pch.h"
#include "bvh_terrain.h"
#include "tri.h"

//------------------------------
//        Single Query
//------------------------------
void TerrainBVH::singleQuery(const AABB& qBox, std::vector<Tri*>& out) const {
    if (nodes.empty()) return;

    out.clear();
    out.reserve(MaxCollisionBuf);

    constexpr int MaxStackSize = TerrainBVH::MaxStackSize;
    const Node* stack[MaxStackSize];
    int sp = 0;

    stack[sp++] = &nodes[rootIdx];

    while (sp) {
        const Node* n = stack[--sp];

        if (!qBox.intersects(n->fatBox)) {
            continue;
        }

        if (!n->isLeaf) {
            if (n->childAIdx != -1 && sp < MaxStackSize) stack[sp++] = &nodes[n->childAIdx];
            if (n->childBIdx != -1 && sp < MaxStackSize) stack[sp++] = &nodes[n->childBIdx];
            continue;
        }

        // Leaf contains prims[start ... start + count)
        const int begin = n->start;
        const int end = n->start + n->count;

        for (int i = begin; i < end; ++i) {
            const BVHPrimitive& prim = prims[i];

            if (qBox.intersects(prim.box)) {
                out.push_back(prim.element);
            }
        }
    }
}

//------------------------------
//        Query Any
//------------------------------
bool TerrainBVH::queryAny(const AABB& qBox) const {
    if (nodes.empty()) return false;

    constexpr int MaxDepth = TerrainBVH::MaxStackSize;
    const Node* stack[MaxDepth];
    int sp = 0;
    stack[sp++] = &nodes[rootIdx];

    while (sp) {
        const Node* n = stack[--sp];

        if (!qBox.intersects(n->fatBox)) {
            continue;
        }

        if (!n->isLeaf) {
            if (n->childAIdx != -1 && sp < MaxStackSize)
                stack[sp++] = &nodes[n->childAIdx];

            if (n->childBIdx != -1 && sp < MaxStackSize)
                stack[sp++] = &nodes[n->childBIdx];

            continue;
        }

        // Leaf contains prims[start ... start + count)
        const int begin = n->start;
        const int end = n->start + n->count;

        for (int i = begin; i < end; ++i) {
            const BVHPrimitive& prim = prims[i];

            if (qBox.intersects(prim.box)) {
                return true;
            }
        }
    }

    return false;
}

//-------------------------
//         Build 
//-------------------------
void TerrainBVH::build(std::vector<Tri>& tris) {
    nodes.clear();

    // Fyll primitives
    createPrimitives(tris);

    if (prims.empty()) return;

    // Förallokera nod-poolen
    nodes.reserve(prims.size() * 2);

    // Skapa root-nod
    rootIdx = 0;
    nodes.emplace_back();
    Node& root = nodes[rootIdx];
    root.start = 0;
    root.count = prims.size();

    // Calculate root AABB as union of all primitives
    root.fatBox.worldMin = prims[0].box.worldMin;
    root.fatBox.worldMax = prims[0].box.worldMax;
    for (int i = 1; i < prims.size(); i++) {
        root.fatBox.growToInclude(prims[i].box.worldMin);
        root.fatBox.growToInclude(prims[i].box.worldMax);
    }

    // split into children
    int depth = 0;
    split(rootIdx, depth);

    for (auto& n : nodes) {
        if (n.isLeaf) {
            n.fatBox.grow(fatBoxMargin);

            updateRenderData(n);
        }
        else {
            n.dirty = true;
        }
    }
    if (rootIdx != -1) refitNode(rootIdx);
}

//------------------------------
//      Create Primitives
//------------------------------
void TerrainBVH::createPrimitives(std::vector<Tri>& tris) {
    prims.clear();
    prims.reserve(tris.size());

    for (Tri& tri : tris) {
        prims.emplace_back(tri.getAABB(), &tri);
    }
}

//------------------------------
//            Split
//------------------------------
void TerrainBVH::split(int parentIdx, int depth) {
    Node& parent = nodes[parentIdx];
    int start = parent.start;
    int count = parent.count;

    // create leaf node
    if (count <= leafThreshold) {
        makeLeaf(parentIdx);
        return;
    }

    int mid = start + (count / 2);
    int end = start + count;

    // choose by largest extent axis to split
    int axis;
    glm::vec3 extent = parent.fatBox.worldMax - parent.fatBox.worldMin;
    axis = (extent.x > extent.y
        ? (extent.x > extent.z ? 0 : 2)
        : (extent.y > extent.z ? 1 : 2));

    // median‐partition of primitives
    std::nth_element(
        prims.begin() + start, prims.begin() + mid, prims.begin() + start + count,
        [&](auto const& a, auto const& b) {
            return a.box.worldCenter[axis] < b.box.worldCenter[axis];
        });

    // create child nodes and init them
    Node* A = &nodes.emplace_back();
    A->selfIdx = nodes.size() - 1;
    Node* B = &nodes.emplace_back();
    B->selfIdx = nodes.size() - 1;

    initChild(parentIdx, A->selfIdx, true, start, mid);
    initChild(parentIdx, B->selfIdx, false, mid, end);

    // recursive split
    split(A->selfIdx, depth + 1);
    split(B->selfIdx, depth + 1);
}

//------------------------------
//          Init Child
//------------------------------
void TerrainBVH::initChild(int parentIdx, int childIdx, bool isLeft, int start, int end) {

    Node& parent = nodes[parentIdx];
    Node& child = nodes[childIdx];

    child.parentIdx = parentIdx;
    child.start = start;
    child.count = end - start;

    if (isLeft) parent.childAIdx = childIdx;
    else        parent.childBIdx = childIdx;

    // calculate both child's fatBox
    child.fatBox.worldMin = prims[start].box.worldMin;
    child.fatBox.worldMax = prims[start].box.worldMax;

    for (int i = start + 1; i < end; ++i) {
        child.fatBox.growToInclude(prims[i].box.worldMin);
        child.fatBox.growToInclude(prims[i].box.worldMax);
    }
}

//------------------------------
//          Make Leaf
//------------------------------
void TerrainBVH::makeLeaf(int nodeIdx) {
    Node& leaf = nodes[nodeIdx];

    leaf.selfIdx = nodeIdx;
    leaf.isLeaf = true;
}

//------------------------------
//          Refit Node
//------------------------------
void TerrainBVH::refitNode(int nodeIdx) {
    Node& node = nodes[nodeIdx];

    if (!node.dirty)
        return;

    if (node.isLeaf) {
        // blad: fatBox är redan expanderad vid containment‐kontrollen
        return;
    }

    Node* childA = &nodes[node.childAIdx];
    Node* childB = &nodes[node.childBIdx];

    // först barnen
    refitNode(node.childAIdx);
    refitNode(node.childBIdx);

    // när barnen är klara, unionera dem
    if (childA and childB) {
        node.fatBox.worldMin = glm::min(childA->fatBox.worldMin, childB->fatBox.worldMin);
        node.fatBox.worldMax = glm::max(childA->fatBox.worldMax, childB->fatBox.worldMax);
    }

    updateRenderData(node);

    node.dirty = false;
}

//------------------------------
//     Update Render Data
//------------------------------
void TerrainBVH::updateRenderData(Node& n) {
    n.fatBox.worldCenter = (n.fatBox.worldMin + n.fatBox.worldMax) * 0.5f;
    n.fatBox.worldHalfExtents = (n.fatBox.worldMax - n.fatBox.worldMin) * 0.5f;
}