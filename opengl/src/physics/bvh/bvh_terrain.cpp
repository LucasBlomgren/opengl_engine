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

    constexpr int MaxDepth = 256;
    const Node* stack[MaxDepth];
    int sp = 0;
    stack[sp++] = &nodes[rootIdx];

    while (sp) {
        const Node* n = stack[--sp];

        if (!n->isLeaf) {
            if (!qBox.intersects(n->fatBox)) continue;

            if (n->childAIdx != -1) stack[sp++] = &nodes[n->childAIdx];
            if (n->childBIdx != -1) stack[sp++] = &nodes[n->childBIdx];
        }
        else {
            if (qBox.intersects(n->tightBox)) out.push_back(n->element);
        }
    }
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

    // Beräkna root->aabb som union av alla primitiva
    root.fatBox.worldMin = prims[0].min;
    root.fatBox.worldMax = prims[0].max;
    for (int i = 1; i < prims.size(); i++) {
        root.fatBox.growToInclude(prims[i].min);
        root.fatBox.growToInclude(prims[i].max);
    }

    // Splittra in i children
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
        Tri* t = &tri;
        AABB box = t->getAABB();

        BVHPrimitive prim;
        prim.min = box.worldMin;
        prim.max = box.worldMax;
        prim.centroid = box.worldCenter;
        prim.element = t;
        prims.push_back(prim);
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

    // choose by largest extent axis to split
    int axis;
    glm::vec3 extent = parent.fatBox.worldMax - parent.fatBox.worldMin;
    axis = (extent.x > extent.y
        ? (extent.x > extent.z ? 0 : 2)
        : (extent.y > extent.z ? 1 : 2));

    // median‐partition of primitives
    int mid = start + count / 2;
    std::nth_element(
        prims.begin() + start, prims.begin() + mid, prims.begin() + start + count,
        [&](auto const& a, auto const& b) {
            return a.centroid[axis] < b.centroid[axis];
        });

    // skapa child-nodes och initiera dem
    Node* A = &nodes.emplace_back();
    int idxA = nodes.size() - 1;

    Node* B = &nodes.emplace_back();
    int idxB = nodes.size() - 1;

    initChild(parentIdx, idxA, true, start, mid, mid - start);
    initChild(parentIdx, idxB, false, mid, start + count, start + count - mid);

    // rekursivt splitta vidare
    split(idxA, depth + 1);
    split(idxB, depth + 1);
}

//------------------------------
//          Init Child
//------------------------------
void TerrainBVH::initChild(int parentIdx, int childIdx, bool isLeft, int start, int end, int count) {

    Node& parent = nodes[parentIdx];
    Node& child = nodes[childIdx];

    child.start = start;
    child.count = count;

    if (isLeft) parent.childAIdx = childIdx;
    else        parent.childBIdx = childIdx;

    // beräkna båda childs fatBox
    child.fatBox.worldMin = prims[start].min;
    child.fatBox.worldMax = prims[start].max;
    for (int i = start + 1; i < end; ++i) {
        child.fatBox.growToInclude(prims[i].min);
        child.fatBox.growToInclude(prims[i].max);
    }
}

//------------------------------
//          Make Leaf
//------------------------------
void TerrainBVH::makeLeaf(int nodeIdx) {
    Node& leaf = nodes[nodeIdx];

    leaf.isLeaf = true;
    leaf.element = prims[leaf.start].element;
    leaf.element->broadphaseHandle.leafIdx = nodeIdx;
    leaf.tightBox = leaf.element->getAABB();
    leaf.fatBox = leaf.tightBox;
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