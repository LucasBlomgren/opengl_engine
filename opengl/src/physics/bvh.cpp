#include "pch.h"
#include "bvh.h"
#include "game_object.h"
#include "tri.h"

template class BVHTree<GameObject>;
template class BVHTree<Tri>;

//------------------------------
//        Single Query
//------------------------------
template<typename E>
void BVHTree<E>::singleQuery(const AABB& qBox, std::vector<E*>& out) {

    if (nodes.size() == 0) {
        return;
    }

    // 1) Pre-allocate the output vector once per call
    constexpr int MaxExpected = BVHTree<E>::MaxCollisionBuf;
    out.clear();
    out.reserve(MaxExpected);

    // 2) Fast, stack-allocated traversal stack
    constexpr int MaxDepth = 64;
    Node* stack[MaxDepth];
    int   sp = 0;
    stack[sp++] = &nodes[rootIdx];

    // 3) Traverse without any heap-allocations
    while (sp) {
        Node* n = stack[--sp];

        if (!n->isLeaf) {
            if (!qBox.intersects(n->fatBox))
                continue;


            if (n->childAIdx != -1) {
                Node* node = &nodes[n->childAIdx];
                stack[sp++] = node;
            }
            if (n->childBIdx != -1) { 
                Node* node = &nodes[n->childBIdx];
                stack[sp++] = node;
            }
        }
        else {
            if (qBox.intersects(n->tightBox)) {
                // bara en enkel branch + push_back (ingen omallokering pga reserve)
                out.push_back(n->element);
            }
        }
    }
}

//------------------------------
//          Update 
//------------------------------
template<typename E>
void BVHTree<E>::update(std::vector<E>& elements) {
    if (numRefits > rebuildThreshold or numIterationsSinceRebuild >= updateInterval) {
        // för många refits, bygg om trädet
        build(elements);
        numRefits = 0; 
        numRebuilds++; 
        numIterationsSinceRebuild = 0; 
        return;
    }
    numIterationsSinceRebuild++;

    for (auto& n : nodes) n.dirty = false;

    updateLeaves();
    if (rootIdx != -1) refitNode(rootIdx);
}

//------------------------------
//       Update Leaves
//------------------------------
template<typename E>
void BVHTree<E>::updateLeaves() {
    for (int i = 0; i < (int)nodes.size(); ++i) 
    {
        Node& n = nodes[i];

        if (!n.isLeaf) 
            continue;

        n.tightBox = n.element->getAABB();
        if (n.fatBox.contains(n.tightBox)) 
            continue;

        n.fatBox.wMin = n.tightBox.wMin;
        n.fatBox.wMax = n.tightBox.wMax;
        n.fatBox.grow(fatBoxMargin);
        this->numRefits++;

        // aabbrenderer
        n.fatBox.centroid = (n.fatBox.wMin + n.fatBox.wMax) * 0.5f;
        n.fatBox.halfExtents = (n.fatBox.wMax - n.fatBox.wMin) * 0.5f;

        // Markera just det lövet **och alla dess föräldrar**
        for (int p = i; p != -1; p = nodes[p].parentIdx)
            nodes[p].dirty = true;
    }
}

//------------------------------
//          Refit Node
//------------------------------
template<typename E>
void BVHTree<E>::refitNode(int nodeIdx) {
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
        node.fatBox.wMin = glm::min(childA->fatBox.wMin, childB->fatBox.wMin);
        node.fatBox.wMax = glm::max(childA->fatBox.wMax, childB->fatBox.wMax);
    }

    // aabbrenderer
    node.fatBox.centroid = (node.fatBox.wMin + node.fatBox.wMax) * 0.5f;
    node.fatBox.halfExtents = (node.fatBox.wMax - node.fatBox.wMin) * 0.5f;

    node.dirty = false;
}

//----------------------------------
//     Build & helper functions
//----------------------------------
template<typename E>
void BVHTree<E>::build(std::vector<E>& elements) {
    rootIdx = 0;
    nodes.clear();

    // Fyll primitives
    createPrimitives(elements);

    if (prims.empty())
        return;

    rebuildThreshold = std::max(minRebuildThreshold, int(prims.size() * rebuildRatio + 0.5f));

    // Förallokera nod-poolen
    nodes.reserve(prims.size() * 2);

    // Skapa root-nod
    nodes.emplace_back();

    Node& root = nodes[rootIdx];
    root.start = 0;
    root.count = prims.size();

    // Beräkna root->aabb som union av alla primitiva
    root.fatBox.wMin = prims[0].min;
    root.fatBox.wMax = prims[0].max;
    for (int i = 1; i < prims.size(); i++) {
        root.fatBox.growToInclude(prims[i].min);
        root.fatBox.growToInclude(prims[i].max);
    }

    // setup fatBoxRenderer
    root.fatBox.centroid = (root.fatBox.wMin + root.fatBox.wMax) * 0.5f;
    root.fatBox.halfExtents = (root.fatBox.wMax - root.fatBox.wMin) * 0.5f;

    // Splittra in i children
    int depth = 0;
    split(rootIdx, depth);
}

//------------------------------
//      Create Primitives
//------------------------------
template<typename E>
void BVHTree<E>::createPrimitives(std::vector<E>& elements) {
    prims.clear();
    prims.reserve(elements.size());

    for (auto& elem : elements) {
        BVHPrimitive prim;
        AABB Ebox = elem.getAABB();
        prim.min = Ebox.wMin;
        prim.max = Ebox.wMax;
        prim.centroid = Ebox.centroid;
        prim.element = &elem;
        prims.push_back(prim);
    }
}

//------------------------------
//            Split
//------------------------------
template<typename E>
void BVHTree<E>::split(int parentIdx, int depth) {
    Node& parent = nodes[parentIdx];
    int start = parent.start;
    int count = parent.count;

    // create leaf node
    if (count <= leafThreshold) {
        makeLeaf(parentIdx);
        return;
    }

    // välj enligt största extent + median
    int axis;
    glm::vec3 extent = parent.fatBox.wMax - parent.fatBox.wMin;
    axis = (extent.x > extent.y
        ? (extent.x > extent.z ? 0 : 2)
        : (extent.y > extent.z ? 1 : 2));

    // median‐partition av primitives
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
template<typename E>
void BVHTree<E>::initChild(int parentIdx, int childIdx, bool isLeft, int start, int end, int count) {

    Node& parent = nodes[parentIdx];
    Node& child = nodes[childIdx];

    child.parentIdx = parentIdx;
    child.start = start;
    child.count = count;

    if (isLeft) parent.childAIdx = childIdx;
    else        parent.childBIdx = childIdx;

    // beräkna båda childs fatBox
    child.fatBox.wMin = prims[start].min;
    child.fatBox.wMax = prims[start].max;
    for (int i = start + 1; i < end; ++i) {
        child.fatBox.growToInclude(prims[i].min);
        child.fatBox.growToInclude(prims[i].max);
    }

    // setup aabbRenderer wireframe
    child.fatBox.centroid = (child.fatBox.wMin + child.fatBox.wMax) * 0.5f;
    child.fatBox.halfExtents = (child.fatBox.wMax - child.fatBox.wMin) * 0.5f;
}

//------------------------------
//          Make Leaf
//------------------------------
template<typename E>
void BVHTree<E>::makeLeaf(int nodeIdx) {
    Node& leaf = nodes[nodeIdx];

    leaf.isLeaf = true;
    leaf.element = prims[leaf.start].element;
    leaf.tightBox = leaf.element->getAABB(); 
    leaf.fatBox = leaf.tightBox;
    leaf.fatBox.grow(fatBoxMargin);

    // setup aabbRenderer wireframe
    leaf.fatBox.centroid = (leaf.fatBox.wMin + leaf.fatBox.wMax) * 0.5f;
    leaf.fatBox.halfExtents = (leaf.fatBox.wMax - leaf.fatBox.wMin) * 0.5f;
}