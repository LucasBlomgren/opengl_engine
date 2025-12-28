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
    constexpr int MaxDepth = 256;
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
//        Insert Leaf
//------------------------------
template<typename E>
void BVHTree<E>::insertLeaf(E* e) 
{
    if (e->bvhLeafIdx != -1) {
        // already in tree
        return;
    }

    // create leaf node
    Node* leaf = createLeaf(e);

    // if tree is empty, set rootIdx. Leaf becomes root.
    if (rootIdx == -1) { 
        rootIdx = leaf->selfIdx; 
        return;
    } 

    // find best sibling
    Node* sibling = findBestSibling(leaf->fatBox);

    // create new parent node
    Node* parent = &nodes.emplace_back(); 

    parent->selfIdx = nodes.size() - 1; 
    parent->childAIdx = sibling->selfIdx;
    parent->childBIdx = leaf->selfIdx;

    int oldParentIdx = sibling->parentIdx;
    sibling->parentIdx = parent->selfIdx;
    leaf->parentIdx = parent->selfIdx; 

    // sibling was root, parent becomes new root
    if (oldParentIdx == -1) {
        rootIdx = parent->selfIdx;
        parent->parentIdx = -1;
    }
    // else connect new parent to old parent
    else {
        Node* oldParent = &nodes[oldParentIdx];
        bool siblingIsLeft = (sibling->selfIdx == oldParent->childAIdx);
        if (siblingIsLeft) {
            oldParent->childAIdx = parent->selfIdx;
        } else {
            oldParent->childBIdx = parent->selfIdx;
        }
        parent->parentIdx = oldParent->selfIdx;
    }

    // refit all parents
    refitParents(leaf->parentIdx);

}

//------------------------------
//      Refit Parents
//------------------------------
template<typename E>
void BVHTree<E>::refitParents(int parentIdx) {
    for (int p = parentIdx; p != -1; p = nodes[p].parentIdx) {
        Node& n = nodes[p];

        Node& nA = nodes[n.childAIdx];
        Node& nB = nodes[n.childBIdx];

        const AABB* boxA = &nodes[n.childAIdx].fatBox;
        const AABB* boxB = &nodes[n.childBIdx].fatBox;

        n.fatBox.wMin = glm::min(boxA->wMin, boxB->wMin);
        n.fatBox.wMax = glm::max(boxA->wMax, boxB->wMax);

        updateRenderData(n);
    }
}

//------------------------------
//      Find Best Sibling
//------------------------------
template<typename E>
typename BVHTree<E>::Node* BVHTree<E>::findBestSibling(AABB& box) 
{
    Node* n = &nodes[rootIdx];

    while (!n->isLeaf) {
        Node* A = &nodes[n->childAIdx];
        Node* B = &nodes[n->childBIdx];

        // ökning i area om vi lägger box under respektive barn
        float mergedSurfaceA = box.getMergedSurfaceArea(A->fatBox, box);
        float mergedSurfaceB = box.getMergedSurfaceArea(B->fatBox, box);

        A->fatBox.setSurfaceArea(); 
        B->fatBox.setSurfaceArea(); 

        float incA = mergedSurfaceA - A->fatBox.surfaceArea;
        float incB = mergedSurfaceB - B->fatBox.surfaceArea;

        n = (incA < incB) ? A : B;
    }

    return n; // löv
}

//------------------------------
//        Create Leaf
//------------------------------
template<typename E>
typename BVHTree<E>::Node* BVHTree<E>::createLeaf(E* e) 
{
    Node* leaf = &nodes.emplace_back();
    leaf->selfIdx = nodes.size() - 1;
    leaf->isLeaf = true;
    leaf->element = e;
    leaf->element->bvhLeafIdx = leaf->selfIdx;
    leaf->tightBox = e->getAABB();
    leaf->fatBox = leaf->tightBox;
    leaf->fatBox.grow(fatBoxMargin);

    updateRenderData(*leaf);

    return leaf;
}

//------------------------------
//          Remove Leaf
//------------------------------
template<typename E>
void BVHTree<E>::removeLeaf(int leafIdx) 
{
    if (leafIdx == -1) {
        return;
    }

    Node& leaf = nodes[leafIdx];

    if (leafIdx == rootIdx) {
        // root is leaf, clear tree
        leaf.alive = false;
        leaf.parentIdx = -1;
        leaf.element->bvhLeafIdx = -1;
        leaf.element = nullptr;
        rootIdx = -1;
        nodes.clear();
        prims.clear();
        return;
    }

    Node& parent = nodes[leaf.parentIdx];
    Node& sibling = (parent.childAIdx == leafIdx) ? nodes[parent.childBIdx] : nodes[parent.childAIdx];

    if (parent.selfIdx == rootIdx) {
        // parent is root, replace root with sibling
        rootIdx = sibling.selfIdx;
        sibling.parentIdx = -1;
    } 
    else {
        // replace parent with sibling
        sibling.parentIdx = parent.parentIdx;
        Node& grandParent = nodes[parent.parentIdx];

        if (grandParent.childAIdx == parent.selfIdx) {
            grandParent.childAIdx = sibling.selfIdx;
        } else {
            grandParent.childBIdx = sibling.selfIdx;
        }

        // refit all parents
        refitParents(grandParent.selfIdx);
    }

    // remove parent and leaf
    leaf.alive = false;
    leaf.parentIdx = -1;
    leaf.element->bvhLeafIdx = -1;
    leaf.element = nullptr; 

    parent.alive = false;
    parent.parentIdx = -1;
    parent.childAIdx = -1;
    parent.childBIdx = -1;
}

//------------------------------
//          Update 
//------------------------------
template<typename E>
void BVHTree<E>::update(std::vector<E>& elements, std::vector<int>& indexes, bool useAllElements) {
    if (numRefits > rebuildThreshold /* or numIterationsSinceRebuild >= updateInterval*/) {
        // för många refits, bygg om trädet
        build(elements, indexes, useAllElements);
        numRefits = 0; 
        numIterationsSinceRebuild = 0; 
        numRebuilds++;
        return;
    }
    numIterationsSinceRebuild++;

    for (auto& n : nodes) n.dirty = false;

    updateLeaves();
    if (rootIdx != -1) refitNode(rootIdx);

    this->dirty = false;
}

//------------------------------
//       Update Leaves
//------------------------------
template<typename E>
void BVHTree<E>::updateLeaves() {
    for (int i = 0; i < (int)nodes.size(); ++i) {
        Node& n = nodes[i];

        if (!n.isLeaf or !n.alive) {
            continue;
        }

        n.tightBox = n.element->getAABB();
        if (n.fatBox.contains(n.tightBox)) {
            continue;
        }

        n.fatBox.wMin = n.tightBox.wMin;
        n.fatBox.wMax = n.tightBox.wMax;
        n.fatBox.grow(fatBoxMargin);
        numRefits++;

        updateRenderData(n);

        // Markera just det lövet **och alla dess föräldrar**
        for (int p = i; p != -1; p = nodes[p].parentIdx) {
            nodes[p].dirty = true;
        }
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

    updateRenderData(node);

    node.dirty = false;
}

//-------------------------
//         Build 
//-------------------------
template<typename E>
void BVHTree<E>::build(std::vector<E>& elements, std::vector<int>& indexes, bool useAllElements) {

    nodes.clear();
    // Fyll primitives
    createPrimitives(elements, indexes, useAllElements);

    if (prims.empty()) return;

    numRefits = 0;
    numIterationsSinceRebuild = 0;
    rebuildThreshold = std::max(minRebuildThreshold, int(prims.size() * rebuildRatio + 0.5f));

    // Förallokera nod-poolen
    nodes.reserve(prims.size() * 2);

    // Skapa root-nod
    rootIdx = 0;
    nodes.emplace_back();
    Node& root = nodes[rootIdx];
    root.selfIdx = rootIdx;
    root.start = 0;
    root.count = prims.size();

    // Beräkna root->aabb som union av alla primitiva
    root.fatBox.wMin = prims[0].min;
    root.fatBox.wMax = prims[0].max;
    for (int i = 1; i < prims.size(); i++) {
        root.fatBox.growToInclude(prims[i].min);
        root.fatBox.growToInclude(prims[i].max);
    }

    updateRenderData(root);

    // Splittra in i children
    int depth = 0;
    split(rootIdx, depth);

    for (auto& n : nodes) {
        if (n.isLeaf) {
            n.fatBox.grow(fatBoxMargin);

            updateRenderData(n);
        } else {
            n.dirty = true;
        }
    }
    if (rootIdx != -1) refitNode(rootIdx);
}

//------------------------------
//      Create Primitives
//------------------------------
template<typename E>
void BVHTree<E>::createPrimitives(std::vector<E>& elements, std::vector<int>& idx, bool useAllElements) {
    prims.clear();

    if (useAllElements) {
        prims.reserve(elements.size());
        for (int i = 0; i < elements.size(); i++) {
            E& elem = elements[i];
            elem.bvhLeafIdx = -1; // reset
            BVHPrimitive prim;
            AABB Ebox = elem.getAABB();
            prim.min = Ebox.wMin;
            prim.max = Ebox.wMax;
            prim.centroid = Ebox.centroid;
            prim.element = &elem;
            prims.push_back(prim);
        }
    } else {
        prims.reserve(idx.size());
        for (int i = 0; i < idx.size(); i++) {
            E& elem = elements[idx[i]];
            elem.bvhLeafIdx = -1;
            BVHPrimitive prim;
            AABB Ebox = elem.getAABB();
            prim.min = Ebox.wMin;
            prim.max = Ebox.wMax;
            prim.centroid = Ebox.centroid;
            prim.element = &elem;
            prims.push_back(prim);
        }
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

    // choose by largest extent axis to split
    int axis;
    glm::vec3 extent = parent.fatBox.wMax - parent.fatBox.wMin;
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
    A->selfIdx = nodes.size() - 1;

    Node* B = &nodes.emplace_back();
    B->selfIdx = nodes.size() - 1;

    initChild(parentIdx, A->selfIdx, true, start, mid, mid - start);
    initChild(parentIdx, B->selfIdx, false, mid, start + count, start + count - mid);

    // rekursivt splitta vidare
    split(A->selfIdx, depth + 1);
    split(B->selfIdx, depth + 1);
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
}

//------------------------------
//          Make Leaf
//------------------------------
template<typename E>
void BVHTree<E>::makeLeaf(int nodeIdx) {
    Node& leaf = nodes[nodeIdx];

    leaf.selfIdx = nodeIdx;
    leaf.isLeaf = true;
    leaf.element = prims[leaf.start].element;
    leaf.element->bvhLeafIdx = leaf.selfIdx;
    leaf.tightBox = leaf.element->getAABB(); 
    leaf.fatBox = leaf.tightBox;
}

template<typename E>
void BVHTree<E>::updateRenderData(Node& n) {
    n.fatBox.centroid = (n.fatBox.wMin + n.fatBox.wMax) * 0.5f;
    n.fatBox.halfExtents = (n.fatBox.wMax - n.fatBox.wMin) * 0.5f;
}