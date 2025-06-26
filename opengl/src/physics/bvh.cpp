#include "bvh.h"
#include "game_object.h"

#include <glm/gtx/string_cast.hpp>

template class BVHTree<GameObject>;
template class BVHTree<Tri>;

template<typename E>
void BVHTree<E>::singleQuery(const AABB& qBox, std::vector<E*>& out) {
    // 1) Pre-allocate the output vector once per call
    constexpr int MaxExpected = BVHTree<E>::MaxCollisionBuf;
    out.clear();
    out.reserve(MaxExpected);

    // 2) Fast, stack-allocated traversal stack
    constexpr int MaxDepth = 64;
    Node* stack[MaxDepth];
    int   sp = 0;
    stack[sp++] = root;

    // 3) Traverse without any heap-allocations
    while (sp) {
        Node* n = stack[--sp];

        if (!n->isLeaf) {
            if (!qBox.intersects(n->fatBox))
                continue;
            if (n->childA) stack[sp++] = n->childA;
            if (n->childB) stack[sp++] = n->childB;
        }
        else {
            if (qBox.intersects(n->tightBox)) {
                // bara en enkel branch + push_back (ingen omallokering pga reserve)
                out.push_back(n->element);
            }
        }
    }
}

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
    if (root) refitNode(root);
}

template<typename E>
void BVHTree<E>::updateLeaves() {
    for (auto& n : nodes) {
        if (!n.isLeaf) 
            continue;

        n.tightBox = n.element->getAABB();
        if (n.fatBox.contains(n.tightBox)) 
            continue;

        n.fatBox.wMin = n.tightBox.wMin;
        n.fatBox.wMax = n.tightBox.wMax;
        n.fatBox.grow(fatBoxMargin);
        this->numRefits++;

        n.fatBox.centroid = (n.fatBox.wMin + n.fatBox.wMax) * 0.5f;
        n.fatBox.halfExtents = (n.fatBox.wMax - n.fatBox.wMin) * 0.5f;

        // Markera just det lövet **och alla dess föräldrar**
        for (Node* p = &n; p; p = p->parent)
            p->dirty = true;
    }
}

template<typename E>
void BVHTree<E>::refitNode(Node* node) {
    if (!node or !node->dirty)
        return;

    if (node->isLeaf) {
        // blad: fatBox är redan expanderad vid containment‐kontrollen
        return;
    }

    // först barnen
    refitNode(node->childA);
    refitNode(node->childB);

    // när barnen är klara, unionera dem
    if (node->childA && node->childB) {
        node->fatBox.wMin = glm::min(node->childA->fatBox.wMin, node->childB->fatBox.wMin);
        node->fatBox.wMax = glm::max(node->childA->fatBox.wMax, node->childB->fatBox.wMax);
    }

    node->fatBox.centroid = (node->fatBox.wMin + node->fatBox.wMax) * 0.5f;
    node->fatBox.halfExtents = (node->fatBox.wMax - node->fatBox.wMin) * 0.5f;

    node->dirty = false;
}

template<typename E>
void BVHTree<E>::makeLeaf(Node& parent) {
    parent.isLeaf = true;
    parent.element = prims[parent.start].element;
    parent.tightBox = parent.element->getAABB();
    parent.fatBox = parent.tightBox;
    parent.fatBox.grow(fatBoxMargin);

    // setup aabbRenderer wireframe
    parent.fatBox.centroid = (parent.fatBox.wMin + parent.fatBox.wMax) * 0.5f;
    parent.fatBox.halfExtents = (parent.fatBox.wMax - parent.fatBox.wMin) * 0.5f;
}

template<typename E>
void BVHTree<E>::initChild(Node& parent, Node* n, bool isLeft, int start, int end, int count) {
    n->parent = &parent;
    if (isLeft) parent.childA = n;
    else        parent.childB = n;

    // initiera child-intervall
    n->start = start;
    n->count = count;

    // beräkna båda childs fatBox
    n->fatBox.wMin = prims[start].min;
    n->fatBox.wMax = prims[start].max;
    for (int i = start + 1; i < end; ++i) {
        n->fatBox.growToInclude(prims[i].min);
        n->fatBox.growToInclude(prims[i].max);
    }
    n->fatBox.grow(fatBoxMargin);

    // setup aabbRenderer wireframe
    n->fatBox.centroid = (n->fatBox.wMin + n->fatBox.wMax) * 0.5f;
    n->fatBox.halfExtents = (n->fatBox.wMax - n->fatBox.wMin) * 0.5f;
}

template<typename E>
void BVHTree<E>::split(Node& parent, int depth) {
    int start = parent.start;
    int count = parent.count;

    // create leaf node
    if (count <= leafThreshold) {
        makeLeaf(parent);
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
    Node* B = &nodes.emplace_back();
    initChild(parent, A, true, start, mid, mid - start);
    initChild(parent, B, false, mid, start + count, start + count - mid);

    // rekursivt splitta vidare
    split(*A, depth + 1);
    split(*B, depth + 1);
}

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

template<typename E>
void BVHTree<E>::build(std::vector<E>& elements) {
    root = nullptr;
    nodes.clear();

    // Fyll primitives
    createPrimitives(elements);

    if (prims.empty())
        return;

    rebuildThreshold = std::max(minRebuildThreshold, int(prims.size() * rebuildRatio + 0.5f));

    // Förallokera nod-poolen
    nodes.reserve(prims.size() * 2);

    // Skapa root-nod
    root = &nodes.emplace_back();
    root->start = 0;
    root->count = prims.size();

    // Beräkna root->aabb som union av alla primitiva
    root->fatBox.wMin = prims[0].min;
    root->fatBox.wMax = prims[0].max;
    for (int i = 1; i < prims.size(); i++) {
        root->fatBox.growToInclude(prims[i].min);
        root->fatBox.growToInclude(prims[i].max);
    }
    root->fatBox.grow(fatBoxMargin);

    // setup fatBoxRenderer
    root->fatBox.centroid = (root->fatBox.wMin + root->fatBox.wMax) * 0.5f;
    root->fatBox.halfExtents = (root->fatBox.wMax - root->fatBox.wMin) * 0.5f;

    // Splittra in i children
    int depth = 0;
    split(*root, depth);
}