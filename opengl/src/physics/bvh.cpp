#include "bvh.h"


int BVHTree::treeVsTreeQuery(BVHTree& bvhA, BVHTree& bvhB) {
    int sp = 0;  // index för nodeStack
    int outSp = 0;  // index för collisionBuf

    nodeStack[sp++] = { bvhA.root, bvhB.root };

    while (sp > 0) {
        // pop
        NodePair cur = nodeStack[--sp];
        Node* A = cur.A;
        Node* B = cur.B;

        // leaf–leaf: tight‐test
        if (A->isLeaf && B->isLeaf) {
            if (A->tightBox.intersects(B->tightBox) &&
                A->object->id < B->object->id &&
                outSp < (int)MaxCollisionBuf)
            {
                collisionBuf[outSp++] = { A->object, B->object };
            }
            continue;
        }

        // prune på fat‐box
        if (!A->fatBox.intersects(B->fatBox))
            continue;

        // expandera utan STL: kontrollera alltid sp < MaxStackSize
        if (A->isLeaf) {
            if (B->childA && sp < (int)MaxStackSize)
                nodeStack[sp++] = { A, B->childA };
            if (B->childB && sp < (int)MaxStackSize)
                nodeStack[sp++] = { A, B->childB };
        }
        else if (B->isLeaf) {
            if (A->childA && sp < (int)MaxStackSize)
                nodeStack[sp++] = { A->childA, B };
            if (A->childB && sp < (int)MaxStackSize)
                nodeStack[sp++] = { A->childB, B };
        }
        else {
            if (A->childA && B->childA && sp < (int)MaxStackSize)
                nodeStack[sp++] = { A->childA, B->childA };
            if (A->childA && B->childB && sp < (int)MaxStackSize)
                nodeStack[sp++] = { A->childA, B->childB };
            if (A->childB && B->childA && sp < (int)MaxStackSize)
                nodeStack[sp++] = { A->childB, B->childA };
            if (A->childB && B->childB && sp < (int)MaxStackSize)
                nodeStack[sp++] = { A->childB, B->childB };
        }
    }

    return outSp;
}

int BVHTree::singleQuery(AABB& qBox) {
    int hc = 0;
    constexpr int MaxDepth = 64;
    Node* stack[MaxDepth];
    int   sp = 0;
    stack[sp++] = root;

    // iterate through the tree
    while (sp) {
        Node* n = stack[--sp];

        if (!n->isLeaf) {
            if (!qBox.intersects(n->fatBox))
                continue;

            if (n->childA) stack[sp++] = n->childA;
            if (n->childB) stack[sp++] = n->childB;
        }   

        // leaf node
        else {
            if (hc >= collisions.size()) {
                collisions.resize(collisions.size() * 4);
            }
            if (qBox.intersects(n->tightBox))
                this->collisions[hc++] = n->object;
        }
    }

    return hc;
}

void BVHTree::update(std::vector<GameObject>& objects) {
    if (numRefits > rebuildThreshold or numIterationsSinceRebuild >= updateInterval) {
        // för många refits, bygg om trädet
        build(objects);
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

void BVHTree::updateLeaves() {
    for (auto& n : nodes) {
        if (!n.isLeaf) 
            continue;

        n.tightBox = n.object->AABB;
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

void BVHTree::refitNode(Node* node) {
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

void BVHTree::createPrimitives(std::vector<GameObject>& objects) {
    prims.clear();
    prims.reserve(objects.size());

    for (GameObject& obj : objects) {
        //if (obj.isStatic) {
        //    continue; // skip static objects
        //}
        BVHPrimitive prim;
        prim.min = obj.AABB.wMin;
        prim.max = obj.AABB.wMax;
        prim.centroid = obj.AABB.centroid;
        prim.object = &obj;
        prims.push_back(prim);
    }
}

void BVHTree::split(Node& parent) {
    int start = parent.start;
    int count = parent.count;

    // create leaf node
    if (count <= leafThreshold) {
        parent.isLeaf = true;
        parent.object = prims[start].object;
        parent.tightBox = parent.object->AABB;
        parent.fatBox = parent.tightBox;
        parent.fatBox.grow(fatBoxMargin);

        // setup aabbRenderer wireframe
        parent.fatBox.centroid = (parent.fatBox.wMin + parent.fatBox.wMax) * 0.5f;
        parent.fatBox.halfExtents = (parent.fatBox.wMax - parent.fatBox.wMin) * 0.5f;

        return;
    }

    // välj split-axel via extent
    glm::vec3 extent = parent.fatBox.wMax - parent.fatBox.wMin;
    int axis = extent.x > extent.y ? (extent.x > extent.z ? 0 : 2) : (extent.y > extent.z ? 1 : 2);

    // median‐partition av primitives
    int mid = start + count / 2;
    std::nth_element(
        prims.begin() + start, prims.begin() + mid, prims.begin() + start + count,
        [&](auto const& a, auto const& b) {
            return a.centroid[axis] < b.centroid[axis];
        });

    // skapa child-nodes
    Node* A = &nodes.emplace_back();
    Node* B = &nodes.emplace_back();

    A->parent = &parent;
    B->parent = &parent;
    parent.childA = A;
    parent.childB = B;

    // initiera child-intervall
    A->start = start;
    A->count = mid - start;
    B->start = mid;
    B->count = start + count - mid;

    // beräkna båda childs fatBox
    A->fatBox.wMin = prims[start].min;
    A->fatBox.wMax = prims[start].max;
    for (int i = start + 1; i < mid; ++i) {
        A->fatBox.growToInclude(prims[i].min);
        A->fatBox.growToInclude(prims[i].max);
    }
    A->fatBox.grow(fatBoxMargin);

    B->fatBox.wMin = prims[mid].min;
    B->fatBox.wMax = prims[mid].max;
    for (int i = mid + 1; i < start + count; ++i) {
        B->fatBox.growToInclude(prims[i].min);
        B->fatBox.growToInclude(prims[i].max);
    }
    B->fatBox.grow(fatBoxMargin);

    // setup aabbRenderer wireframe
    A->fatBox.centroid = (A->fatBox.wMin + A->fatBox.wMax) * 0.5f;
    A->fatBox.halfExtents = (A->fatBox.wMax - A->fatBox.wMin) * 0.5f;
    B->fatBox.centroid = (B->fatBox.wMin + B->fatBox.wMax) * 0.5f;
    B->fatBox.halfExtents = (B->fatBox.wMax - B->fatBox.wMin) * 0.5f;

    // rekursivt splitta vidare
    split(*A);
    split(*B);
}

void BVHTree::build(std::vector<GameObject>& objects) {
    root = nullptr;
    nodes.clear();
    collisions.resize(objects.size() * 2);

    // Fyll primitives
    createPrimitives(objects);

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
    split(*root);
}