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
    if (numRefits > rebuildThreshold or numIterationsSinceRebuild >= 120) {
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
        BVHPrimitive prim;
        prim.min = obj.AABB.wMin;
        prim.max = obj.AABB.wMax;
        prim.centroid = obj.AABB.centroid;
        prim.object = &obj;
        prims.push_back(prim);
    }
}

// Hjälpfunktion: välj axel som minimerar volym‐overlap
int BVHTree::chooseAxisByMinOverlap(int start, int count, const AABB& /*parentBox*/) {
    // 1) Se till att vår centroid‐buffert är stor nog
    static std::vector<float> cents;
    cents.resize(count);

    float bestOverlap = std::numeric_limits<float>::infinity();
    int   bestAxis   = 0;

    // 2) Loop över axlar
    for (int axis = 0; axis < 3; ++axis) {
        // a) extrahera centroid‐värden
        for (int i = 0; i < count; ++i) {
            cents[i] = prims[start + i].centroid[axis];
        }

        // b) hitta pivot = median i cents
        int midIdx = count / 2;
        std::nth_element(cents.begin(),
                         cents.begin() + midIdx,
                         cents.end());
        float pivot = cents[midIdx];

        // c) beräkna AABB and overlap‐volym *utan* ytterligare vektor‐kopiering
        AABB L, R;
        bool initedL = false, initedR = false;
        int  countL = 0, countR = 0;

        for (int i = 0; i < count; ++i) {
            auto& p = prims[start + i];
            if (p.centroid[axis] < pivot) {
                if (!initedL) {
                    L.wMin = p.min; L.wMax = p.max;
                    initedL = true;
                } else {
                    L.growToInclude(p.min);
                    L.growToInclude(p.max);
                }
                ++countL;
            } else {
                if (!initedR) {
                    R.wMin = p.min; R.wMax = p.max;
                    initedR = true;
                } else {
                    R.growToInclude(p.min);
                    R.growToInclude(p.max);
                }
                ++countR;
            }
        }

        // d) intersection‐boxen
        glm::vec3 iMin = glm::max(L.wMin, R.wMin);
        glm::vec3 iMax = glm::min(L.wMax, R.wMax);
        glm::vec3 d    = iMax - iMin;
        float overlapVol = (d.x > 0 && d.y > 0 && d.z > 0)
            ? d.x * d.y * d.z
            : 0.0f;

        // e) uppdatera bästa axel
        if (overlapVol < bestOverlap) {
            bestOverlap = overlapVol;
            bestAxis   = axis;
        }
    }

    return bestAxis;
}

void BVHTree::makeLeaf(Node& parent) {
    parent.isLeaf = true;
    parent.object = prims[parent.start].object;
    parent.tightBox = parent.object->AABB;
    parent.fatBox = parent.tightBox;
    parent.fatBox.grow(fatBoxMargin);

    // setup aabbRenderer wireframe
    parent.fatBox.centroid = (parent.fatBox.wMin + parent.fatBox.wMax) * 0.5f;
    parent.fatBox.halfExtents = (parent.fatBox.wMax - parent.fatBox.wMin) * 0.5f;
}

void BVHTree::initChild(Node& parent, Node* n, bool isLeft, int start, int end, int count) {
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

void BVHTree::split(Node& parent, int depth) {
    int start = parent.start;
    int count = parent.count;

    // create leaf node
    if (count <= leafThreshold) {
        makeLeaf(parent);
        return;
    }

    int axis;
    if (depth < minOverlapDepth) {
        // minimera överlapp mellan barn
        axis = chooseAxisByMinOverlap(start, count, parent.fatBox);
    }
    else {
        // enkelt: välj enligt största extent + median
        glm::vec3 extent = parent.fatBox.wMax - parent.fatBox.wMin;
        axis = (extent.x > extent.y
            ? (extent.x > extent.z ? 0 : 2)
            : (extent.y > extent.z ? 1 : 2));
    }
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
    int depth = 0;
    split(*root, depth);
}