#include "pch.h"
#include "bvh.h"
#include "game_object.h"
#include "tri.h"
#include "world.h"

#define FUNC_NAME __FUNCTION__

void BVHTree::init(
    PhysicsWorld* world,
    RuntimeCaches* caches,
    int allocSize) 
{
    this->world = world;
    this->caches = caches;

    nodes.clear();
    nodes.reserve(allocSize * 2);

    prims.clear();  
    prims.reserve(allocSize);

    rootIdx = -1;
}   

//------------------------------
//        Single Query
//------------------------------
void BVHTree::singleQuery(const AABB& qBox, std::vector<RigidBodyHandle>& out) const {
    if (nodes.empty()) return;

    out.clear();
    out.reserve(BVHTree::MaxCollisionBuf);

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

//------------------------------------------------------------------
//      Insert Leaf
//------------------------------------------------------------------
int BVHTree::insertLeaf(RigidBodyHandle handle) {
    RigidBody* body = caches->bodies.get(handle, FUNC_NAME);

    if (body->broadphaseHandle.leafIdx != -1) {
        // already in tree
        return -1;
    }

    // create leaf node
    int leafIdx = createLeaf(handle, body);

    // if tree is empty, set rootIdx. Leaf becomes root.
    if (rootIdx == -1) {
        rootIdx = leafIdx;
        return leafIdx;
    }

    // find best sibling
    int siblingIdx = findBestSibling(nodes[leafIdx].fatBox);

    // create new parent node
    Node& parent = nodes.emplace_back();

    Node& leaf = nodes[leafIdx];
    Node& sibling = nodes[siblingIdx];

    parent.selfIdx = nodes.size() - 1;
    parent.childAIdx = sibling.selfIdx;
    parent.childBIdx = leaf.selfIdx;

    int oldParentIdx = sibling.parentIdx;
    sibling.parentIdx = parent.selfIdx;
    leaf.parentIdx = parent.selfIdx;

    // sibling was root, parent becomes new root
    if (oldParentIdx == -1) {
        rootIdx = parent.selfIdx;
        parent.parentIdx = -1;
    }
    // else connect new parent to old parent
    else {
        Node* oldParent = &nodes[oldParentIdx];
        bool siblingIsLeft = (sibling.selfIdx == oldParent->childAIdx);
        if (siblingIsLeft) {
            oldParent->childAIdx = parent.selfIdx;
        }
        else {
            oldParent->childBIdx = parent.selfIdx;
        }
        parent.parentIdx = oldParent->selfIdx;
    }

    // refit all parents
    refitParents(leaf.parentIdx);

    return leaf.selfIdx;
}

int BVHTree::createLeaf(RigidBodyHandle handle, RigidBody* body) {
    AABB aabb = world->computeBodyAABB(*body);

    Node& leaf = nodes.emplace_back();
    leaf.selfIdx = nodes.size() - 1;
    leaf.isLeaf = true;
    leaf.element = handle;
    leaf.tightBox = aabb;
    leaf.fatBox = leaf.tightBox;
    leaf.fatBox.grow(fatBoxMargin);

    body->broadphaseHandle.leafIdx = leaf.selfIdx;

    updateRenderData(leaf);

    return leaf.selfIdx;
}

int BVHTree::findBestSibling(AABB& box)
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

    return n->selfIdx;
}

void BVHTree::refitParents(int parentIdx) {
    for (int p = parentIdx; p != -1; p = nodes[p].parentIdx) {
        Node& n = nodes[p];

        Node& nA = nodes[n.childAIdx];
        Node& nB = nodes[n.childBIdx];

        const AABB* boxA = &nodes[n.childAIdx].fatBox;
        const AABB* boxB = &nodes[n.childBIdx].fatBox;

        n.fatBox.worldMin = glm::min(boxA->worldMin, boxB->worldMin);
        n.fatBox.worldMax = glm::max(boxA->worldMax, boxB->worldMax);

        updateRenderData(n);
    }
}

//------------------------------------------------------------------
//      Remove leaf
//------------------------------------------------------------------
void BVHTree::removeLeaf(int leafIdx)
{
    if (leafIdx == -1) {
        return;
    }

    Node& leaf = nodes[leafIdx]; 

    // root is leaf, clear tree
    if (leafIdx == rootIdx) {
        RigidBody* body = caches->bodies.get(leaf.element, FUNC_NAME);
        body->broadphaseHandle.leafIdx = -1;

        rootIdx = -1;
        nodes.clear(); // #TODO: Fixa så att noder återanvänds. Det är bara här/rebuild som noder tas bort.
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
        }
        else {
            grandParent.childBIdx = sibling.selfIdx;
        }

        // refit all parents
        refitParents(grandParent.selfIdx);
    }

    // remove parent and leaf
    leaf.alive = false;
    leaf.parentIdx = -1;

    RigidBody* body = caches->bodies.get(leaf.element, FUNC_NAME);

    body->broadphaseHandle.leafIdx = -1;
    leaf.element = RigidBodyHandle{};

    parent.alive = false;
    parent.parentIdx = -1;
    parent.childAIdx = -1;
    parent.childBIdx = -1;
}

//------------------------------------------------------------------
//      Update
//------------------------------------------------------------------
void BVHTree::update(std::vector<RigidBodyHandle>& handles) {
    if (numRefits > rebuildThreshold) {
        // för många refits, bygg om trädet
        build(handles);
        numRefits = 0;
        return;
    }

    for (auto& n : nodes) n.dirty = false;

    updateLeaves();
    if (rootIdx != -1) refitNode(rootIdx);

    this->dirty = false;
}

void BVHTree::updateLeaves() {
    for (int i = 0; i < (int)nodes.size(); ++i) {
        Node& n = nodes[i];

        if (!n.isLeaf or !n.alive) {
            continue;
        }

        RigidBody* body = caches->bodies.get(n.element, FUNC_NAME);
        n.tightBox = body->aabb;

        if (n.fatBox.contains(n.tightBox)) {
            continue;
        }

        n.fatBox.worldMin = n.tightBox.worldMin;
        n.fatBox.worldMax = n.tightBox.worldMax;
        n.fatBox.grow(fatBoxMargin);
        numRefits++;

        updateRenderData(n);

        // Markera just det lövet **och alla dess föräldrar**
        for (int p = i; p != -1; p = nodes[p].parentIdx) {
            nodes[p].dirty = true;
        }
    }
}

void BVHTree::refitNode(int nodeIdx) {
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

//------------------------------------------------------------------
//      Build
//------------------------------------------------------------------
void BVHTree::build(std::vector<RigidBodyHandle>& handles) {
    nodes.clear();
    // Fill prims vector with primitives from handles
    createPrimitives(handles);

    if (prims.empty()) return;

    numRefits = 0;
    rebuildThreshold = std::max(minRebuildThreshold, int(prims.size() * rebuildRatio + 0.5f));

    // Pre-allocate nodes, max number of nodes is 2n-1 (full binary tree)
    nodes.reserve(prims.size() * 2);

    // Create root node
    rootIdx = 0;
    nodes.emplace_back();
    Node& root = nodes[rootIdx];
    root.selfIdx = rootIdx;
    root.start = 0;
    root.count = prims.size();

    // Calculate root AABB as union of all primitives
    root.fatBox.worldMin = prims[0].min;
    root.fatBox.worldMax = prims[0].max;
    for (int i = 1; i < prims.size(); i++) {
        root.fatBox.growToInclude(prims[i].min);
        root.fatBox.growToInclude(prims[i].max);
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

void BVHTree::createPrimitives(std::vector<RigidBodyHandle>& handles) {
    prims.clear();
    prims.reserve(handles.size());

    for (RigidBodyHandle& bodyH : handles) {
        RigidBody* body = caches->bodies.get(bodyH, FUNC_NAME);
        body->broadphaseHandle.leafIdx = -1;
        
        AABB box = body->aabb;

        BVHPrimitive prim;
        prim.min = box.worldMin;
        prim.max = box.worldMax;
        prim.centroid = box.worldCenter;
        prim.element = bodyH;
        prims.push_back(prim);
    }
}

void BVHTree::split(int parentIdx, int depth) {
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
    int mid = start + (count / 2);
    std::nth_element(
        prims.begin() + start, prims.begin() + mid, prims.begin() + start + count,
        [&](auto const& a, auto const& b) {
            return a.centroid[axis] < b.centroid[axis];
        });

    // create child nodes and init them
    Node* A = &nodes.emplace_back();
    A->selfIdx = nodes.size() - 1;

    Node* B = &nodes.emplace_back();
    B->selfIdx = nodes.size() - 1;

    initChild(parentIdx, A->selfIdx, true, start, mid, mid - start);
    initChild(parentIdx, B->selfIdx, false, mid, start + count, start + count - mid);

    // recursive split
    split(A->selfIdx, depth + 1);
    split(B->selfIdx, depth + 1);
}

void BVHTree::initChild(int parentIdx, int childIdx, bool isLeft, int start, int end, int count) {

    Node& parent = nodes[parentIdx];
    Node& child = nodes[childIdx];

    child.parentIdx = parentIdx;
    child.start = start;
    child.count = count;

    if (isLeft) parent.childAIdx = childIdx;
    else        parent.childBIdx = childIdx;

    // calculate both child's fatBox
    child.fatBox.worldMin = prims[start].min;
    child.fatBox.worldMax = prims[start].max;
    for (int i = start + 1; i < end; ++i) {
        child.fatBox.growToInclude(prims[i].min);
        child.fatBox.growToInclude(prims[i].max);
    }
}

void BVHTree::makeLeaf(int nodeIdx) {
    Node& leaf = nodes[nodeIdx];

    leaf.selfIdx = nodeIdx;
    leaf.isLeaf = true;
    leaf.element = prims[leaf.start].element;

    RigidBody* body = caches->bodies.get(leaf.element, FUNC_NAME);
    body->broadphaseHandle.leafIdx = leaf.selfIdx;

    leaf.tightBox = body->aabb;
    leaf.fatBox = body->aabb;
}

//------------------------------------------------------------------
//      Update Render Data
//------------------------------------------------------------------
void BVHTree::updateRenderData(Node& n) {
    n.fatBox.worldCenter = (n.fatBox.worldMin + n.fatBox.worldMax) * 0.5f;
    n.fatBox.worldHalfExtents = (n.fatBox.worldMax - n.fatBox.worldMin) * 0.5f;
}