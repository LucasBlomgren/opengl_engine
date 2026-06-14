#pragma once

#include "pch.h"
#include "bvh_temp_island.h"

//------------------------------
//    Init & Clear
//------------------------------
void TempIslandBVH::init(RuntimeCaches* caches, int allocSize) {
    this->caches = caches;

    nodes.reserve(allocSize * 2);
    prims.reserve(allocSize);
}

//------------------------------
//        Single Query
//------------------------------
void TempIslandBVH::singleQuery(
    const AABB& qBox,
    std::vector<RigidBodyHandle>& out) const
{
    if (nodes.empty() || rootIdx == -1) return;

    out.clear();
    out.reserve(TempIslandBVH::MaxCollisionBuf);

    constexpr int MaxStackSize = TempIslandBVH::MaxStackSize;
    const Node* stack[MaxStackSize];
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
                out.push_back(prim.element);
            }
        }
    }
}

//------------------------------------------------------------------
//      Update
//------------------------------------------------------------------
void TempIslandBVH::update(std::vector<RigidBodyHandle>& handles) {
    for (auto& n : nodes) n.dirty = false;

    updateLeaves();
    if (rootIdx != -1) refitNode(rootIdx);

    this->dirty = false;
}

void TempIslandBVH::updateLeaves() {
    for (int nodeIdx = 0; nodeIdx < static_cast<int>(nodes.size()); ++nodeIdx) {
        Node& leaf = nodes[nodeIdx];

        if (!leaf.isLeaf || !leaf.alive) {
            continue;
        }

        const int begin = leaf.start;
        const int end = leaf.start + leaf.count;

        if (begin < 0 || leaf.count <= 0) {
            continue;
        }

        // Update all primitive boxes and compute current leaf bounds.
        RigidBody* firstBody = caches->bodies.get(prims[begin].element, FUNC_NAME);
        prims[begin].box = firstBody->aabb;

        AABB leafBounds = prims[begin].box;

        for (int i = begin + 1; i < end; ++i) {
            RigidBody* body = caches->bodies.get(prims[i].element, FUNC_NAME);
            prims[i].box = body->aabb;

            leafBounds.growToInclude(prims[i].box.worldMin);
            leafBounds.growToInclude(prims[i].box.worldMax);
        }

        // If the leaf's current bodies still fit inside the old fat box,
        // no parent refit is needed.
        if (leaf.fatBox.contains(leafBounds)) {
            continue;
        }

        // Otherwise update the leaf fat box.
        leaf.fatBox = leafBounds;
        leaf.fatBox.grow(fatBoxMargin);

        for (int p = nodeIdx; p != -1; p = nodes[p].parentIdx) {
            nodes[p].dirty = true;
        }
    }
}

void TempIslandBVH::refitNode(int nodeIdx) {
    Node& node = nodes[nodeIdx];

    if (!node.dirty)
        return;

    if (node.isLeaf) {
        // leaf: fatBox is already expanded to include tightBox
        return;
    }

    // först barnen
    refitNode(node.childAIdx);
    refitNode(node.childBIdx);

    const Node& childA = nodes[node.childAIdx];
    const Node& childB = nodes[node.childBIdx];

    // när barnen är klara, unionera dem
    node.fatBox.worldMin = glm::min(childA.fatBox.worldMin, childB.fatBox.worldMin);
    node.fatBox.worldMax = glm::max(childA.fatBox.worldMax, childB.fatBox.worldMax);

    node.dirty = false;
}

//------------------------------------------------------------------
//      Build
//------------------------------------------------------------------
void TempIslandBVH::build(std::vector<RigidBodyHandle>& handles) {
    nodes.clear();
    // Fill prims vector with primitives from handles
    createPrimitives(handles);

    if (prims.empty()) return;

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
        } else {
            n.dirty = true;
        }
    }
    if (rootIdx != -1) refitNode(rootIdx);
}

void TempIslandBVH::createPrimitives(std::vector<RigidBodyHandle>& handles) {
    prims.clear();
    prims.reserve(handles.size());

    for (RigidBodyHandle& bodyH : handles) {
        RigidBody* body = caches->bodies.get(bodyH, FUNC_NAME);
        body->broadphaseHandle.leafIdx = -1;

        prims.emplace_back(body->aabb, bodyH);
    }
}

void TempIslandBVH::split(int parentIdx, int depth) {
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

void TempIslandBVH::initChild(int parentIdx, int childIdx, bool isLeft, int start, int end) {

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

void TempIslandBVH::makeLeaf(int nodeIdx) {
    Node& leaf = nodes[nodeIdx];

    leaf.selfIdx = nodeIdx;
    leaf.isLeaf = true;

    for (int i = leaf.start; i < leaf.start + leaf.count; ++i) {
        RigidBody* body = caches->bodies.get(prims[i].element, FUNC_NAME);
        body->broadphaseHandle.leafIdx = leaf.selfIdx;
    }
}