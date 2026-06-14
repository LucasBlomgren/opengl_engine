#pragma once

namespace {
    struct SapEdge {
        float value = 0.0f;
        int bodyIdx = -1;   // index into awakeHandles / boxes
        bool isMin = false;
    };

    bool sapEdgeLess(const SapEdge& a, const SapEdge& b) {
        if (a.value != b.value) {
            return a.value < b.value;
        }

        // Min före max så touching intervals räknas som overlap.
        if (a.isMin != b.isMin) {
            return a.isMin && !b.isMin;
        }

        return a.bodyIdx < b.bodyIdx;
    }

    void insertionSortSapEdges(std::vector<SapEdge>& edges) {
        for (int i = 1; i < static_cast<int>(edges.size()); ++i) {
            SapEdge key = edges[i];
            int j = i - 1;

            while (j >= 0 && sapEdgeLess(key, edges[j])) {
                edges[j + 1] = edges[j];
                --j;
            }

            edges[j + 1] = key;
        }
    }

    int chooseLargestExtentAxis(const std::vector<AABB>& boxes) {
        if (boxes.empty()) {
            return 0;
        }

        glm::vec3 minP = boxes[0].worldMin;
        glm::vec3 maxP = boxes[0].worldMax;

        for (int i = 1; i < static_cast<int>(boxes.size()); ++i) {
            minP = glm::min(minP, boxes[i].worldMin);
            maxP = glm::max(maxP, boxes[i].worldMax);
        }

        glm::vec3 extent = maxP - minP;

        if (extent.x > extent.y) {
            return extent.x > extent.z ? 0 : 2;
        }
        else {
            return extent.y > extent.z ? 1 : 2;
        }
    }

    void removeActiveIndex(std::vector<int>& active, int idx) {
        for (int i = 0; i < static_cast<int>(active.size()); ++i) {
            if (active[i] == idx) {
                active[i] = active.back();
                active.pop_back();
                return;
            }
        }
    }
}