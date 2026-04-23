#include "pch.h"
#include <CppUnitTest.h>
#include <glm/glm.hpp>
#include <vector>
#include <chrono>
#include "../opengl/src/physics/SAT.h"
#include "Collider.h"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace PhysicsTests {

TEST_CLASS(SAT_PerformanceTest) {
public:
    TEST_METHOD(TestBoxBoxCollisionPerformance) {
        // Setup: Create two box colliders in typical collision scenario
        std::vector<glm::vec3> boxAVertices = {
            {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
            {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
        };
        std::vector<glm::vec3> boxBVertices = {
            {0, -1, -1}, {2, -1, -1}, {2, 1, -1}, {0, 1, -1},
            {0, -1, 1}, {2, -1, 1}, {2, 1, 1}, {0, 1, 1}
        };

        // Box normals (unit axes)
        std::vector<glm::vec3> normalsA = {
            {1, 0, 0}, {0, 1, 0}, {0, 0, 1}
        };
        std::vector<glm::vec3> normalsB = {
            {1, 0, 0}, {0, 1, 0}, {0, 0, 1}
        };

        // Run benchmark: 10,000 SAT tests (typical scenario for one physics frame)
        auto start = std::chrono::high_resolution_clock::now();
        
        int collisionCount = 0;
        for (int iter = 0; iter < 10000; iter++) {
            SAT::Result result;
            result.depth = std::numeric_limits<float>::max();
            result.separationA = std::numeric_limits<float>::lowest();
            result.separationB = std::numeric_limits<float>::lowest();

            bool collision = SAT::intersectPolygons(
                std::span(boxAVertices),
                std::span(boxBVertices),
                std::span(normalsA),
                std::span(normalsB),
                result
            );

            if (collision) collisionCount++;
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        Assert::IsTrue(collisionCount > 0, L"Expected collisions detected");
        
        // Log performance result
        std::string msg = "Box-box SAT 10k tests: " + std::to_string(duration.count()) + "ms";
        Logger::WriteMessage(msg.c_str());
    }

    TEST_METHOD(TestEdgeEdgeCasesPerformance) {
        // Test with rotated boxes to force edge-edge tests
        std::vector<glm::vec3> boxAVertices = {
            {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
            {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
        };

        // Rotated box (45 degrees around Z)
        float c = 0.707f; // cos(45°)
        float s = 0.707f; // sin(45°)
        std::vector<glm::vec3> boxBVertices = {
            {-c - s, -c + s, -1}, {c - s, -c - s, -1}, {c + s, c + s, -1}, {-c + s, c - s, -1},
            {-c - s, -c + s, 1}, {c - s, -c - s, 1}, {c + s, c + s, 1}, {-c + s, c - s, 1}
        };

        std::vector<glm::vec3> normalsA = {
            {1, 0, 0}, {0, 1, 0}, {0, 0, 1}
        };
        std::vector<glm::vec3> normalsB = {
            {c, s, 0}, {-s, c, 0}, {0, 0, 1}
        };

        auto start = std::chrono::high_resolution_clock::now();
        
        for (int iter = 0; iter < 10000; iter++) {
            SAT::Result result;
            result.depth = std::numeric_limits<float>::max();
            result.separationA = std::numeric_limits<float>::lowest();
            result.separationB = std::numeric_limits<float>::lowest();

            SAT::intersectPolygons(
                std::span(boxAVertices),
                std::span(boxBVertices),
                std::span(normalsA),
                std::span(normalsB),
                result
            );
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        std::string msg = "Rotated box SAT 10k tests (more edge-edge): " + std::to_string(duration.count()) + "ms";
        Logger::WriteMessage(msg.c_str());
    }
};

}
