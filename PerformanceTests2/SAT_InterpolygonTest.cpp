#include "pch.h"
#include <CppUnitTest.h>
#include <chrono>
#include <glm/glm.hpp>
#include <vector>
#include "../opengl/src/physics/narrowphase/sat.h"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace PhysicsPerformanceTests {

TEST_CLASS(SATIntersectPolygonsPerformance) {
public:
    TEST_METHOD(BoxBoxIntersectionPerformance) {
        // Two overlapping boxes (typical collision scenario)
        std::vector<glm::vec3> boxAVertices = {
            {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
            {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
        };
        
        std::vector<glm::vec3> boxBVertices = {
            {0.5f, -1, -1}, {2.5f, -1, -1}, {2.5f, 1, -1}, {0.5f, 1, -1},
            {0.5f, -1, 1}, {2.5f, -1, 1}, {2.5f, 1, 1}, {0.5f, 1, 1}
        };

        std::vector<glm::vec3> normalsA = {
            glm::normalize(glm::vec3(1, 0, 0)),
            glm::normalize(glm::vec3(0, 1, 0)),
            glm::normalize(glm::vec3(0, 0, 1))
        };

        std::vector<glm::vec3> normalsB = {
            glm::normalize(glm::vec3(1, 0, 0)),
            glm::normalize(glm::vec3(0, 1, 0)),
            glm::normalize(glm::vec3(0, 0, 1))
        };

        // Warm up
        for (int w = 0; w < 100; w++) {
            SAT::Result result;
            result.depth = std::numeric_limits<float>::max();
            result.separationA = -std::numeric_limits<float>::infinity();
            result.separationB = -std::numeric_limits<float>::infinity();
            SAT::intersectPolygons(
                std::span(boxAVertices),
                std::span(boxBVertices),
                std::span(normalsA),
                std::span(normalsB),
                result
            );
        }

        // Measure: 5000 box-box SAT tests
        auto start = std::chrono::high_resolution_clock::now();
        
        int collisionCount = 0;
        for (int iter = 0; iter < 5000; iter++) {
            SAT::Result result;
            result.depth = std::numeric_limits<float>::max();
            result.separationA = -std::numeric_limits<float>::infinity();
            result.separationB = -std::numeric_limits<float>::infinity();
            
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
        
        std::string msg = "Box-box SAT 5000 tests: " + std::to_string(duration.count()) + "ms";
        Logger::WriteMessage(msg.c_str());
    }

    TEST_METHOD(RotatedBoxIntersectionPerformance) {
        // Rotated boxes force edge-edge testing
        std::vector<glm::vec3> boxAVertices = {
            {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
            {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
        };

        // 45-degree rotated box around Z
        float c = 0.707f;
        float s = 0.707f;
        std::vector<glm::vec3> boxBVertices = {
            {-c - s, -c + s, -1}, {c - s, -c - s, -1}, {c + s, c + s, -1}, {-c + s, c - s, -1},
            {-c - s, -c + s, 1}, {c - s, -c - s, 1}, {c + s, c + s, 1}, {-c + s, c - s, 1}
        };

        std::vector<glm::vec3> normalsA = {
            glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1)
        };

        std::vector<glm::vec3> normalsB = {
            glm::normalize(glm::vec3(c, s, 0)),
            glm::normalize(glm::vec3(-s, c, 0)),
            glm::vec3(0, 0, 1)
        };

        // Warm up
        for (int w = 0; w < 100; w++) {
            SAT::Result result;
            result.depth = std::numeric_limits<float>::max();
            result.separationA = -std::numeric_limits<float>::infinity();
            result.separationB = -std::numeric_limits<float>::infinity();
            SAT::intersectPolygons(
                std::span(boxAVertices),
                std::span(boxBVertices),
                std::span(normalsA),
                std::span(normalsB),
                result
            );
        }

        // Measure: 5000 tests with more edge-edge work
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int iter = 0; iter < 5000; iter++) {
            SAT::Result result;
            result.depth = std::numeric_limits<float>::max();
            result.separationA = -std::numeric_limits<float>::infinity();
            result.separationB = -std::numeric_limits<float>::infinity();

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

        std::string msg = "Rotated box SAT 5000 tests (more edge-edge): " + std::to_string(duration.count()) + "ms";
        Logger::WriteMessage(msg.c_str());
    }
};

}
