#pragma once

#include "core/engine_state.h"
#include "physics/physics.h"
#include "mesh/mesh_manager.h"
#include "shaders/shader_manager.h"

#include "debug/sphere_outline_renderer.h"
#include "debug/arrow_renderer.h"
#include "debug/normals_renderer.h"
#include "debug/aabb_renderer.h"
#include "debug/oobb_renderer.h"

class Renderer;

class DebugRenderer {
public:
    void init(const EngineState& engineState, const MeshManager& meshManager, const ShaderManager& shaderManager);

    void prepareFrame(
        PhysicsEngine& physics, 
        const std::vector<GameObject>& objects, 
        World& world, 
        GameObjectHandle& selectedObjectHandle, 
        int selectedSubPartIndex
    );

    void renderShadowPass() const;
    void renderOverlayPass(const PhysicsEngine& physicsEngine, const Camera& camera, const std::vector<GameObject>& objects, World& world);

    OOBBRenderer oobbRenderer;
    SphereOutlineRenderer sphereOutlineRenderer;

private:
    const EngineState* engineState = nullptr;
    Shader* litMeshShader;
    Shader* debugShapeShader;
    Shader* shadowMeshShader;

    AABBRenderer aabbRenderer;
    ArrowRenderer arrowRenderer;
    NormalsRenderer normalsRenderer;
    unsigned int VAO_contactPoint = 0;

    // lit debug meshes
    struct SceneDebugMesh {
        Mesh* mesh;
        glm::mat4 model;
        glm::vec3 color;
        bool castsShadow;
    };
    std::vector<SceneDebugMesh> sceneDebugMeshes;
    void prepareCollisionNormals(PhysicsEngine& physics, World& world);
    void prepareObjectLocalNormals(const std::vector<GameObject>& objects, World& world, GameObjectHandle& selectedObjectHandle, int selectedSubPartIndex);
    void prepareXYZAxes();

    // unlit debug shapes
    void renderAABBs(const std::vector<GameObject>& objects, World& world);
    void renderColliders(const std::vector<GameObject>& objects, const Camera& camera, World& world);
    void renderContactPoints(const std::unordered_map<size_t, Contact>& cache) const;
    void renderFrustum(const glm::mat4& viewProj) const;
    void renderBVHs(const PhysicsEngine& physics);
    template<class Tree> void renderBVH(const Tree& tree, const glm::vec3& nodeColor, const glm::vec3& leafColor);
    
    struct BVHColors {
        glm::vec3 awakeNode{ 0.80f, 0.40f, 0.00f }; // orange
        glm::vec3 awakeLeaf{ 1.00f, 0.65f, 0.20f };
        glm::vec3 asleepNode{ 0.13f, 0.33f, 0.67f }; // blue
        glm::vec3 asleepLeaf{ 0.45f, 0.70f, 1.00f };
        glm::vec3 staticNode{ 0.10f, 0.60f, 0.27f }; // green
        glm::vec3 staticLeaf{ 0.30f, 0.95f, 0.50f };
        glm::vec3 terrainNode{ 0.40f, 0.31f, 0.13f }; // brown 
        glm::vec3 terrainLeaf{ 0.70f, 0.50f, 0.25f };
    };
    const BVHColors BVH_COLORS;

    // outline colors for physics debug
    const glm::vec3 STATIC_COLOR = glm::vec3(0.30f, 0.95f, 0.50f);  // light green
    const glm::vec3 ASLEEP_COLOR = glm::vec3(0.0f, 0.0f, 1.0f);     // blue
    const glm::vec3 AWAKE_COLOR = glm::vec3(1.0f, 0.0f, 0.0f);      // red

    // debug shape line widths
    const float FRUSTUM_LINE_WIDTH = 2.0f;
    const float BVH_LEAF_LINE_WIDTH = 3.0f;
    const float BVH_NODE_LINE_WIDTH = 2.0f;

    const float AABB_LINE_WIDTH = 2.0f;
    const float OOBB_LINE_WIDTH = 2.0f;
    const float SPHERE_LINE_WIDTH = 4.0f;
};