#pragma once

#include "physics/physics.h"
#include "mesh/mesh_manager.h"
#include "shaders/shader_manager.h"

#include "debug/render_contact_points.h"
#include "debug/xyz_object.h"
#include "debug/sphere_outline_renderer.h"
#include "debug/quad_renderer.h"
#include "debug/arrow_renderer.h"
#include "debug/normals_renderer.h"

class Renderer;

class DebugRenderer {
public:
    void init(const EngineState& engineState, const MeshManager& meshManager, const ShaderManager& shaderManager);

    void prepareFrame(const PhysicsEngine& physics, const std::vector<GameObject>& objects);
    void renderShadowPass();
    void renderOverlayPass(const PhysicsEngine& physicsEngine, const Camera& camera, const std::vector<GameObject>& objects);

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
    unsigned int VAO_contactPoint;

    // lit debug meshes
    struct SceneDebugMesh {
        Mesh* mesh;
        glm::mat4 model;
        glm::vec3 color;
        bool castsShadow;
    };
    std::vector<SceneDebugMesh> sceneDebugMeshes;
    void prepareCollisionNormals(const PhysicsEngine& physics);
    void prepareObjectLocalNormals(const std::vector<GameObject>& objects);
    void prepareXYZAxes();

    // unlit debug shapes
    void renderAABBs(const std::vector<GameObject>& objects);
    void renderColliders(const std::vector<GameObject>& objects, const Camera& camera);
    void renderContactPoints(const std::unordered_map<size_t, Contact>& cache);
    void renderFrustum(const glm::mat4& viewProj);
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
    BVHColors bvhColors;
};