#pragma once

#include "shaders/shader.h"
#include "camera.h"
#include "game_object.h"
#include "physics.h"
#include "engine_state.h"
#include "lighting/light_manager.h"
#include "shaders/shader_manager.h"
#include "lighting/shadow_manager.h"
#include "skybox/skybox_manager.h"

#include "debug/render_contact_points.h"
#include "debug/xyz_object.h"
#include "debug/sphere_outline_renderer.h"
#include "debug/quad_renderer.h"
#include "debug/arrow_renderer.h"
#include "debug/normals_renderer.h"

#include "bvh.h"
#include "scene_builder.h"
#include "editor/editor_main.h"
#include "player.h"
#include "raycast.h"


class Renderer {
public:
    void init(
        unsigned int width,
        unsigned int height,
        Editor::EditorMain& editor,
        Player& player,
        EngineState& engineState, 
        LightManager& lightManager, 
        ShaderManager& shaderManager,
        ShadowManager& shadowManager, 
        SkyboxManager& skyboxManager,
        MeshManager& meshManager
    );

    void render(
        Camera& camera,
        SceneBuilder& builder,
        PhysicsEngine& physics,
        GLuint qShadow[],
        GLuint qMain[],
        GLuint qDebug[],
        int writeIdx,
        const Editor::ViewportFBO* viewportFBO
    );

    void computeSceneBounds(SceneBuilder& builder);
    glm::mat4 computeLightSpaceMatrix();
    void setViewPort(unsigned int w, unsigned int h);
    void setViewProjection(Camera& camera, float aspect);

    void setShadowRender(glm::mat4& lightSpaceMatrix);
    void cleanupShadowRender();

    void setDefaultRender(glm::mat4& lightSpaceMatrix, int targetW, int targetH);
    void uploadLightsToShader(); 
    void uploadDirectionalLight(); 

    void renderScene(SceneBuilder& builder);

    void fillBatchInstances();
    void renderGameObjects(std::vector<GameObject>& objects);
    void renderGameObjectsShadow();
    void renderTerrain(SceneBuilder::TerrainData& data, bool sceneDirty, bool shadowPass);
    void renderLights() const; 
    void renderRayCastHit(GameObject* obj, Camera& camera, SceneBuilder& builder);

    // debug
    template<typename E>
    void renderBVH(const BVHTree<E>& tree, glm::vec3& nodeColor, glm::vec3& leafColor);
    void renderDebug(PhysicsEngine& physicsEngine, Camera& camera, std::vector<GameObject>& objects, unsigned int VAO_contactPoint, unsigned int VAO_xyz);
    void renderFrustum(const glm::mat4& viewProj);

    struct DebugMesh {
        Mesh* mesh;
        glm::mat4 model;
        glm::vec3 color;
        bool castsShadow;
    };
    std::vector<DebugMesh> debugMeshes;
    void fillDebugMeshes(PhysicsEngine* physicsEngine, std::vector<GameObject>& objects);
    void renderDebugMeshesShadow();
    void renderDebugMeshesDefault();

    Shader* defaultShader;
    Shader* debugShader;
    Shader* shadowShader;
    Shader* skyboxShader;

    float terrainRotationAngle = 0.0f;

    struct InstanceData {
        glm::mat4 model;
        glm::vec3 color;  // valfritt, om du vill ha per-instans-färg
    };
    void addObjectToBatch(GameObject* obj);
    void removeObjectFromBatch(GameObject* obj);
    void clearRenderBatches();

private:
    float screenWidth;
    float screenHeight;

    Editor::EditorMain* editor = nullptr;
    Player* player = nullptr;
    EngineState* engineState = nullptr;
    LightManager* lightManager = nullptr;
    ShaderManager* shaderManager = nullptr;
    ShadowManager* shadowManager = nullptr;
    SkyboxManager* skyboxManager = nullptr;
    MeshManager* meshManager = nullptr;

    AABBRenderer aabbRenderer;
    OOBBRenderer oobbRenderer;
    SphereOutlineRenderer sphereOutlineRenderer;
    QuadRenderer quadRenderer;
    ArrowRenderer arrowRenderer;
    NormalsRenderer normalsRenderer;

    float maxViewDistance = 10000000000000.0f;

    glm::vec3 sceneCenter = { 0.0f, 0.0f, 0.0f };
    glm::vec3 sceneMin = { 0.0f, 0.0f, 0.0f };
    glm::vec3 sceneMax = { 0.0f, 0.0f, 0.0f };
    std::vector<glm::vec3> sceneCorners; 

    unsigned int VAO_line;
    unsigned int VAO_xyz;
    unsigned int VAO_contactPoint;

    struct RenderBatch {
        Mesh* mesh;
        Shader* shader;
        GLuint  textureId;

        std::vector<GameObject*> objects;      // vilka som använder detta
        std::vector<InstanceData> instances;   // fylls varje frame
    };
    std::vector<RenderBatch> batches;

    constexpr static int INSTANCING_THRESHOLD = 3;
};