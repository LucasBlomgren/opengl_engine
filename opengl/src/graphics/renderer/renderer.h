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

#include "bvh.h"
#include "scene_builder.h"
#include "editor.h"
#include "raycast.h"

class Renderer {
public:
    void init(
        unsigned int width, 
        unsigned int height, 
        Editor& editor,
        EngineState& engineState, 
        LightManager& lightManager, 
        ShaderManager& shaderManager,
        ShadowManager& shadowManager, 
        SkyboxManager& skyboxManager
    );

    void render(
        Camera& camera,
        SceneBuilder& builder,
        PhysicsEngine& physics,
        GLuint qShadow[],
        GLuint qMain[],
        GLuint qDebug[],
        int writeIdx
    );

    void computeSceneBounds(SceneBuilder& builder);
    glm::mat4 computeLightSpaceMatrix();
    void setViewPort(unsigned int w, unsigned int h);
    void setViewProjection(Camera& camera);

    void setShadowRender(glm::mat4& lightSpaceMatrix);
    void cleanupShadowRender();

    void setDefaultRender(glm::mat4& lightSpaceMatrix);
    void uploadLightsToShader(); 
    void uploadDirectionalLight(); 

    void renderScene(Shader& shader, SceneBuilder& builder);

    void fillBatchInstances();
    void renderGameObjects(std::vector<GameObject>& objects);
    void renderGameObjectsShadow();
    void renderTerrain(Shader& shader, SceneBuilder::TerrainData& data, bool sceneDirty);
    void renderLights() const; 
    void renderRayCastHit(Camera& camera, SceneBuilder& builder); 

    // debug
    template<typename E>
    void renderBVH(BVHTree<E>& tree, glm::vec3& nodeColor, glm::vec3& leafColor);  
    void renderDebug(PhysicsEngine& physicsEngine, Camera& camera, std::vector<GameObject>& objects, unsigned int VAO_contactPoint, unsigned int VAO_xyz);
    void renderFrustum(const glm::mat4& viewProj);

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

    Editor* editor = nullptr;
    EngineState* engineState = nullptr;
    LightManager* lightManager = nullptr;
    ShaderManager* shaderManager = nullptr;
    ShadowManager* shadowManager = nullptr;
    SkyboxManager* skyboxManager = nullptr;

    AABBRenderer aabbRenderer;
    SphereOutlineRenderer sphereOutlineRenderer;

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