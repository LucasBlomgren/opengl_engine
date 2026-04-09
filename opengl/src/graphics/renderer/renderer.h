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
#include "debug/quad_renderer.h"

#include "renderer/debug_renderer.h"

#include "scene_builder.h"
#include "editor/editor_main.h"
#include "player.h"
#include "raycast.h"


class Renderer {
public:
    void init(
        unsigned int width,
        unsigned int height,
        World& world,
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
    void renderRayCastHit(GameObjectHandle& handle, Camera& camera, SceneBuilder& builder);

    Shader* defaultShader;
    Shader* debugShader;
    Shader* shadowShader;
    Shader* skyboxShader;

    struct InstanceData {
        glm::mat4 model;
        glm::vec3 color;
    };
    void addObjectToBatch(GameObjectHandle handle);
    void removeObjectFromBatch(GameObjectHandle handle);
    void clearRenderBatches();

private:
    DebugRenderer debugRenderer;

    float screenWidth;
    float screenHeight;

    World* world = nullptr;
    Editor::EditorMain* editor = nullptr;
    Player* player = nullptr;
    EngineState* engineState = nullptr;
    LightManager* lightManager = nullptr;
    ShaderManager* shaderManager = nullptr;
    ShadowManager* shadowManager = nullptr;
    SkyboxManager* skyboxManager = nullptr;
    MeshManager* meshManager = nullptr;

    QuadRenderer quadRenderer;

    float maxViewDistance = 10000000000000.0f;

    glm::vec3 sceneCenter = { 0.0f, 0.0f, 0.0f };
    glm::vec3 sceneMin = { 0.0f, 0.0f, 0.0f };
    glm::vec3 sceneMax = { 0.0f, 0.0f, 0.0f };
    std::vector<glm::vec3> sceneCorners; 

    struct RenderRef {
        GameObjectHandle objectH;
        int partIndex;
    };

    struct RenderBatch {
        Mesh* mesh;
        Shader* shader;
        GLuint  textureId;

        std::vector<RenderRef> parts;
        std::vector<InstanceData> instances;    // fylls varje frame
    };
    std::vector<RenderBatch> batches;

    constexpr static int INSTANCING_THRESHOLD = 3;
};