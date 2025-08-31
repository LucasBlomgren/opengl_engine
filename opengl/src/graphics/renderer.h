#pragma once

#include "shader.h"
#include "camera.h"
#include "game_object.h"
#include "physics.h"
#include "engine_state.h"
#include "light_manager.h"
#include "shadow_manager.h"
#include "skybox_manager.h"

#include "render_contact_points.h"
#include "xyz_object.h"
#include "sphere_outline_renderer.h"

#include "bvh.h"
#include "scene_builder.h"
#include "editor.h"

class Renderer {
public:
    void init(
        unsigned int width, 
        unsigned int height, 
        EngineState& engineState, 
        LightManager& lightManager, 
        ShadowManager& shadowManager, 
        SkyboxManager& skyboxManager
    );

    void update(
        Camera& camera,
        SceneBuilder& builder,
        PhysicsEngine& physics,
        Editor& editor,
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

    void renderScene(
        Shader& shader,
        Camera& camera,
        SceneBuilder& builder,
        PhysicsEngine& physics,
        Editor& editor
    );

    void renderGameObjects(Shader& shader, std::vector<GameObject>& objects, Camera& camera);
    void renderTerrain(Shader& shader, SceneBuilder::TerrainData& data, bool sceneDirty);
    void renderLights() const; 
    void renderRayCastHit(Camera& camera, SceneBuilder& builder); 

    // debug
    template<typename E>
    void renderBVH(BVHTree<E>& tree, glm::vec3& color);  
    void renderDebug(PhysicsEngine& physicsEngine, Camera& camera, std::vector<GameObject>& objects, unsigned int VAO_contactPoint, unsigned int VAO_xyz);
    void renderFrustum(const glm::mat4& viewProj, Shader& debugShader);

    Shader defaultShader;
    Shader debugShader;
    Shader shadowShader;
    Shader skyboxShader;

    float terrainRotationAngle = 0.0f;

private:
    float screenWidth;
    float screenHeight;

    EngineState* engineState = nullptr;
    LightManager* lightManager = nullptr;
    ShadowManager* shadowManager = nullptr;
    SkyboxManager* skyboxManager = nullptr;

    AABBRenderer aabbRenderer;
    SphereOutlineRenderer sphereOutlineRenderer;

    float maxViewDistance = 10000.0f;

    glm::vec3 sceneCenter = { 0.0f, 0.0f, 0.0f };
    glm::vec3 sceneMin = { 0.0f, 0.0f, 0.0f };
    glm::vec3 sceneMax = { 0.0f, 0.0f, 0.0f };
    std::vector<glm::vec3> sceneCorners; 

    unsigned int VAO_line;
    unsigned int VAO_xyz;
    unsigned int VAO_contactPoint;
};