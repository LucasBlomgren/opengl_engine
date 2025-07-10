#pragma once

#include "shader.h"
#include "camera.h"
#include "game_object.h"
#include "physics.h"
#include "engine_state.h"
#include "light_manager.h"

#include "draw_contact_points.h"
#include "xyz_object.h"
#include "sphere_outline_renderer.h"

#include "bvh.h"
#include "scene_builder.h"

class Renderer {
public:
    void init(unsigned int width, unsigned int height, EngineState& state, LightManager& lightManager, Shader& shader, Shader& debugShader);
    void beginFrame() const;
    void setViewProjection(Camera& camera);
    void drawGameObjects(std::vector<GameObject>& objects, Camera& camera);
    void drawLights() const;
    void drawDebug(PhysicsEngine& physicsEngine, Camera& camera, std::vector<GameObject>& objects, unsigned int VAO_contactPoint, unsigned int VAO_xyz);

    template<typename E>
    void drawBVH(BVHTree<E>& tree, unsigned int VAO_line);

    void uploadLightsToShader();
    void uploadDirectionalLight();

    void drawTerrain(SceneBuilder::TerrainData& data, bool sceneDirty);

private:
    float screenWidth;
    float screenHeight;

    EngineState* engineState = nullptr;
    LightManager* lightManager = nullptr;
    Shader* shader = nullptr;
    Shader* debugShader = nullptr;

    AABBRenderer aabbRenderer;
    SphereOutlineRenderer sphereOutlineRenderer;

    float maxViewDistance = 10000.0f;
};