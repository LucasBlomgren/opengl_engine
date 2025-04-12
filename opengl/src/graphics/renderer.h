#pragma once

#include "shader.h"
#include "camera.h"
#include "game_object.h"
#include "physics.h"
#include "engine_state.h"
#include "light_manager.h"

#include "draw_contact_points.h"
#include "world_frame.h"
#include "xyz_object.h"

class Renderer 
{
public:
    void Init(unsigned int width, unsigned int height, EngineState& state, LightManager& lightManager, Shader& shader);
    void BeginFrame();
    void SetViewProjection(Camera& camera);
    void DrawGameObjects(std::vector<GameObject>& objects, unsigned int VAO_line);
    void DrawDebug(PhysicsEngine& physicsEngine, unsigned int VAO_contactPoint, unsigned int VAO_xyz, unsigned int VAO_worldFrame);

    void UploadLightsToShader();
    void UploadDirectionalLight();

private:
    float screenWidth;
    float screenHeight;

    EngineState* engineState = nullptr;
    LightManager* lightManager = nullptr;
    Shader* shader = nullptr;

    glm::mat4 projection;
    glm::mat4 view;
    float maxViewDistance = 5000.0f;
};