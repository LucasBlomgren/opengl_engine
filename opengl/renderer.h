#pragma once

#include "shader.h"
#include "camera.h"
#include "GameObject.h"
#include "physics.h"
#include "engineState.h"

#include "drawContactPoints.h"
#include "worldFrame.h"
#include "xyzObject.h"

class Renderer 
{
public:
    void Init(unsigned int width, unsigned int height, EngineState& state, Shader& shader);
    void BeginFrame();
    void SetViewProjection(Camera& camera);
    void DrawGameObjects(std::vector<GameObject>& objects, unsigned int VAO_line);
    void DrawDebug(PhysicsEngine& physicsEngine, unsigned int VAO_contactPoint, unsigned int VAO_xyz, unsigned int VAO_worldFrame);

private:
    EngineState* engineState = nullptr;
    glm::mat4 projection;
    glm::mat4 view;
    Shader* shader = nullptr;
    float screenWidth;
    float screenHeight;
    float maxViewDistance = 5000.0f;
};