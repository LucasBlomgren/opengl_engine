#pragma once

#include "physics.h"
#include "game_object.h"
#include "geometry/vertex.h"
#include "texture_manager.h"
#include "light_manager.h"

#include "geometry/sphere_data.h"
#include "geometry/cube_data.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class SceneBuilder 
{
public:
    void setPointers(TextureManager* tm, LightManager* lm);
    void createScene(PhysicsEngine& physicsEngine);
    GameObject& createObject(
        PhysicsEngine& physicsEngine,
        const std::string& textureName,
        glm::vec3 pos,
        glm::vec3 size,
        float mass,
        bool isStatic,
        glm::quat orientation = glm::quat(1, 0, 0, 0),
        float sleepCounterThreshold = 0.5f
    );
    std::vector<GameObject>& getGameObjectList();
    void toggleDayTime();

private:
    TextureManager* textureManager = nullptr;
    LightManager* lightManager = nullptr;

    bool dayTime = false;

    int objectId;
    std::vector<GameObject> GameObjectList;
};