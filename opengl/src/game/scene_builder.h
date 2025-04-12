#pragma once

#include "physics.h"
#include "game_object.h"
#include "vertex.h"
#include "texture_manager.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class SceneBuilder {
public:
    int amountObjects;
    int amountStacks;

    void SetTextureManager(TextureManager* tm);
    void createScene(PhysicsEngine& physicsEngine, std::vector<Vertex>& cubeVertices, std::vector<unsigned int>& indices);
    GameObject& createObject(PhysicsEngine& physicsEngine, const std::string& textureName, glm::vec3 pos, glm::vec3 size, float mass, bool isStatic, std::vector<Vertex>& cubeVertices, std::vector<unsigned int>& indices);

    std::vector<GameObject>& GetGameObjectList();
private:
    TextureManager* textureManager = nullptr;

    int objectId;
    std::vector<GameObject> GameObjectList;
};