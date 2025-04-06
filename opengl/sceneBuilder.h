#pragma once

#include "physics.h"
#include "GameObject.h"
#include "vertex.h"
#include "textureManager.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class SceneBuilder {
public:

    int objectId;
    int amountObjects;
    int amountStacks;

    //float lightStrength;
    //glm::vec3 lightStartingPos;
    //glm::vec3 lightPos;
    //Mesh* light;

    void SetTextureManager(TextureManager* tm);
    void createScene(PhysicsEngine& physicsEngine, std::vector<GameObject>& meshList, std::vector<Vertex>& cubeVertices, std::vector<unsigned int>& indices);
    void createObject(PhysicsEngine& physicsEngine, std::vector<GameObject>& meshList, const std::string& textureName, glm::vec3 pos, glm::vec3 size, float mass, bool isStatic, std::vector<Vertex>& cubeVertices, std::vector<unsigned int>& indices);
private:
    TextureManager* textureManager = nullptr;
};