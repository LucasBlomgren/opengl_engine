#include "renderer.h"

template void Renderer::drawBVH(BVHTree<GameObject>&, unsigned int);
template void Renderer::drawBVH(BVHTree<Tri>&, unsigned int);

// ------ Initialization functions ------
void Renderer::init(unsigned int width, unsigned int height, EngineState& engineState, LightManager& lightManager, Shader& shader, Shader& debugShader)
{   
    screenWidth = (float) width;
    screenHeight = (float) height;

    this->engineState = &engineState;
    this->lightManager = &lightManager;

    aabbRenderer.InitShared();
    sphereOutlineRenderer.init();

    this->shader = &shader;
    this->debugShader = &debugShader;
    shader.use();
}

void Renderer::uploadLightsToShader() {

    shader->use();
    const std::vector<Light>& lights = lightManager->getLights();

    for (int i = 0; i < lights.size(); ++i) {
        const Light& light = lights[i];
        std::string base = "pointLights[" + std::to_string(i) + "]";

        shader->setVec3(base + ".position", light.position);
        shader->setVec3(base + ".color", light.color);

        shader->setVec3(base + ".ambient", light.ambient * light.intensity);
        shader->setVec3(base + ".diffuse", light.diffuse * light.intensity);
        shader->setVec3(base + ".specular", light.specular * light.intensity);

        shader->setFloat(base + ".constant", light.constant);
        shader->setFloat(base + ".linear", light.linear);
        shader->setFloat(base + ".quadratic", light.quadratic);
    }

    shader->setInt("numPointLights", static_cast<int>(lights.size()));
}

void Renderer::uploadDirectionalLight() {
    const DirectionalLight& light = lightManager->getDirectionalLight();

    shader->use();
    shader->setVec3("dirLight.direction", light.direction);
    shader->setVec3("dirLight.ambient", light.ambient);
    shader->setVec3("dirLight.diffuse", light.diffuse);
    shader->setVec3("dirLight.specular", light.specular);
}

// ------ Render frame functions ------
void Renderer::beginFrame() const
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
}

void Renderer::setViewProjection(Camera& camera)
{
    // projection matrix 
    glm::mat4 projection = glm::perspective(glm::radians(camera.zoom), screenWidth / screenHeight, 0.1f, maxViewDistance);
    // camera/view transformation
    glm::mat4 view = camera.GetViewMatrix();

    shader->use();
    shader->setMat4("projection", projection);
    shader->setMat4("view", view);
    shader->setVec3("viewPos", camera.position);

    debugShader->use();
    debugShader->setMat4("projection", projection);
    debugShader->setMat4("view", view);
    debugShader->setVec3("viewPos", camera.position);
}

void Renderer::drawLights() const {
    const std::vector<Light>& lights = lightManager->getLights();
    for (const Light& light : lights) {
        light.draw(*shader);
    }
}

void Renderer::drawGameObjects(std::vector<GameObject>& objects, Camera& camera) {
    shader->use();
    for (GameObject& obj : objects) {
        obj.drawMesh(*shader);

        if (obj.isRaycastHit) { 
            debugShader->use(); 
            debugShader->setBool("debug.useUniformColor", true);
            if (obj.colliderType == ColliderType::CUBOID) { 
                obj.oobbRenderer.drawBox(*debugShader, obj.modelMatrix, obj.asleep, obj.isRaycastHit); 
            }
            else if (obj.colliderType == ColliderType::SPHERE) { 
                sphereOutlineRenderer.draw(*debugShader, camera.position, obj.position, obj.radius, obj.asleep, obj.isRaycastHit); 
            }
            debugShader->setBool("debug.useUniformColor", false);
            shader->use(); 
        }
        obj.isRaycastHit = false; 
    }
}

void Renderer::drawTerrain(SceneBuilder::TerrainData& data, bool sceneDirty) {
    if (data.triangles.size() == 0) {
        return;
    }

    std::vector<Vertex>& vertices = data.vertices;
    std::vector<uint32_t>& indices = data.indices;

    static GLuint VAO = 0;
    static GLuint VBO = 0;
    static GLuint EBO    = 0;

    static float minHeight = std::numeric_limits<float>::max();
    static float maxHeight = std::numeric_limits<float>::lowest();

    if (sceneDirty) {
        if (VAO != 0) { 
            glDeleteVertexArrays(1, &VAO);
            glDeleteBuffers(1, &VBO); 
            glDeleteBuffers(1, &EBO);
        }
        VAO = 0; 
        VBO = 0; 
        EBO = 0;
    }

    // --- Wireframe setup: skapas bara en gång ----
    if (VAO == 0) {
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);


        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32_t), indices.data(), GL_STATIC_DRAW);

        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position)); 
        glEnableVertexAttribArray(0); 
        // normal attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal)); 
        glEnableVertexAttribArray(1); 
        // texture coord attribute 
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords)); 
        glEnableVertexAttribArray(2); 

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    static glm::mat4 model = glm::mat4(1.0f);
    shader->use();
    shader->setMat4("model", model);

    shader->setBool("useTexture", true);
    shader->setBool("useUniformColor", true);
    shader->setVec3("uColor", glm::vec3(0,1,0));

    glBindTexture(GL_TEXTURE_2D, 3);
    glBindVertexArray(VAO);

    // --- Fyllning ---
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, nullptr);

    debugShader->use();
    debugShader->setMat4("model", model);
    debugShader->setInt("debug.objectType", 0);
    debugShader->setBool("debug.useUniformColor", true); 
    debugShader->setVec3("debug.uColor", glm::vec3{ 0.2 });

    // --- Wireframe ---
    glEnable(GL_POLYGON_OFFSET_LINE); 
    // Skjuter ut linjerna en aning så att de inte z-fightar med fyllningen
    glPolygonOffset(-1.0f, -1.0f); 
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); 
    glLineWidth(1.f); 
    //glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, nullptr); 
    glDisable(GL_POLYGON_OFFSET_LINE); 
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Renderer::drawDebug(PhysicsEngine& physicsEngine, Camera& camera, std::vector<GameObject>& objects, unsigned int VAO_contactPoint, unsigned int VAO_xyz) {
    debugShader->use();
    debugShader->setBool("debug.useUniformColor", true); 
    debugShader->setInt("debug.objectType", 0); 

    if (engineState->getShowAABB()) {
        glLineWidth(2.0f); 
        glBindVertexArray(aabbRenderer.sVAO);  
        glm::vec3 color{ 0.9f, 0.7f, 0.2f };

        for (GameObject& obj : objects) {
            aabbRenderer.updateModel(obj.aabb, false);
            aabbRenderer.draw(color, *debugShader);
        }
    }

    if (engineState->getShowColliders()) {
        glLineWidth(2.0f);
        for (GameObject& obj : objects) {
            debugShader->setBool("debug.useUniformColor", true);
            if (obj.colliderType == ColliderType::CUBOID) {
                obj.oobbRenderer.drawBox(*debugShader, obj.modelMatrix, obj.asleep, obj.isRaycastHit);
            }
            else if (obj.colliderType == ColliderType::SPHERE) {
                sphereOutlineRenderer.draw(*debugShader, camera.position, obj.position, obj.radius, obj.asleep, obj.isRaycastHit);
            }
        }
    }

    if (engineState->getShowNormals()) {
        glLineWidth(4.0f); 
        glBindVertexArray(objects[0].oobbRenderer.VAO_normals);
        debugShader->setInt("debug.objectType", 0);
        debugShader->setBool("debug.useUniformColor", false);

        for (GameObject& obj : objects) {
            obj.oobbRenderer.drawNormals(*debugShader, obj.modelMatrix);
        }
    }

    if (engineState->getShowContactPoints()) {
        debugShader->setInt("debug.objectType", 2);
        debugShader->setVec3("debug.uColor", glm::vec3(0, 250, 154));
        debugShader->setBool("debug.useUniformColor", true);

        const auto& contactCache = physicsEngine.GetContactCache();
        for (const auto& pair : contactCache) {
            const Contact& contact = pair.second;

            for (int i = 0; i < contact.points.size(); i++) {
                if (!contact.points[i].wasUsedThisFrame) {
                    continue;
                }
                drawContactPoint(*debugShader, VAO_contactPoint, contact.points[i].globalCoord);
            }
        }
    }

    draw_xyzObject(*debugShader, VAO_xyz);
}

template<typename E>
void Renderer::drawBVH(BVHTree<E>& tree, unsigned int VAO_line) {
    if (!engineState->getShowBVH()) {
        return;
    }

    debugShader->use();
    debugShader->setBool("debug.useUniformColor", true);

    glLineWidth(2.0f); 
    glBindVertexArray(aabbRenderer.sVAO); 

    glm::vec3 color;
    AABB toDraw;

    for (auto& node : tree.nodes) {
        if (node.isLeaf) {
            toDraw = node.tightBox;
            toDraw.centroid = (toDraw.wMin + toDraw.wMax) * 0.5f;
            toDraw.halfExtents = (toDraw.wMax - toDraw.wMin) * 0.5f;
            color = glm::vec3(0, 1, 1);
        }
        else {
            toDraw = node.fatBox;
            color = glm::vec3(1, 0, 1);
        }

        aabbRenderer.updateModel(toDraw, /*asleep=*/false);
        aabbRenderer.draw(color, *debugShader);
    }
}