#include "renderer.h"

template void Renderer::drawBVH(BVHTree<GameObject>&, unsigned int);
template void Renderer::drawBVH(BVHTree<Tri>&, unsigned int);

void Renderer::init(unsigned int width, unsigned int height, EngineState& engineState, LightManager& lightManager, Shader& shader, Shader& debugShader)
{   
    screenWidth = (float) width;
    screenHeight = (float) height;

    this->engineState = &engineState;
    this->lightManager = &lightManager;

    aabbRenderer.InitShared();

    this->shader = &shader;
    this->debugShader = &debugShader;
    shader.use();
}

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

void Renderer::drawGameObjects(std::vector<GameObject>& objects, unsigned int VAO_line) {
    shader->use();
    for (GameObject& obj : objects) {
        obj.drawMesh(*shader);

        if (obj.isRaycastHit) {
            obj.oobbRenderer.drawBox(*debugShader, obj.modelMatrix, obj.asleep, obj.isRaycastHit);
            shader->use();
        }
        obj.isRaycastHit = false;
    }
}

template<typename E>
void Renderer::drawBVH(BVHTree<E>& tree, unsigned int VAO_line) {
    if (!engineState->getShowBVH()) 
        return;

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

void Renderer::drawLights() const {
    const std::vector<Light>& lights = lightManager->getLights();
    for (const Light& light : lights) {
        light.draw(*shader);
    }
}

void Renderer::drawDebug(PhysicsEngine& physicsEngine, std::vector<GameObject>& objects, unsigned int VAO_contactPoint, unsigned int VAO_xyz) {
    debugShader->use();

    if (engineState->getShowAABB()) {
        glm::vec3 color{ 0.9f, 0.7f, 0.2f };
        for (GameObject& obj : objects) {
            aabbRenderer.updateModel(obj.aabb, false);
            aabbRenderer.draw(color, *debugShader);
        }
    }

    if (engineState->getShowOOBB()) {
        for (GameObject& obj : objects) {
            obj.oobbRenderer.drawBox(*debugShader, obj.modelMatrix, obj.asleep, obj.isRaycastHit);
        }
    }

    if (engineState->getShowNormals()) {
        for (GameObject& obj : objects) {
            obj.oobbRenderer.drawNormals(*debugShader, obj.modelMatrix);
        }
    }

    if (engineState->getShowContactPoints()) {
        const auto& contactCache = physicsEngine.GetContactCache();
        for (const auto& pair : contactCache) {
            const Contact& contact = pair.second;
            for (int i = 0; i < contact.points.size(); i++)
                drawContactPoint(*debugShader, VAO_contactPoint, contact.points[i].globalCoord);
        }
    }

    draw_xyzObject(*debugShader, VAO_xyz);
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

void Renderer::drawTerrain(std::vector<Tri>& triangles) {
    // --- Wireframe setup: skapas bara en gång ----
    static GLuint triVAO = 0, triVBO = 0;
    if (triVAO == 0) {
        glGenVertexArrays(1, &triVAO);
        glGenBuffers(1, &triVBO);

        glBindVertexArray(triVAO);
        glBindBuffer(GL_ARRAY_BUFFER, triVBO);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    // Lägg ihop alla vertexdata i en enkel float-array
    std::vector<float> vertexData;
    vertexData.reserve(triangles.size() * 3 * 3);
    for (auto& tri : triangles) {
        vertexData.push_back(tri.v0.x);
        vertexData.push_back(tri.v0.y);
        vertexData.push_back(tri.v0.z);

        vertexData.push_back(tri.v1.x);
        vertexData.push_back(tri.v1.y);
        vertexData.push_back(tri.v1.z);

        vertexData.push_back(tri.v2.x);
        vertexData.push_back(tri.v2.y);
        vertexData.push_back(tri.v2.z);
    }

    //get min and maxheight of all tris
    float minHeight = std::numeric_limits<float>::max();
    float maxHeight = std::numeric_limits<float>::lowest();
    for (const auto& tri : triangles) {
        minHeight = std::min({ minHeight, tri.v0.y, tri.v1.y, tri.v2.y });
        maxHeight = std::max({ maxHeight, tri.v0.y, tri.v1.y, tri.v2.y });
    }

    glm::mat4 model = glm::mat4(1.0f);
    debugShader->use();
    debugShader->setMat4("model", model);
    debugShader->setBool("debug.useUniformColor", true);
    debugShader->setBool("terrain", true);
    debugShader->setFloat("maxHeight", maxHeight);
    debugShader->setFloat("minHeight", minHeight);

    glBindBuffer(GL_ARRAY_BUFFER, triVBO);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_DYNAMIC_DRAW);
    glBindVertexArray(triVAO);

    GLsizei vertexCount = GLsizei(vertexData.size() / 3);
    // --- Fyllning ---
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawArrays(GL_TRIANGLES, 0, vertexCount);

    debugShader->setVec3("debug.uColor", glm::vec3{ 0.15f });
    debugShader->setBool("terrain", false);

    // --- Wireframe ---
    glEnable(GL_POLYGON_OFFSET_LINE);
    // Skjuter ut linjerna en aning så att de inte z-fightar med fyllningen
    glPolygonOffset(-1.0f, -1.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(0.1f);
    glDrawArrays(GL_TRIANGLES, 0, vertexCount);
    glDisable(GL_POLYGON_OFFSET_LINE);

    // --- Återställ polygon mode om du vill ---
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}