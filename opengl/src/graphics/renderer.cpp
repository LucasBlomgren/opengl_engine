#include "renderer.h"

void Renderer::init(unsigned int width, unsigned int height, EngineState& engineState, LightManager& lightManager, Shader& shader, Shader& debugShader)
{   
    screenWidth = (float) width;
    screenHeight = (float) height;

    this->engineState = &engineState;
    this->lightManager = &lightManager;

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
    glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), screenWidth / screenHeight, 0.1f, maxViewDistance);
    // camera/view transformation
    glm::mat4 view = camera.GetViewMatrix();

    shader->use();
    shader->setMat4("projection", projection);
    shader->setMat4("view", view);
    shader->setVec3("viewPos", camera.Position);

    debugShader->use();
    debugShader->setMat4("projection", projection);
    debugShader->setMat4("view", view);
    debugShader->setVec3("viewPos", camera.Position);
}

void Renderer::drawGameObjects(std::vector<GameObject>& objects, unsigned int VAO_line) const
{
    for (GameObject& obj : objects) {
        glBindTexture(GL_TEXTURE_2D, obj.textureID);
        shader->use();
        obj.drawMesh(*shader);

        debugShader->use();
        if (engineState->getShowAABB()) {
            //obj.AABB.draw(*debugShader, obj.asleep);
            obj.aabbRenderer.drawBox(obj.AABB, *debugShader, obj.asleep);
        }

        if (engineState->getShowOOBB()) {
            obj.oobbRenderer.drawBox(*debugShader, obj.modelMatrix, obj.asleep);
        }
        if (engineState->getShowNormals()) {
            obj.oobbRenderer.drawNormals(*debugShader, obj.modelMatrix);
        }
    }
}

void Renderer::drawLights() const
{
    const std::vector<Light>& lights = lightManager->getLights();
    for (const Light& light : lights) {
        light.draw(*shader);
    }
}

void Renderer::drawDebug(PhysicsEngine& physicsEngine, unsigned int VAO_contactPoint, unsigned int VAO_xyz, unsigned int VAO_worldFrame) {
    debugShader->use();
    if (engineState->getShowContactPoints()) {
        const auto& contactCache = physicsEngine.GetContactCache();
        for (const auto& pair : contactCache) {
            const Contact& contact = pair.second;
            for (int i = 0; i < contact.counter; i++)
                drawContactPoint(*debugShader, VAO_contactPoint, contact.points[i].globalCoord);
        }
    }

    draw_xyzObject(*debugShader, VAO_xyz);
    //draw_worldFrame(*debugShader, VAO_worldFrame);
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