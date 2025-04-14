#include "renderer.h"

void Renderer::init(unsigned int width, unsigned int height, EngineState& engineState, LightManager& lightManager, Shader& shader)
{   
    screenWidth = (float) width;
    screenHeight = (float) height;

    this->engineState = &engineState;
    this->lightManager = &lightManager;

    this->shader = &shader;
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
    shader->setMat4("projection", projection);

    // camera/view transformation
    glm::mat4 view = camera.GetViewMatrix();
    shader->setMat4("view", view);

    shader->setVec3("viewPos", camera.Position);
}

void Renderer::drawGameObjects(std::vector<GameObject>& objects, unsigned int VAO_line) const
{
    for (GameObject& obj : objects) {
        glBindTexture(GL_TEXTURE_2D, obj.textureID);
        obj.drawMesh(*shader);

        if (engineState->getShowAABB())
            obj.AABB.draw(*shader, obj.asleep);

        if (engineState->getShowOOBB()) {
            if (!obj.asleep or obj.isStatic) {
                obj.OOBB_shouldUpdate = true;
                obj.updateOOBB();
            }
            obj.OOBB.draw(*shader, obj.asleep, obj.isStatic);
        }
        if (engineState->getShowNormals())
            obj.OOBB.drawNormals(*shader, VAO_line, obj.position);
    }
}

void Renderer::drawDebug(PhysicsEngine& physicsEngine, unsigned int VAO_contactPoint, unsigned int VAO_xyz, unsigned int VAO_worldFrame) {
    if (engineState->getShowContactPoints()) {
        shader->setBool("useUniformColor", true);
        shader->setVec3("uColor", glm::vec3(0, 250, 154));
        shader->setBool("isContactPoint", true);
        shader->setBool("useTexture", false);

        const auto& contactCache = physicsEngine.GetContactCache();
        for (const auto& pair : contactCache) {
            const Contact& contact = pair.second;
            for (int i = 0; i < contact.counter; i++)
                drawContactPoint(*shader, VAO_contactPoint, contact.points[i].globalCoord);
        }
        shader->setBool("isContactPoint", false);
    }

    draw_xyzObject(*shader, VAO_xyz);
    //draw_worldFrame(*shader, VAO_worldFrame);
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
    shader->setVec3("dirLight.direction", light.direction);
    shader->setVec3("dirLight.ambient", light.ambient);
    shader->setVec3("dirLight.diffuse", light.diffuse);
    shader->setVec3("dirLight.specular", light.specular);
}