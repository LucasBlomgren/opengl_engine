
#include "renderer.h"

void Renderer::Init(unsigned int width, unsigned int height, EngineState& engineState, Shader& shader)
{   
    screenWidth = (float) width;
    screenHeight = (float) height;

    this->engineState = &engineState;

    this->shader = &shader;
    shader.use();
    shader.setInt("texture1", 0);
    shader.setInt("texture2", 1);
}

void Renderer::BeginFrame()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
}

void Renderer::SetViewProjection(Camera& camera)
{
    // projection matrix 
    glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), screenWidth / screenHeight, 0.1f, maxViewDistance);
    shader->setMat4("projection", projection);

    // camera/view transformation
    glm::mat4 view = camera.GetViewMatrix();
    shader->setMat4("view", view);
}

void Renderer::DrawGameObjects(std::vector<GameObject>& objects, unsigned int VAO_line)
{
    for (GameObject& obj : objects) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, obj.textureID);
        obj.drawMesh(*shader);

        if (engineState->GetShowAABB())
            obj.AABB.draw(*shader, obj.colliding);
        if (engineState->GetShowNormals())
            obj.OOBB.drawNormals(*shader, VAO_line, obj.position);
    }
}

void Renderer::DrawDebug(PhysicsEngine& physicsEngine, unsigned int VAO_contactPoint, unsigned int VAO_xyz, unsigned int VAO_worldFrame)
{
    if (engineState->GetShowContactPoints()) {
        shader->setBool("useUniformColor", true);
        shader->setVec3("uColor", glm::vec3(0, 250, 154));
        shader->setBool("isContactPoint", true);

        for (auto& pair : physicsEngine.contactCache) {
            Contact& contact = pair.second;
            for (int i = 0; i < contact.counter; i++)
                drawContactPoint(*shader, VAO_contactPoint, contact.points[i].globalCoord);
        }
        shader->setBool("isContactPoint", false);
    }

    draw_xyzObject(*shader, VAO_xyz);
    draw_worldFrame(*shader, VAO_worldFrame);
}