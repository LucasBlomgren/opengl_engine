#include "pch.h"
#include "renderer.h"
#include "debug/draw_line.h"
#include "mesh/mesh_manager.h"

//-----------------------------
//           Init
//-----------------------------
void Renderer::init(
    unsigned int width,
    unsigned int height,
    World& world,
    Editor::EditorMain& editor,
    Player& player,
    EngineState& engineState, 
    LightManager& lightManager, 
    ShaderManager& shaderManager,
    ShadowManager& shadowManager, 
    SkyboxManager& skyboxManager,
    MeshManager& meshManager) 
{
    screenWidth  = (float)width;
    screenHeight = (float)height;

    this->world         = &world;
    this->editor        = &editor;
    this->player        = &player;
    this->engineState   = &engineState;
    this->lightManager  = &lightManager;
    this->shadowManager = &shadowManager;
    this->skyboxManager = &skyboxManager;
    this->shaderManager = &shaderManager;
    this->meshManager   = &meshManager;

    defaultShader = shaderManager.getShader("default");
    debugShader   = shaderManager.getShader("debug");
    shadowShader  = shaderManager.getShader("shadow");
    skyboxShader  = shaderManager.getShader("skybox");

    defaultShader->use();
    batches.reserve(100000);

    debugRenderer.init(engineState, meshManager, shaderManager);
}

// set viewport
void Renderer::setViewPort(unsigned int w, unsigned int h) {
    glViewport(0, 0, w, h);
}

// clear render batches
void Renderer::clearRenderBatches() {
    batches.clear();
}

//-----------------------------
//    Add Object to Batch
//----------------------------- 
void Renderer::addObjectToBatch(GameObjectHandle handle) {
    GameObject* obj = world->getGameObject(handle);   

    // check existing buckets
    for (RenderBatch& bucket : batches) {
        if (bucket.mesh == obj->mesh &&
            bucket.shader == defaultShader &&
            bucket.textureId == obj->textureId)
        {
            bucket.objects.push_back(handle);
            bucket.instances.emplace_back(obj->transform.modelMatrix, obj->color);

            obj->batchIdx = static_cast<int>(&bucket - &batches[0]);
            obj->batchInstanceIdx = static_cast<int>(bucket.objects.size()) - 1;
            return;
        }
    }

    // no existing bucket found, create a new one
    RenderBatch newBatch;
    newBatch.mesh = obj->mesh;
    newBatch.shader = defaultShader;
    newBatch.textureId = obj->textureId;
    newBatch.objects.push_back(handle);
    newBatch.instances.emplace_back(obj->transform.modelMatrix, obj->color);

    // new batch & instance
    obj->batchIdx = static_cast<int>(batches.size());
    obj->batchInstanceIdx = 0;

    batches.push_back(std::move(newBatch));
}

//-----------------------------
//  Remove Object from Batch
//----------------------------- 
void Renderer::removeObjectFromBatch(GameObjectHandle handle) {
    GameObject* obj = world->getGameObject(handle);

    if (obj->batchIdx == -1) return; // not in a batch

    RenderBatch& batch = batches[obj->batchIdx];
    int lastIdx = static_cast<int>(batch.objects.size()) - 1;

    if (obj->batchInstanceIdx != lastIdx) {
        // swap with last object
        GameObjectHandle lastObjHandle = batch.objects[lastIdx];
        GameObject* lastObjPtr = world->getGameObject(lastObjHandle);

        batch.objects[obj->batchInstanceIdx] = lastObjHandle;
        batch.instances[obj->batchInstanceIdx] = batch.instances[lastIdx];
        lastObjPtr->batchInstanceIdx = obj->batchInstanceIdx;
    }

    batch.objects.pop_back();
    batch.instances.pop_back();
    obj->batchIdx = -1;
    obj->batchInstanceIdx = -1;
}

//-----------------------------
//       View Projection
//-----------------------------
void Renderer::setViewProjection(Camera& camera, float aspect) {
    glm::mat4 projection = glm::perspective(glm::radians(80.0f), aspect, 0.1f, maxViewDistance);
    // camera/view transformation
    glm::mat4 view = camera.GetViewMatrix();

    defaultShader->use();
    defaultShader->setMat4("projection", projection);
    defaultShader->setMat4("view", view);
    defaultShader->setVec3("viewPos", camera.position);

    Shader* defaultInstancedShader = defaultShader->instancedVariant;
    if (defaultInstancedShader) {
        defaultInstancedShader->use();
        defaultInstancedShader->setMat4("projection", projection);
        defaultInstancedShader->setMat4("view", view);
        defaultInstancedShader->setVec3("viewPos", camera.position);
    }

    debugShader->use();
    debugShader->setMat4("projection", projection);
    debugShader->setMat4("view", view);
    debugShader->setVec3("viewPos", camera.position);

    skyboxShader->use();
    skyboxShader->setMat4("projection", projection);
    skyboxShader->setMat4("view", glm::mat4(glm::mat3(view))); // remove translation from the view matrix
}

//-----------------------------
//      Set Shadow Render
//-----------------------------
void Renderer::setShadowRender(glm::mat4& lightSpaceMatrix) {
    setViewPort(shadowManager->width, shadowManager->height);
    shadowManager->bind();
    glClear(GL_DEPTH_BUFFER_BIT);
    glCullFace(GL_FRONT);

    shadowShader->use();
    shadowShader->setMat4("lightSpaceMatrix", lightSpaceMatrix);

    Shader* shadowInstancedShader = shadowShader->instancedVariant;
    if (shadowInstancedShader) {
        shadowInstancedShader->use();
        shadowInstancedShader->setMat4("lightSpaceMatrix", lightSpaceMatrix);
    }
}

//-----------------------------
//    Cleanup Shadow Render
//-----------------------------
void Renderer::cleanupShadowRender() {
    glCullFace(GL_BACK);
    shadowManager->unbind();
}

//-----------------------------
//      Set Default Render
//-----------------------------
void Renderer::setDefaultRender(glm::mat4& lightSpaceMatrix, int targetW, int targetH) {
    setViewPort(targetW, targetH);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

    defaultShader->use();
    defaultShader->setMat4("lightSpaceMatrix", lightSpaceMatrix);
    defaultShader->setInt("shadowMap", 1);

    Shader* inst = defaultShader->instancedVariant;
    inst->use();
    inst->setMat4("lightSpaceMatrix", lightSpaceMatrix);
    inst->setInt("shadowMap", 1);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, shadowManager->depthMap);
    glActiveTexture(GL_TEXTURE0);
}

//-------------------------------------
//        Main render
//-------------------------------------
void Renderer::render(
    Camera& camera,
    SceneBuilder& builder,
    PhysicsEngine& physics,
    GLuint qShadow[],
    GLuint qMain[],
    GLuint qDebug[],
    int writeIdx,
    const Editor::ViewportFBO* viewportFBO
) {
    // update scene bounds if dirty
    if (builder.sceneDirty) {
        computeSceneBounds(builder);
    }

    fillBatchInstances();
    debugRenderer.prepareFrame(physics, world->getGameObjectsMap().dense(), *world);

    // shadow depth map render
    glBeginQuery(GL_TIME_ELAPSED, qShadow[writeIdx]);
    glm::mat4 lightSpaceMatrix = computeLightSpaceMatrix();
    setShadowRender(lightSpaceMatrix);
    renderGameObjectsShadow();
    debugRenderer.renderShadowPass(); 
    renderTerrain(builder.getTerrainData(), builder.sceneDirty, true);
    cleanupShadowRender();
    glEndQuery(GL_TIME_ELAPSED);

    // bind viewport FBO if provided
    int targetW = (int)screenWidth;
    int targetH = (int)screenHeight;
    // viewport render
    if (viewportFBO) {
        viewportFBO->bind();
        targetW = viewportFBO->width;
        targetH = viewportFBO->height;
    } else {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    float aspect = (float)targetW / (float)targetH;

    // default render
    glBeginQuery(GL_TIME_ELAPSED, qMain[writeIdx]);
    setDefaultRender(lightSpaceMatrix, targetW, targetH);
    setViewProjection(camera, aspect);
    uploadDirectionalLight();
    uploadLightsToShader();

    // draw sky quad
    //DirectionalLight& light = lightManager->getDirectionalLight();
    //light.direction.y -= 0.0001f;
    //quadRenderer.draw(shaderManager->getShader("SkyShader"), &camera, light.direction, targetW, targetH);

    renderLights();
    renderScene(builder);
    glEndQuery(GL_TIME_ELAPSED);

    // render raycast hit for player/editor
    if (engineState->isPlayerMode()) {
        renderRayCastHit(player->selectedObjectHandle, camera, builder);
        renderRayCastHit(player->hoveredObjectHandle, camera, builder);
    } else {
        renderRayCastHit(editor->selectedObjectHandle, camera, builder);
        renderRayCastHit(editor->hoveredObjectHandle, camera, builder);
    }

    glDepthFunc(GL_LEQUAL);
    skyboxManager->render(*skyboxShader); 
    glDepthFunc(GL_LESS); 

    // debug
    glBeginQuery(GL_TIME_ELAPSED, qDebug[writeIdx]);
    debugRenderer.renderOverlayPass(physics, camera, world->getGameObjectsMap().dense(), *world);
    glEndQuery(GL_TIME_ELAPSED);

    if (viewportFBO) {
        viewportFBO->unbind(screenWidth, screenHeight);
    }
}

//-----------------------------
//   Fill Batch Instances
//-----------------------------
void Renderer::fillBatchInstances() {
    for (RenderBatch& bucket : batches) {
        bucket.instances.clear();
        for (GameObjectHandle& handle : bucket.objects) {
            GameObject* obj = world->getGameObject(handle);
            bucket.instances.emplace_back(obj->transform.modelMatrix, obj->color);
        }
    }
}

//-----------------------------
//       Render Shadows
//-----------------------------
void Renderer::renderGameObjectsShadow() {
    for (auto& batch : batches) {
        auto& instances = batch.instances;
        Shader* shader;
        Mesh* mesh = batch.mesh;
        glBindVertexArray(mesh->VAO);

        // no instancing
        if (instances.size() < INSTANCING_THRESHOLD) {
            shader = shadowShader;
            shader->use();
            for (const auto& inst : instances) {
                shader->setMat4("model", inst.model);
                mesh->draw();
            }
        }
        // instancing
        else {
            shader = shadowShader->instancedVariant;
            shader->use();

            glBindBuffer(GL_ARRAY_BUFFER, mesh->instanceVBO);
            glBufferData(GL_ARRAY_BUFFER, instances.size() * sizeof(InstanceData), instances.data(), GL_DYNAMIC_DRAW);
            glDrawElementsInstanced(GL_TRIANGLES, mesh->indexCount, GL_UNSIGNED_INT, 0, instances.size());
        }
    }
}

//-----------------------------
//       Render scene
//-----------------------------
void Renderer::renderScene(SceneBuilder& builder) {
    renderGameObjects(world->getGameObjectsMap().dense());
    renderTerrain(builder.getTerrainData(), builder.sceneDirty, false);
}

//---------------------------
//    Render game objects 
//---------------------------
void Renderer::renderGameObjects(std::vector<GameObject>& objects) {
    for (RenderBatch& bucket : batches) {
        Shader* shader;
        Mesh* mesh = bucket.mesh;
        GLuint texId = bucket.textureId;
        auto& instances = bucket.instances;

        glBindVertexArray(mesh->VAO);
        glBindTexture(GL_TEXTURE_2D, texId);

        // no instancing
        if (instances.size() < INSTANCING_THRESHOLD) {
            shader = bucket.shader;
            shader->use();
            for (const auto& inst : instances) {
                // Set texture or uniform color
                if (texId != 999) {
                    shader->setBool("useTexture", true);
                } else {
                    shader->setBool("useTexture", false);
                    shader->setVec3("uColor", inst.color);
                }

                // random color shader hack
                if (inst.color.x == -1 && inst.color.y == -1 && inst.color.z == -1) {
                    shader->setBool("useRandomColor", true);
                } else {
                    shader->setBool("useRandomColor", false);
                }

                shader->setMat4("model", inst.model);
                mesh->draw();
            }
        }
        // instancing
        else {
            shader = bucket.shader->instancedVariant;
            shader->use();

            if (texId != 999) {
                shader->setBool("useTexture", true);
            } else {
                shader->setBool("useTexture", false);
            }

            // fill instace buffer
            glBindBuffer(GL_ARRAY_BUFFER, mesh->instanceVBO);
            glBufferData(GL_ARRAY_BUFFER, instances.size() * sizeof(InstanceData), instances.data(), GL_DYNAMIC_DRAW);

            // draw instanced
            glDrawElementsInstanced(GL_TRIANGLES, mesh->indexCount, GL_UNSIGNED_INT, 0, instances.size());
        }
    }
}

//-----------------------------
//       Render terrain
//-----------------------------
void Renderer::renderTerrain(SceneBuilder::TerrainData& data, bool sceneDirty, bool shadowPass) {
    if (data.triangles.size() == 0) {
        return;
    }

    std::vector<Vertex>& vertices = data.vertices;
    std::vector<uint32_t>& indices = data.indices;

    static GLuint VAO = 0;
    static GLuint VBO = 0;
    static GLuint EBO = 0;

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
        glGenVertexArrays(1, &VAO); glcount::incVAO();
        glGenBuffers(1, &VBO); glcount::incVBO();

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);


        glGenBuffers(1, &EBO); glcount::incEBO();
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
    glBindVertexArray(VAO);

    constexpr bool renderTextured = true;
    constexpr bool renderWireframe = false;

    if (shadowPass) {
        shadowShader->use();
        shadowShader->setMat4("model", model);
    }

    if (renderTextured) {
        if (!shadowPass) {
            defaultShader->use();
            defaultShader->setMat4("model", model);
            defaultShader->setBool("useTexture", true);
            defaultShader->setBool("useRandomColor", false);
            defaultShader->setVec3("uColor", glm::vec3(0, 1, 0));
            glBindTexture(GL_TEXTURE_2D, 5);
        }

        glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_INT, nullptr);
    }
    if (renderWireframe) {
        if (!shadowPass) {
            debugShader->use();
            debugShader->setMat4("model", model);
            debugShader->setInt("debug.objectType", 0);
            debugShader->setVec3("debug.uColor", glm::vec3(0.3f));
            debugShader->setBool("debug.useUniformColor", true);

            glDisable(GL_CULL_FACE);
            glEnable(GL_POLYGON_OFFSET_LINE);
            glPolygonOffset(-1.0f, -1.0f);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glLineWidth(3.f);
        }

        glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_INT, nullptr);

        if (!shadowPass) {
            // restore
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisable(GL_POLYGON_OFFSET_LINE);
            glEnable(GL_CULL_FACE);
        }
    }
}

//-----------------------------------
//    Compute Light Space Matrix
//-----------------------------------
glm::mat4 Renderer::computeLightSpaceMatrix() {
    const DirectionalLight& light = lightManager->getDirectionalLight();

    glm::vec3 lightPos = sceneCenter - light.direction;
    glm::mat4 lightView = glm::lookAt(lightPos, sceneCenter, glm::vec3(0, 1, 0));

    glm::vec3 minLS(std::numeric_limits<float>::max());
    glm::vec3 maxLS(-std::numeric_limits<float>::max());
    for (auto& cw : sceneCorners) {
        glm::vec4 tmp = lightView * glm::vec4(cw, 1.0f);
        glm::vec3 ls = glm::vec3(tmp) / tmp.w; // om du vill, för orto w är 1
        minLS = glm::min(minLS, ls);
        maxLS = glm::max(maxLS, ls);
    }

    glm::mat4 lightProjection = glm::ortho(
         minLS.x,  maxLS.x,    // tight X‐intervall
         minLS.y,  maxLS.y,    // tight Y‐intervall
        -maxLS.z, -minLS.z     // Z‐intervall: notera att near/far i ljus‐space ofta behöver inverteras
    );

    return lightProjection * lightView;
}

//-----------------------------------
//    Compute Scene Bounds
//-----------------------------------
void Renderer::computeSceneBounds(SceneBuilder& builder) {
    sceneCorners.clear();

    SceneBuilder::TerrainData& tData = builder.getTerrainData();
    std::vector<GameObject>& dData = world->getGameObjectsMap().dense();

    //inf max variable
    float minY = std::numeric_limits<float>::max(); 
    float maxY = std::numeric_limits<float>::lowest();
    float minX = std::numeric_limits<float>::max(); 
    float maxX = std::numeric_limits<float>::lowest(); 
    float minZ = std::numeric_limits<float>::max(); 
    float maxZ = std::numeric_limits<float>::lowest(); 

    // beräkna min/max för terrängen
    if (tData.triangles.size() > 0) {
        for (const Tri& tri : tData.triangles) {
            for (const glm::vec3& v : tri.vertices) {
                minY = std::min(minY, v.y); 
                maxY = std::max(maxY, v.y); 
                minX = std::min(minX, v.x); 
                maxX = std::max(maxX, v.x); 
                minZ = std::min(minZ, v.z); 
                maxZ = std::max(maxZ, v.z); 
            }
        }
    }
    // beräkna min/max för dynamiska objekt
    else if (dData.size() > 0) {
        for (const GameObject& obj : dData) {
            if (!obj.isInsideShadowFrustum) {
                continue;
            }

            Collider* col = world->getCollider(obj.colliderHandle);
            const AABB& aabb = col->aabb;
            minY = std::min(minY, aabb.wMin.y); 
            maxY = std::max(maxY, aabb.wMax.y); 
            minX = std::min(minX, aabb.wMin.x); 
            maxX = std::max(maxX, aabb.wMax.x); 
            minZ = std::min(minZ, aabb.wMin.z); 
            maxZ = std::max(maxZ, aabb.wMax.z); 
        }
    }

    sceneMin = glm::vec3(minX, minY, minZ);
    sceneMax = glm::vec3(maxX, maxY, maxZ);
    sceneCenter = (sceneMin + sceneMax) * 0.5f; // mittpunkt av scenen 

    sceneCorners.emplace_back(sceneMin.x, sceneMin.y, sceneMin.z);
    sceneCorners.emplace_back(sceneMin.x, sceneMin.y, sceneMax.z);
    sceneCorners.emplace_back(sceneMin.x, sceneMax.y, sceneMin.z);  
    sceneCorners.emplace_back(sceneMin.x, sceneMax.y, sceneMax.z); 
    sceneCorners.emplace_back(sceneMax.x, sceneMin.y, sceneMin.z); 
    sceneCorners.emplace_back(sceneMax.x, sceneMin.y, sceneMax.z); 
    sceneCorners.emplace_back(sceneMax.x, sceneMax.y, sceneMin.z); 
    sceneCorners.emplace_back(sceneMax.x, sceneMax.y, sceneMax.z); 
}

//------------------------------
//     Upload Point Lights
//------------------------------
void Renderer::uploadLightsToShader() {

    defaultShader->use();
    const std::vector<Light>& lights = lightManager->getLights();

    for (int i = 0; i < lights.size(); ++i) {
        const Light& light = lights[i];
        std::string base = "pointLights[" + std::to_string(i) + "]";

        defaultShader->setVec3(base + ".position", light.position);
        defaultShader->setVec3(base + ".color", light.color);

        defaultShader->setVec3(base + ".ambient", light.ambient * light.intensity);
        defaultShader->setVec3(base + ".diffuse", light.diffuse * light.intensity);
        defaultShader->setVec3(base + ".specular", light.specular * light.intensity);

        defaultShader->setFloat(base + ".constant", light.constant);
        defaultShader->setFloat(base + ".linear", light.linear);
        defaultShader->setFloat(base + ".quadratic", light.quadratic);
    }
    defaultShader->setInt("numPointLights", static_cast<int>(lights.size()));

    // Instanced shader
    Shader* defaultInstancedShader = defaultShader->instancedVariant;
    defaultInstancedShader->use();

    for (int i = 0; i < lights.size(); ++i) {
        const Light& light = lights[i];
        std::string base = "pointLights[" + std::to_string(i) + "]";

        defaultInstancedShader->setVec3(base + ".position", light.position);
        defaultInstancedShader->setVec3(base + ".color", light.color);

        defaultInstancedShader->setVec3(base + ".ambient", light.ambient * light.intensity);
        defaultInstancedShader->setVec3(base + ".diffuse", light.diffuse * light.intensity);
        defaultInstancedShader->setVec3(base + ".specular", light.specular * light.intensity);

        defaultInstancedShader->setFloat(base + ".constant", light.constant);
        defaultInstancedShader->setFloat(base + ".linear", light.linear);
        defaultInstancedShader->setFloat(base + ".quadratic", light.quadratic);
    }
    defaultInstancedShader->setInt("numPointLights", static_cast<int>(lights.size()));

}

//---------------------------------
//     Upload Directional Light
//---------------------------------
void Renderer::uploadDirectionalLight() {
    const DirectionalLight& light = lightManager->getDirectionalLight();

    defaultShader->use();
    defaultShader->setVec3("dirLight.direction", light.direction);
    defaultShader->setVec3("dirLight.ambient", light.ambient);
    defaultShader->setVec3("dirLight.diffuse", light.diffuse);
    defaultShader->setVec3("dirLight.specular", light.specular);

    Shader* defaultInstancedShader = defaultShader->instancedVariant;
    defaultInstancedShader->use();
    defaultInstancedShader->setVec3("dirLight.direction", light.direction);
    defaultInstancedShader->setVec3("dirLight.ambient", light.ambient);
    defaultInstancedShader->setVec3("dirLight.diffuse", light.diffuse);
    defaultInstancedShader->setVec3("dirLight.specular", light.specular);
}

void Renderer::renderLights() const {
    const std::vector<Light>& lights = lightManager->getLights();
    for (const Light& light : lights) {
        light.render(*defaultShader);
    }
}

//----------------------------
//     Render Raycast Hit
//----------------------------
void Renderer::renderRayCastHit(GameObjectHandle& handle, Camera& camera, SceneBuilder& builder) {
    // #TODO: fix logic for selected vs hovered

    GameObject* obj = world->getGameObject(handle);
    if (obj == nullptr) {
        return;
    }

    Collider* col = world->getCollider(obj->colliderHandle);
    RigidBody* rb = world->getRigidBody(obj->rigidBodyHandle);

    debugShader->use();
    debugShader->setBool("debug.useUniformColor", true);

    bool selected;
    if (obj->selectedByEditor or obj->selectedByPlayer) { selected = true; }
    else if (obj->hoveredByEditor) { selected = false; }
    else return;

    bool isStatic = (rb->type == BodyType::Static);

    if (col->type == ColliderType::CUBOID) {
        const OOBB& box = std::get<OOBB>(col->shape);
        debugRenderer.oobbRenderer.renderBox(*debugShader, box, rb->asleep, isStatic, selected, obj->hoveredByEditor);
    }
    else if (col->type == ColliderType::SPHERE) {
        Sphere& sphere = std::get<Sphere>(col->shape);
        debugRenderer.sphereOutlineRenderer.render(*debugShader, camera.position, sphere.centerWorld, sphere.radiusWorld, rb->asleep, isStatic, selected, obj->hoveredByEditor);
    }
}