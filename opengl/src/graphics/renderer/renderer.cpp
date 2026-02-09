#include "pch.h"
#include "renderer.h"
#include "debug/draw_line.h"
#include "mesh/mesh_manager.h"

template void Renderer::renderBVH(const BVHTree<GameObject>&, glm::vec3&, glm::vec3&);
template void Renderer::renderBVH(const BVHTree<Tri>&, glm::vec3& , glm::vec3&);

//-----------------------------
//           Init
//-----------------------------
void Renderer::init(
    unsigned int width,
    unsigned int height,
    Editor::EditorMain &editor,
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

    this->editor        = &editor;
    this->player        = &player;
    this->engineState   = &engineState;
    this->lightManager  = &lightManager;
    this->shadowManager = &shadowManager;
    this->skyboxManager = &skyboxManager;
    this->shaderManager = &shaderManager;
    this->meshManager   = &meshManager;

    aabbRenderer.initShared();
    oobbRenderer.initShared();
    sphereOutlineRenderer.init();
    this->arrowRenderer.mesh = meshManager.getMesh("debug_arrow");
    this->normalsRenderer.init();

    defaultShader = shaderManager.getShader("default");
    debugShader   = shaderManager.getShader("debug");
    shadowShader  = shaderManager.getShader("shadow");
    skyboxShader  = shaderManager.getShader("skybox");

    defaultShader->use();

    this->VAO_line = setupLine();
    this->VAO_xyz = setup_xyzObject();
    this->VAO_contactPoint = setupContactPoint();

    batches.reserve(100000);
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
void Renderer::addObjectToBatch(GameObject* obj) {
    // check existing buckets
    for (RenderBatch& bucket : batches) {
        if (bucket.mesh == obj->mesh &&
            bucket.shader == defaultShader &&
            bucket.textureId == obj->textureId)
        {
            bucket.objects.push_back(obj);
            bucket.instances.emplace_back(obj->modelMatrix, obj->color);

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
    newBatch.objects.push_back(obj);
    newBatch.instances.emplace_back(obj->modelMatrix, obj->color);

    // new batch & instance
    obj->batchIdx = static_cast<int>(batches.size());
    obj->batchInstanceIdx = 0;

    batches.push_back(std::move(newBatch));
}

//-----------------------------
//  Remove Object from Batch
//----------------------------- 
void Renderer::removeObjectFromBatch(GameObject* obj) {
    if (obj->batchIdx == -1) return; // not in a batch

    RenderBatch& batch = batches[obj->batchIdx];
    int lastIdx = static_cast<int>(batch.objects.size()) - 1;

    if (obj->batchInstanceIdx != lastIdx) {
        // swap with last object
        GameObject* lastObj = batch.objects[lastIdx];
        batch.objects[obj->batchInstanceIdx] = lastObj;
        batch.instances[obj->batchInstanceIdx] = batch.instances[lastIdx];
        lastObj->batchInstanceIdx = obj->batchInstanceIdx;
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

//-------------------------------------------
//     Debug Meshes
//-------------------------------------------
void Renderer::fillDebugMeshes(PhysicsEngine* physicsEngine, std::vector<GameObject>& objects) {
    debugMeshes.clear();

    // contact normals
    if (engineState->getShowCollisionNormals()) {
        for (Contact* c : physicsEngine->contactsToSolve)
        {
            glm::vec3 pos;
            GameObject* objA = c->objA_ptr;
            GameObject* objB = c->objB_ptr;

            if (objA == nullptr) {
                pos = objB->position;
            }
            else if (objB == nullptr) {
                pos = objA->position;
            }

            if (objA != nullptr && objB != nullptr) {

                // 1) dynamic vs dynamic: enklast (du kan senare byta till support-midpoint)
                if (!objA->isStatic && !objB->isStatic) {
                    pos = 0.5f * (objA->position + objB->position);
                }
                else {
                    // 2) hitta dynamiska objektet (det som INTE är static)
                    GameObject* dyn = objA->isStatic ? objB : objA;

                    // 3) välj d så att den pekar från dyn MOT den andra kroppen
                    // antag: c->normal pekar från A -> B
                    glm::vec3 d = (dyn == objB) ? (-c->normal) : (c->normal);   // om dyn är A: mot B = -n, om dyn är B: mot A = +n

                    // 4) flytta pos till dyn-AABB sidan som vetter mot den andra
                    pos = dyn->position;

                    float ax = std::abs(d.x), ay = std::abs(d.y), az = std::abs(d.z);

                    AABB* aabb = &dyn->collider.getAABB(); // se till att denna är world-space halfExtents

                    if (ax >= ay && ax >= az) {
                        pos.x += (d.x >= 0 ? aabb->halfExtents.x : -aabb->halfExtents.x);
                    }
                    else if (ay >= ax && ay >= az) {
                        pos.y += (d.y >= 0 ? aabb->halfExtents.y : -aabb->halfExtents.y);
                    }
                    else {
                        pos.z += (d.z >= 0 ? aabb->halfExtents.z : -aabb->halfExtents.z);
                    }

                    // liten offset så pilen syns ovanpå ytan
                    pos += c->normal * 0.01f;
                }
            }


            // push arrow mesh with model matrix oriented along the contact normal
            debugMeshes.push_back({
                arrowRenderer.mesh,
                arrowRenderer.getModelMatrix(pos, c->normal, glm::vec3(0.2f)),
                glm::vec3(1, 0, 1),
                false
                });
        }
    }

    // object local normals
    if (engineState->getShowObjectLocalNormals()) {
        for (GameObject& obj : objects) {
            Mesh* m = arrowRenderer.mesh;

            // bygg baseTR från obj.modelMatrix: samma position + rotation, men ingen skala
            glm::vec3 pos = glm::vec3(obj.modelMatrix[3]);

            glm::mat3 R = glm::mat3(obj.modelMatrix);
            R[0] = glm::normalize(R[0]);
            R[1] = glm::normalize(R[1]);
            R[2] = glm::normalize(R[2]);

            glm::mat4 baseTR(1.0f);
            baseTR[0] = glm::vec4(R[0], 0.0f);
            baseTR[1] = glm::vec4(R[1], 0.0f);
            baseTR[2] = glm::vec4(R[2], 0.0f);
            baseTR[3] = glm::vec4(pos, 1.0f);

            // valfri debug-skala (konstant i world)
            glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(0.1f));

            debugMeshes.push_back({ m, baseTR * normalsRenderer.modelX * S, glm::vec3(1,0,0), false });
            debugMeshes.push_back({ m, baseTR * normalsRenderer.modelY * S, glm::vec3(0,1,0), false });
            debugMeshes.push_back({ m, baseTR * normalsRenderer.modelZ * S, glm::vec3(0,0,1), false });
        }
    }

    // XYZ axes at world origin
    if (engineState->getShowObjectLocalNormals() or engineState->getShowCollisionNormals()) {
        Mesh* m = arrowRenderer.mesh;

        const float axisLength = 1.0f;
        const glm::vec3 offset = glm::vec3(-50.0f, 0.0f, -50.0f);
        const glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(axisLength));
        const glm::mat4 T = glm::translate(glm::mat4(1.0f), offset);
        const glm::mat4 TailToOrigin = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 5.77f, 0.0f));

        debugMeshes.push_back({ m, T * normalsRenderer.modelX * TailToOrigin * S, glm::vec3(1,0,0), false });
        debugMeshes.push_back({ m, T * normalsRenderer.modelY * TailToOrigin * S, glm::vec3(0,1,0), false });
        debugMeshes.push_back({ m, T * normalsRenderer.modelZ * TailToOrigin * S, glm::vec3(0,0,1), false });
    }
}
void Renderer::renderDebugMeshesDefault() {
    defaultShader->use();
    defaultShader->setBool("useTexture", false);

    for (const DebugMesh& dm : debugMeshes) {
        glBindVertexArray(dm.mesh->VAO);

        defaultShader->setVec3("uColor", dm.color);
        defaultShader->setMat4("model", dm.model);

        dm.mesh->draw();
    }
}
void Renderer::renderDebugMeshesShadow() {
    shadowShader->use();

    for (const DebugMesh& dm : debugMeshes) {
        if (!dm.castsShadow)
            continue;

        glBindVertexArray(dm.mesh->VAO);
        shadowShader->setMat4("model", dm.model);
        dm.mesh->draw();
    }
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
)
{
    // update scene bounds if dirty
    if (builder.sceneDirty) {
        computeSceneBounds(builder);
    }

    fillBatchInstances();
    fillDebugMeshes(&physics, builder.getDynamicObjects());

    // shadow depth map render
    glBeginQuery(GL_TIME_ELAPSED, qShadow[writeIdx]);
    glm::mat4 lightSpaceMatrix = computeLightSpaceMatrix();
    setShadowRender(lightSpaceMatrix);
    renderGameObjectsShadow();
    renderDebugMeshesShadow();
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
    DirectionalLight& light = lightManager->getDirectionalLight();
    //light.direction.y -= 0.0001f;
    quadRenderer.draw(shaderManager->getShader("SkyShader"), &camera, light.direction, targetW, targetH);

    renderLights();
    renderScene(builder);
    glEndQuery(GL_TIME_ELAPSED);

    // render raycast hit for player/editor
    if (engineState->isPlayerMode()) {
        renderRayCastHit(player->selectedObject, camera, builder);
    } else {
        renderRayCastHit(editor->selectedObject, camera, builder);
        renderRayCastHit(editor->hoveredObject, camera, builder);
    }

    glDepthFunc(GL_LEQUAL);
    //skyboxManager->render(*skyboxShader); 
    glDepthFunc(GL_LESS); 

    // debug
    glBeginQuery(GL_TIME_ELAPSED, qDebug[writeIdx]);

    renderDebug(physics, camera, builder.getDynamicObjects(), VAO_contactPoint, VAO_xyz);
    //renderFrustum(lightSpaceMatrix);

    if (engineState->getShowBVH_awake()) {
        static glm::vec3 c{ 0.80f, 0.40f, 0.00f }; // Dynamic awake nodes
        static glm::vec3 c2{ 1.00f, 0.65f, 0.20f }; // Dynamic awake leaves (ljusare orange)
        renderBVH(physics.getDynamicAwakeBvh(), c, c2);
    }
    if (engineState->getShowBVH_asleep()) {
        static glm::vec3 c{ 0.13f, 0.33f, 0.67f }; // Dynamic asleep nodes
        static glm::vec3 c2{ 0.45f, 0.70f, 1.00f }; // Dynamic asleep leaves (ljusare blå)
        renderBVH(physics.getDynamicAsleepBvh(), c,c2);
    }
    if (engineState->getShowBVH_static()) {
        static glm::vec3 c{ 0.10f, 0.60f, 0.27f }; // Static nodes
        static glm::vec3 c2{ 0.30f, 0.95f, 0.50f }; // Static leaves (ljusare grön)
        renderBVH(physics.getStaticBvh(), c, c2);
    }
    if (engineState->getShowBVH_terrain()) {
        static glm::vec3 c{ 0.40f, 0.31f, 0.13f }; // Terrain nodes
        static glm::vec3 c2{ 0.70f, 0.50f, 0.25f }; // Terrain leaves (ljusare brun/ockra)
        renderBVH(physics.getTerrainBvh(), c, c2);
    }

    glClear(GL_DEPTH_BUFFER_BIT);
    renderDebugMeshesDefault();
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
        for (GameObject* obj : bucket.objects) {
            bucket.instances.emplace_back(obj->modelMatrix, obj->color);
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
    renderGameObjects(builder.getDynamicObjects());
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
    std::vector<GameObject>& dData = builder.getDynamicObjects(); 

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

            const AABB& aabb = obj.aabb;
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
void Renderer::renderRayCastHit(GameObject* obj, Camera& camera, SceneBuilder& builder) {
    if (obj == nullptr) {
        return;
    }

    // #TODO: fix logic for selected vs hovered

    debugShader->use();
    debugShader->setBool("debug.useUniformColor", true);

    bool selected;
    if (obj->selectedByEditor or obj->selectedByPlayer) { selected = true; } 
    else if (obj->hoveredByEditor) { selected = false; } 

    if (obj->colliderType == ColliderType::CUBOID) {
        OOBB& box = std::get<OOBB>(obj->collider.shape);
        oobbRenderer.renderBox(*debugShader, box, obj->asleep, obj->isStatic, selected, obj->hoveredByEditor);
    }
    else if (obj->colliderType == ColliderType::SPHERE) {
        Sphere& sphere = std::get<Sphere>(obj->collider.shape);
        sphereOutlineRenderer.render(*debugShader, camera.position, sphere.wCenter, sphere.radius, obj->asleep, obj->isStatic, selected, obj->hoveredByEditor);
    }
}

//----------------------------
//     Render debug info
//----------------------------
void Renderer::renderDebug(PhysicsEngine& physicsEngine, Camera& camera, std::vector<GameObject>& objects, unsigned int VAO_contactPoint, unsigned int VAO_xyz) {

    // #TODO: fixa instancing för debug-rendering

    if (!(engineState->getShowAABB() || engineState->getShowColliders() || engineState->getShowContactPoints())) {
        return;
    }

    debugShader->use();
    debugShader->setBool("debug.useUniformColor", true); 
    debugShader->setInt("debug.objectType", 0); 

    if (engineState->getShowAABB()) {
        glLineWidth(2.0f); 
        glBindVertexArray(aabbRenderer.sVAO);  
        glm::vec3 color{ 0.9f, 0.7f, 0.2f };

        for (GameObject& obj : objects) {
            aabbRenderer.updateModel(obj.aabb, false);
            aabbRenderer.render(color, *debugShader);
        }
    }

    if (engineState->getShowColliders()) {
        glLineWidth(2.0f);
        for (GameObject& obj : objects) {
            debugShader->setBool("debug.useUniformColor", true);

            if (obj.colliderType == ColliderType::CUBOID) {
                OOBB& box = std::get<OOBB>(obj.collider.shape);
                oobbRenderer.renderBox(*debugShader, box, obj.asleep, obj.isStatic, false, false);
            }
            else if (obj.colliderType == ColliderType::SPHERE) {
                Sphere& sphere = std::get<Sphere>(obj.collider.shape);
                sphereOutlineRenderer.render(*debugShader, camera.position, sphere.wCenter, sphere.radius, obj.asleep, obj.isStatic, false, false);
            }
        }
    }

    if (engineState->getShowContactPoints()) {
        debugShader->setInt("debug.objectType", 2);
        debugShader->setBool("debug.useUniformColor", true);
        debugShader->setVec3("debug.uColor", glm::vec3(0, 250, 154));

        // skip depth test
        glDisable(GL_DEPTH_TEST);
        const auto& contactCache = physicsEngine.GetContactCache();
        for (const auto& pair : contactCache) {
            const Contact& contact = pair.second;

            for (int i = 0; i < contact.points.size(); i++) {
                if (!contact.points[i].wasUsedThisFrame) {
                    continue;
                }

                if (contact.points[i].wasWarmStarted) {
                    debugShader->setVec3("debug.uColor", glm::vec3(250,0,0)); // röd för warm-startade punkter
                } else {
                    debugShader->setVec3("debug.uColor", glm::vec3(0, 250, 154)); // grön för nya kontaktpunkter
                }

                renderContactPoint(*debugShader, VAO_contactPoint, contact.points[i].globalCoord);
            }
        }
        glEnable(GL_DEPTH_TEST);
    }
}

//-------------------------
//       Render BVH
//-------------------------
template<typename E>
void Renderer::renderBVH(const BVHTree<E>& tree, glm::vec3& nodeColor, glm::vec3& leafColor) {
    debugShader->use();
    debugShader->setBool("debug.useUniformColor", true);

    glLineWidth(2.0f); 
    glBindVertexArray(aabbRenderer.sVAO); 

    glm::vec3 color;
    AABB toDraw;

    for (auto& node : tree.nodes) {
        if (!node.alive) continue;

        if (node.isLeaf) {
            glLineWidth(4.0f);
            toDraw = node.tightBox;
            toDraw.centroid = (toDraw.wMin + toDraw.wMax) * 0.5f;
            toDraw.halfExtents = (toDraw.wMax - toDraw.wMin) * 0.5f;
            color = leafColor;
        }
        else {
            glLineWidth(2.0f);
            toDraw = node.fatBox;
            color = nodeColor;
        }

        aabbRenderer.updateModel(toDraw, /*asleep=*/false);
        aabbRenderer.render(color, *debugShader);
    }
}

//------------------------------
//     Render Shadow Frustum
//------------------------------
void Renderer::renderFrustum(const glm::mat4& viewProj)
{
    // 1) Invertera viewProj för att gå från clip → world
    glm::mat4 inv = glm::inverse(viewProj);

    // 2) Definiera hörn i clip-space
    glm::vec4 clip[8] = {
        {-1,-1,-1,1}, { 1,-1,-1,1}, { 1, 1,-1,1}, {-1, 1,-1,1},  // near
        {-1,-1, 1,1}, { 1,-1, 1,1}, { 1, 1, 1,1}, {-1, 1, 1,1}   // far
    };

    // 3) Transformera till world-space och dela med w
    glm::vec3 wc[8];
    for (int i = 0; i < 8; i++) {
        glm::vec4 t = inv * clip[i];
        wc[i] = glm::vec3(t) / t.w;
    }

    // 4) Platta ut 12 linje-segment (24 punkter)
    glm::vec3 lines[24] = {
        wc[0], wc[1], wc[1], wc[2], wc[2], wc[3], wc[3], wc[0], // near
        wc[4], wc[5], wc[5], wc[6], wc[6], wc[7], wc[7], wc[4], // far
        wc[0], wc[4], wc[1], wc[5], wc[2], wc[6], wc[3], wc[7]  // sidor
    };

    // 5) Skapa/VISA en VAO/VBO EN gång, uppdatera data och rita
    static GLuint vao = 0, vbo = 0;
    if (!vao) {
        glGenVertexArrays(1, &vao); glcount::incVAO();
        glGenBuffers(1, &vbo); glcount::incVBO();
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(lines), lines, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    }
    else {
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(lines), lines);
    }

    // 6) Rita
    debugShader->use();
    debugShader->setMat4("model", glm::mat4(1.0f));
    debugShader->setBool("debug.useUniformColor", true);
    debugShader->setVec3("debug.uColor", glm::vec3(1, 0, 0));
    glBindVertexArray(vao);
    glDrawArrays(GL_LINES, 0, 24);
    glBindVertexArray(0);
}