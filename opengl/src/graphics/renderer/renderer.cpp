#include "pch.h"
#include "renderer.h"
#include "debug/draw_line.h"

template void Renderer::renderBVH(BVHTree<GameObject>&, glm::vec3&, glm::vec3&);
template void Renderer::renderBVH(BVHTree<Tri>&, glm::vec3& , glm::vec3&);

//-----------------------------
//           Init
//-----------------------------
void Renderer::init(
    unsigned int width, 
    unsigned int height, 
    EngineState& engineState, 
    LightManager& lightManager, 
    ShaderManager& shaderManager,
    ShadowManager& shadowManager, 
    SkyboxManager& skyboxManager) 
{
    screenWidth  = (float)width;
    screenHeight = (float)height;

    this->engineState   = &engineState;
    this->lightManager  = &lightManager;
    this->shadowManager = &shadowManager;
    this->skyboxManager = &skyboxManager;

    aabbRenderer.InitShared();
    sphereOutlineRenderer.init();

    defaultShader = shaderManager.getShader("default");
    debugShader   = shaderManager.getShader("debug");
    shadowShader  = shaderManager.getShader("shadow");
    skyboxShader  = shaderManager.getShader("skybox");

    defaultShader->use();

    this->VAO_line = setupLine();
    this->VAO_xyz = setup_xyzObject();
    this->VAO_contactPoint = setupContactPoint();
}

void Renderer::setViewPort(unsigned int w, unsigned int h) {
    glViewport(0, 0, w, h);
}

//-----------------------------
//       View Projection
//-----------------------------
void Renderer::setViewProjection(Camera& camera) {
    // projection matrix 
    glm::mat4 projection = glm::perspective(glm::radians(camera.zoom), screenWidth / screenHeight, 0.1f, maxViewDistance);
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
void Renderer::setDefaultRender(glm::mat4& lightSpaceMatrix) {
    setViewPort(screenWidth, screenHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

    defaultShader->use();
    defaultShader->setMat4("lightSpaceMatrix", lightSpaceMatrix);
    defaultShader->setInt("shadowMap", 1);

    Shader* defaultInstancedShader = defaultShader->instancedVariant;
    defaultInstancedShader->use();
    defaultInstancedShader->setMat4("lightSpaceMatrix", lightSpaceMatrix);
    defaultInstancedShader->setInt("shadowMap", 1);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, shadowManager->depthMap);
    glActiveTexture(GL_TEXTURE0);
}

//-----------------------------
//        Main render
//-----------------------------
void Renderer::render(
    Camera& camera,
    SceneBuilder& builder,
    PhysicsEngine& physics,
    GLuint qShadow[],
    GLuint qMain[],
    GLuint qDebug[],
    int writeIdx)
{
    if (builder.sceneDirty) {
        computeSceneBounds(builder);
    }

    createRenderQueue(builder.getDynamicObjects());
    buildBatchesFromRenderQueue();

    // shadow depth map render
    glBeginQuery(GL_TIME_ELAPSED, qShadow[writeIdx]);
    glm::mat4 lightSpaceMatrix = computeLightSpaceMatrix();
    setShadowRender(lightSpaceMatrix);
    renderGameObjectsShadow();
    cleanupShadowRender();
    glEndQuery(GL_TIME_ELAPSED);

    // default render
    glBeginQuery(GL_TIME_ELAPSED, qMain[writeIdx]);
    setDefaultRender(lightSpaceMatrix);
    setViewProjection(camera);
    uploadDirectionalLight();
    uploadLightsToShader();
    renderLights();
    renderScene(*defaultShader, builder);
    glEndQuery(GL_TIME_ELAPSED);

    renderRayCastHit(camera, builder);

    glDepthFunc(GL_LEQUAL);
    skyboxManager->render(*skyboxShader); 
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

    glEndQuery(GL_TIME_ELAPSED);
}

//-----------------------------
//      Create Render Queue
//-----------------------------
void Renderer::createRenderQueue(std::vector<GameObject>& dynamicObjects) {
    renderQueue.clear();
    renderQueue.reserve(dynamicObjects.size());

    for (GameObject& obj : dynamicObjects) {
        if (obj.seeThrough) {
            continue;
        }
        renderQueue.emplace_back(obj.shader, obj.mesh, obj.textureId, obj.modelMatrix, obj.color);
    }

    std::sort(renderQueue.begin(), renderQueue.end(),
        [](const RenderItem& a, const RenderItem& b) {
            if (a.shader < b.shader) return true;
            if (a.shader > b.shader) return false;

            if (a.mesh < b.mesh) return true;
            if (a.mesh > b.mesh) return false;

            return a.textureId < b.textureId;
        });
}

//-----------------------------
//   Build Batches from Queue
//-----------------------------
void Renderer::buildBatchesFromRenderQueue() {
    batches.clear();
    if (renderQueue.empty()) return;

    // start first batch
    Batch current;
    current.mesh = renderQueue[0].mesh;
    current.shader = renderQueue[0].shader;
    current.textureId = renderQueue[0].textureId;

    auto sameKey = [](const RenderItem& a, const RenderItem& b) {
        return a.mesh == b.mesh &&
            a.shader == b.shader &&
            a.textureId == b.textureId;
        };

    for (int i = 0; i < renderQueue.size(); ++i) {
        const auto& item = renderQueue[i];

        if (i > 0 && !sameKey(item, renderQueue[i - 1])) {
            batches.push_back(std::move(current)); // new combination -> move old batch

            // start new batch
            current = Batch{};
            current.mesh = item.mesh;
            current.shader = item.shader;
            current.textureId = item.textureId;
        }

        current.instances.emplace_back(item.modelMatrix, item.color);
    }

    // last batch
    batches.push_back(std::move(current));
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

        // without instancing
        if (instances.size() < INSTANCING_THRESHOLD) {
            shader = shadowShader;
            shader->use();
            for (const auto& inst : instances) {
                shader->setMat4("model", inst.model);
                mesh->draw();
            }
        }
        else {
            // with instancing
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
void Renderer::renderScene(Shader& shader, SceneBuilder& builder) {
    renderGameObjects(builder.getDynamicObjects());
    renderTerrain(shader, builder.getTerrainData(), builder.sceneDirty);
}

//---------------------------
//    Render game objects 
//---------------------------
void Renderer::renderGameObjects(std::vector<GameObject>& objects) {
    for (auto& batch : batches) {
        auto& instances = batch.instances;

        Shader* shader;
        Mesh* mesh = batch.mesh;
        GLuint texId = batch.textureId;

        glBindVertexArray(mesh->VAO);
        glBindTexture(GL_TEXTURE_2D, texId);

        // without instancing
        if (instances.size() < INSTANCING_THRESHOLD) {
            shader = batch.shader;
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
        // with instancing
        else {
            shader = batch.shader->instancedVariant;
            shader->use();

            if (texId != 999) {
                shader->setBool("useTexture", true);
            } else {
                shader->setBool("useTexture", false);
            }

            // fyll instanceVbo med alla instanser i batchen
            glBindBuffer(GL_ARRAY_BUFFER, mesh->instanceVBO);
            glBufferData(GL_ARRAY_BUFFER, instances.size() * sizeof(InstanceData), instances.data(), GL_DYNAMIC_DRAW);

            // gör ETT draw call för alla instanser
            glDrawElementsInstanced(GL_TRIANGLES, mesh->indexCount, GL_UNSIGNED_INT, 0, instances.size());
        }
    }
}

//-----------------------------
//       Render terrain
//-----------------------------
void Renderer::renderTerrain(Shader& shader, SceneBuilder::TerrainData& data, bool sceneDirty) {
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
    shader.use();
    shader.setMat4("model", model);

    shader.setBool("useTexture", true);
    shader.setBool("useRandomColor", false);
    shader.setVec3("uColor", glm::vec3(0, 1, 0));

    glBindTexture(GL_TEXTURE_2D, 4);
    glBindVertexArray(VAO);

    // --- Fyllning ---
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, nullptr);

    // --- Wireframe ---
    
    //debugShader->use();
    //debugShader->setMat4("model", model);
    //debugShader->setInt("debug.objectType", 0);
    //debugShader->setVec3("debug.uColor", glm::vec3{ 0.2 });
    // 
    //glDisable(GL_CULL_FACE); // ta bort cull face
    //glEnable(GL_POLYGON_OFFSET_LINE); 
    //// Skjuter ut linjerna en aning så att de inte z-fightar med fyllningen
    //glPolygonOffset(-1.0f, -1.0f); 
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); 
    //glLineWidth(3.f); 
    //glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, nullptr); 
    //glDisable(GL_POLYGON_OFFSET_LINE); 
    //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    //glEnable(GL_CULL_FACE); // återställ cull face
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
void Renderer::renderRayCastHit(Camera& camera, SceneBuilder& builder) {
    for (GameObject& obj : builder.getDynamicObjects()) {
        if (obj.isRaycastHit) {

            debugShader->use();
            debugShader->setBool("debug.useUniformColor", true);

            if (obj.colliderType == ColliderType::CUBOID) {
                OOBB& box = std::get<OOBB>(obj.collider.shape);
                obj.oobbRenderer.renderBox(*debugShader, box, obj.asleep, obj.isStatic, obj.isRaycastHit);
            }
            else if (obj.colliderType == ColliderType::SPHERE) {
                Sphere& sphere = std::get<Sphere>(obj.collider.shape);
                sphereOutlineRenderer.render(*debugShader, camera.position, sphere.wCenter, sphere.radius, obj.asleep, obj.isStatic, obj.isRaycastHit);
            }

            obj.isRaycastHit = false;
            break;
        }
    }
}

//----------------------------
//     Render debug info
//----------------------------
void Renderer::renderDebug(PhysicsEngine& physicsEngine, Camera& camera, std::vector<GameObject>& objects, unsigned int VAO_contactPoint, unsigned int VAO_xyz) {
    if (!(engineState->getShowAABB() || engineState->getShowColliders() || engineState->getShowNormals() || engineState->getShowContactPoints())) {
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
                obj.oobbRenderer.renderBox(*debugShader, box, obj.asleep, obj.isStatic, obj.isRaycastHit);
            }
            else if (obj.colliderType == ColliderType::SPHERE) {
                Sphere& sphere = std::get<Sphere>(obj.collider.shape);
                sphereOutlineRenderer.render(*debugShader, camera.position, sphere.wCenter, sphere.radius, obj.asleep, obj.isStatic, obj.isRaycastHit);
            }
        }
    }

    if (engineState->getShowNormals()) {
        glLineWidth(4.0f); 
        glBindVertexArray(objects[0].oobbRenderer.VAO_normals);
        debugShader->setInt("debug.objectType", 0);
        debugShader->setBool("debug.useUniformColor", false);

        for (GameObject& obj : objects) {
            obj.oobbRenderer.renderNormals(*debugShader, obj.modelMatrix);
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
        glEnable(GL_DEPTH_TEST); // återställ depth test
    }

    render_xyzObject(*debugShader, VAO_xyz);
}

//-------------------------
//       Render BVH
//-------------------------
template<typename E>
void Renderer::renderBVH(BVHTree<E>& tree, glm::vec3& nodeColor, glm::vec3& leafColor) {
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
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
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