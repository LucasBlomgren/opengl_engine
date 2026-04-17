#include "pch.h"
#include "debug_renderer.h"
#include "debug/render_contact_points.h"

template void DebugRenderer::renderBVH<BVHTree>(const BVHTree&, const glm::vec3&, const glm::vec3&);
template void DebugRenderer::renderBVH<TerrainBVH>(const TerrainBVH&, const glm::vec3&, const glm::vec3&);

//-----------------------------
//   Init
//-----------------------------
void DebugRenderer::init(const EngineState& engineState, const MeshManager& meshManager, const ShaderManager& shaderManager) {
    this->engineState = &engineState;
    litMeshShader = shaderManager.getShader("default");
    debugShapeShader = shaderManager.getShader("debug");
    shadowMeshShader = shaderManager.getShader("shadow");

    aabbRenderer.init();
    oobbRenderer.init();
    sphereOutlineRenderer.init();
    normalsRenderer.init();
    arrowRenderer.mesh = meshManager.getMesh("debug_arrow");

    litMeshShader->use();
    VAO_contactPoint = setupContactPoint();
}

//---------------------------------------------
//  Prepare Scene debug meshes
//---------------------------------------------
void DebugRenderer::prepareFrame(PhysicsEngine& physics, const std::vector<GameObject>& objects, World& world) {
    sceneDebugMeshes.clear();

    prepareObjectLocalNormals(objects, world);
    prepareCollisionNormals(physics, world);
    prepareXYZAxes();
}

// --------------------------------------------
//   Render Scene debug meshes to shadow map 
// --------------------------------------------
void DebugRenderer::renderShadowPass() const {
    shadowMeshShader->use();

    for (const SceneDebugMesh& dm : sceneDebugMeshes) {
        if (!dm.castsShadow)
            continue;

        glBindVertexArray(dm.mesh->VAO);
        shadowMeshShader->setMat4("model", dm.model);
        dm.mesh->draw();
    }
}

// ---------------------------------------------------------------------
//    Render Overlay Pass: (debug shapes + debug meshes with lighting)
// ---------------------------------------------------------------------
void DebugRenderer::renderOverlayPass(const PhysicsEngine& physics, const Camera& camera, const std::vector<GameObject>& objects, World& world) {
    // render debug shapes (AABBs, contact points etc) without lighting
    debugShapeShader->use();
    debugShapeShader->setBool("debug.useUniformColor", true);
    debugShapeShader->setInt("debug.objectType", 0);
    renderAABBs(objects, world);
    renderColliders(objects, camera, world);
    renderContactPoints(physics.GetContactCache());
    renderBVHs(physics);

    // render debug meshes with lighting
    litMeshShader->use();
    litMeshShader->setBool("useTexture", false);
    glClear(GL_DEPTH_BUFFER_BIT);

    for (const SceneDebugMesh& dm : sceneDebugMeshes) {
        glBindVertexArray(dm.mesh->VAO);
        litMeshShader->setVec3("uColor", dm.color);
        litMeshShader->setMat4("model", dm.model);
        dm.mesh->draw();
    }
}

//-----------------------------------------------------------------
// Prepare collisions normals, object local normals and XYZ axes
//-----------------------------------------------------------------
void DebugRenderer::prepareCollisionNormals(PhysicsEngine& physics, World& world) {
    if (!engineState->getShowCollisionNormals()) return;

    for (Contact* c : physics.contactsToSolve) {
        glm::vec3 pos(0.0f);

        if (!c->points.empty()) {
            for (const ContactPoint& cp : c->points) {
                pos += cp.globalCoord;
            }
            pos /= static_cast<float>(c->points.size());
        }
        else {
            // fallback om inga kontaktpunkter finns
            if (c->partnerTypeA == ContactPartnerType::RigidBody &&
                c->runtimeData.colliderA != nullptr) {
                if (c->runtimeData.colliderA->type == ColliderType::CUBOID) {
                    pos = std::get<OOBB>(c->runtimeData.colliderA->shape).worldCenter;
                }
                else {
                    pos = std::get<Sphere>(c->runtimeData.colliderA->shape).centerWorld;
                }
            }
            else if (c->partnerTypeB == ContactPartnerType::RigidBody &&
                c->runtimeData.colliderB != nullptr) {
                if (c->runtimeData.colliderB->type == ColliderType::CUBOID) {
                    pos = std::get<OOBB>(c->runtimeData.colliderB->shape).worldCenter;
                }
                else {
                    pos = std::get<Sphere>(c->runtimeData.colliderB->shape).centerWorld;
                }
            }
            else {
                continue;
            }
        }

        pos += c->normal * 0.01f;

        sceneDebugMeshes.push_back({
            arrowRenderer.mesh,
            arrowRenderer.getModelMatrix(pos, c->normal, glm::vec3(0.2f)),
            glm::vec3(1, 0, 1),
            false
            });
    }
}
void DebugRenderer::prepareObjectLocalNormals(const std::vector<GameObject>& objects, World& world) {
    if (!engineState->getShowObjectLocalNormals()) return;

    for (const GameObject& obj : objects) {
        Mesh* m = arrowRenderer.mesh;

        RigidBody* rb = world.getRigidBody(obj.rigidBodyHandle);
        Transform* t = world.getTransform(rb->rootTransformHandle);

        // bygg baseTR från obj.modelMatrix: samma position + rotation, men ingen skala
        glm::vec3 pos = t->position;
        glm::mat3 R = glm::mat3_cast(t->orientation);
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

        sceneDebugMeshes.push_back({ m, baseTR * normalsRenderer.modelX * S, glm::vec3(1,0,0), false });
        sceneDebugMeshes.push_back({ m, baseTR * normalsRenderer.modelY * S, glm::vec3(0,1,0), false });
        sceneDebugMeshes.push_back({ m, baseTR * normalsRenderer.modelZ * S, glm::vec3(0,0,1), false });
    }
}
void DebugRenderer::prepareXYZAxes() {
    if (!engineState->getShowObjectLocalNormals() and !engineState->getShowCollisionNormals()) return;

    Mesh* m = arrowRenderer.mesh;

    const float axisLength = 1.0f;
    const glm::vec3 offset = glm::vec3(-50.0f, 0.0f, -50.0f);
    const glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(axisLength));
    const glm::mat4 T = glm::translate(glm::mat4(1.0f), offset);
    const glm::mat4 TailToOrigin = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 5.77f, 0.0f));

    sceneDebugMeshes.push_back({ m, T * normalsRenderer.modelX * TailToOrigin * S, glm::vec3(1,0,0), false });
    sceneDebugMeshes.push_back({ m, T * normalsRenderer.modelY * TailToOrigin * S, glm::vec3(0,1,0), false });
    sceneDebugMeshes.push_back({ m, T * normalsRenderer.modelZ * TailToOrigin * S, glm::vec3(0,0,1), false });
}

// ----------------------------------------------
//  Render AABBs, Colliders & Contact Points
// ----------------------------------------------
void DebugRenderer::renderAABBs(const std::vector<GameObject>& objects, World& world) {
    if (!engineState->getShowAABB()) return;

    glLineWidth(2.0f);
    glBindVertexArray(aabbRenderer.sVAO);
    glm::vec3 color{ 0.9f, 0.7f, 0.2f };

    for (const GameObject& obj : objects) {
        RigidBody* rb = world.getRigidBody(obj.rigidBodyHandle);
        aabbRenderer.updateModel(rb->aabb, false);
        aabbRenderer.render(color, *debugShapeShader);
    }
}
void DebugRenderer::renderColliders(const std::vector<GameObject>& objects, const Camera& camera, World& world) {
    if (!engineState->getShowColliders()) return;

    glLineWidth(2.0f);
    debugShapeShader->setBool("debug.useUniformColor", true);

    for (const GameObject& obj : objects) {
        RigidBody* rb = world.getRigidBody(obj.rigidBodyHandle);
        bool isStatic = (rb->type == BodyType::Static);

        for (size_t i = 0; i < rb->colliderHandles.size(); ++i) {
            Collider* col = world.getCollider(rb->colliderHandles[i]);

            if (col->type == ColliderType::CUBOID) {
                const OOBB& box = std::get<OOBB>(col->shape);
                oobbRenderer.renderBox(
                    *debugShapeShader,
                    box,
                    rb->asleep,
                    isStatic,
                    obj.selectedByEditor || obj.selectedByPlayer,
                    obj.hoveredByEditor
                );
            }
            else if (col->type == ColliderType::SPHERE) {
                const Sphere& sphere = std::get<Sphere>(col->shape);
                sphereOutlineRenderer.render(
                    *debugShapeShader,
                    camera.position,
                    sphere.centerWorld,
                    sphere.radiusWorld,
                    rb->asleep,
                    isStatic,
                    obj.selectedByEditor || obj.selectedByPlayer,
                    obj.hoveredByEditor
                );
            }
        }
    }
}
void DebugRenderer::renderContactPoints(const std::unordered_map<size_t, Contact>& cache) const {
    if (!engineState->getShowContactPoints()) return;

    debugShapeShader->setInt("debug.objectType", 2);
    debugShapeShader->setBool("debug.useUniformColor", true);
    debugShapeShader->setVec3("debug.uColor", glm::vec3(0, 250, 154));

    // skip depth test
    glDisable(GL_DEPTH_TEST);
    for (const auto& pair : cache) {
        const Contact& contact = pair.second;

        for (int i = 0; i < contact.points.size(); i++) {
            if (!contact.points[i].wasUsedThisFrame) {
                continue;
            }

            if (contact.points[i].wasWarmStarted) {
                debugShapeShader->setVec3("debug.uColor", glm::vec3(250, 0, 0)); // röd för warm-startade punkter
            } else {
                debugShapeShader->setVec3("debug.uColor", glm::vec3(0, 250, 154)); // grön för nya kontaktpunkter
            }

            renderContactPoint(*debugShapeShader, VAO_contactPoint, contact.points[i].globalCoord);
        }
    }
    glEnable(GL_DEPTH_TEST);
}

//----------------------------------------
//    Render BVHs
//----------------------------------------
void DebugRenderer::renderBVHs(const PhysicsEngine& physics) {
    if (engineState->getShowBVH_awake()) {
        renderBVH(physics.getDynamicAwakeBvh(), bvhColors.awakeNode, bvhColors.awakeLeaf);
    }
    if (engineState->getShowBVH_asleep()) {
        renderBVH(physics.getDynamicAsleepBvh(), bvhColors.asleepNode, bvhColors.asleepLeaf);
    }
    if (engineState->getShowBVH_static()) {
        renderBVH(physics.getStaticBvh(), bvhColors.staticNode, bvhColors.staticLeaf);
    }
    if (engineState->getShowBVH_terrain()) {
        renderBVH(physics.getTerrainBvh(), bvhColors.terrainNode, bvhColors.terrainLeaf);
    }
}
template<class Tree> 
void DebugRenderer::renderBVH(const Tree& tree, const glm::vec3& nodeColor, const glm::vec3& leafColor) {
    debugShapeShader->use();
    debugShapeShader->setBool("debug.useUniformColor", true);

    glLineWidth(2.0f);
    glBindVertexArray(aabbRenderer.sVAO);

    glm::vec3 color;
    AABB toDraw;

    for (const auto& node : tree.nodes) {
        if (!node.alive) continue;

        if (node.isLeaf) {
            glLineWidth(4.0f);
            toDraw = node.tightBox;
            toDraw.worldCenter = (toDraw.worldMin + toDraw.worldMax) * 0.5f;
            toDraw.worldHalfExtents = (toDraw.worldMax - toDraw.worldMin) * 0.5f;
            color = leafColor;
        }
        else {
            glLineWidth(2.0f);
            toDraw = node.fatBox;
            color = nodeColor;
        }

        aabbRenderer.updateModel(toDraw, /*asleep=*/false);
        aabbRenderer.render(color, *debugShapeShader);
    }
}

//------------------------------
//     Render Shadow Frustum
//------------------------------
void DebugRenderer::renderFrustum(const glm::mat4& viewProj) const {
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
    debugShapeShader->use();
    debugShapeShader->setMat4("model", glm::mat4(1.0f));
    debugShapeShader->setBool("debug.useUniformColor", true);
    debugShapeShader->setVec3("debug.uColor", glm::vec3(1, 0, 0));
    glBindVertexArray(vao);
    glDrawArrays(GL_LINES, 0, 24);
    glBindVertexArray(0);
}