#include "pch.h"
#include "graphics/renderer/debug_renderer.h"
#include "graphics/debug/render_contact_points.h"
#include "game/world.h"

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
    arrowRenderer.mesh = meshManager.getMesh("debug_arrow");

    litMeshShader->use();
    VAO_contactPoint = setupContactPoint();
}

//---------------------------------------------
//  Prepare Scene debug meshes
//---------------------------------------------
void DebugRenderer::prepareFrame(physics::Engine& physics, const std::vector<GameObject>& objects, World& world, GameObjectHandle& selectedObjectHandle, int selectedSubPartIndex) {
    sceneDebugMeshes.clear();

    prepareObjectLocalNormals(
        physics,
        objects,
        world,
        selectedObjectHandle,
        selectedSubPartIndex
    );
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
void DebugRenderer::renderOverlayPass(const physics::Engine& physics, const Camera& camera, const std::vector<GameObject>& objects, World& world) {
    // render debug shapes (AABBs, contact points etc) without lighting
    debugShapeShader->use();
    debugShapeShader->setBool("debug.useUniformColor", true);
    debugShapeShader->setInt("debug.objectType", 0);
    renderAABBs(physics, objects);
    renderColliders(physics, objects, camera);
    renderContactPoints(
        physics.getDebugContacts(),
        physics.getDebugSpeculativeContacts()
    );
    renderBVHs(physics);
    //renderSweptAABBs(physics.debugSweeps, glm::vec3(1.0f, 1.0f, 0.0f));

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
void DebugRenderer::prepareCollisionNormals(physics::Engine& physics, World& world) {
    if (!engineState->getShowCollisionNormals()) return;

    for (const physics::debug::Contact& contact :
        physics.getDebugContacts()) {
        const glm::vec3 position =
            contact.representativePoint +
            contact.normal * 0.01f;

        sceneDebugMeshes.push_back({
            arrowRenderer.mesh,
            arrowRenderer.getModelMatrix(
                position,
                contact.normal,
                glm::vec3(0.2f)
            ),
            glm::vec3(1, 0, 1),
            false
            });
    }
}
void DebugRenderer::prepareObjectLocalNormals(
    physics::Engine& physics,
    const std::vector<GameObject>& objects,
    World& world,
    GameObjectHandle& selectedObjectHandle,
    int selectedSubPartIndex) 
{
    if (!selectedObjectHandle.isValid()) return;

    GameObject* selectedObject = world.getGameObject(selectedObjectHandle);
    if (!selectedObject) return;

    Transform* rootTransform =
        world.getTransform(selectedObject->rootTransformHandle);
    if (!rootTransform) return;

    Mesh* m = arrowRenderer.mesh;

    // by default, use root transform of object, to visualize world space normals
    glm::quat orientationToUse = rootTransform->orientation;
    glm::vec3 positionToUse = rootTransform->position;

    // subpart selected: use local transform + collider pose of subpart instead of root transform, to visualize local normals in the correct orientation/position
    if (selectedSubPartIndex >= 0 && selectedSubPartIndex < selectedObject->parts.size()) {
        const std::optional<physics::ColliderState> collider =
            physics.getColliderState(
                selectedObject
                    ->parts[selectedSubPartIndex]
                    .colliderHandle
            );

        if (collider) {
            orientationToUse = collider->worldPose.orientation;
            positionToUse = collider->worldPose.position;
        }
    }

    // build baseTR from obj.modelMatrix: same position + rotation, but no scale
    glm::vec3 pos = positionToUse;
    glm::mat3 R = glm::mat3_cast(orientationToUse);
    R[0] = glm::normalize(R[0]);
    R[1] = glm::normalize(R[1]);
    R[2] = glm::normalize(R[2]);

    glm::mat4 baseTR(1.0f);
    baseTR[0] = glm::vec4(R[0], 0.0f);
    baseTR[1] = glm::vec4(R[1], 0.0f);
    baseTR[2] = glm::vec4(R[2], 0.0f);
    baseTR[3] = glm::vec4(pos, 1.0f);

    // optional scale (constant in world space)
    glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(0.1f));

    // add a debug mesh for each local normal axis
    sceneDebugMeshes.push_back({ m, baseTR * normalsRenderer.modelX * S, glm::vec3(1,0,0), false });
    sceneDebugMeshes.push_back({ m, baseTR * normalsRenderer.modelY * S, glm::vec3(0,1,0), false });
    sceneDebugMeshes.push_back({ m, baseTR * normalsRenderer.modelZ * S, glm::vec3(0,0,1), false });
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
void DebugRenderer::renderAABBs(
    const physics::Engine& physics,
    const std::vector<GameObject>& objects) {
    if (!engineState->getShowAABB()) return;

    glLineWidth(AABB_LINE_WIDTH);
    glBindVertexArray(aabbRenderer.sVAO);
    glm::vec3 color{ 0.9f, 0.7f, 0.2f };

    for (const GameObject& obj : objects) {
        const std::optional<physics::BodyState> body =
            physics.getRigidBodyState(obj.rigidBodyHandle);
        if (!body) continue;

        aabbRenderer.updateModel(body->bounds, false);
        aabbRenderer.render(color, *debugShapeShader);
    }
}
void DebugRenderer::renderColliders(
    const physics::Engine& physics,
    const std::vector<GameObject>& objects,
    const Camera& camera) {
    if (!engineState->getShowColliders()) return;

    debugShapeShader->setBool("debug.useUniformColor", true);

    for (const GameObject& obj : objects) 
    {
        const std::optional<physics::BodyState> body =
            physics.getRigidBodyState(obj.rigidBodyHandle);
        if (!body) continue;

        for (physics::ColliderHandle colliderHandle : body->colliders) 
        {
            const std::optional<physics::ColliderState> collider =
                physics.getColliderState(colliderHandle);
            if (!collider) continue;

            glm::vec3 color;
            if (body->type == physics::BodyType::Static) {
                color = STATIC_COLOR;
            } else if (body->asleep) {
                color = ASLEEP_COLOR;
            } else {
                color = AWAKE_COLOR;
            }

            if (collider->type == physics::ColliderType::CUBOID) {
                glLineWidth(OOBB_LINE_WIDTH);
                oobbRenderer.renderBox(
                    *debugShapeShader,
                    collider->worldPose,
                    std::get<physics::BoxGeometry>(collider->shape),
                    color
                );
            }
            else if (collider->type == physics::ColliderType::SPHERE) {
                glLineWidth(SPHERE_LINE_WIDTH);
                const physics::SphereGeometry& sphere =
                    std::get<physics::SphereGeometry>(
                        collider->shape
                    );
                sphereOutlineRenderer.render(
                    *debugShapeShader,
                    camera.position,
                    sphere.worldCenter,
                    sphere.radius,
                    color
                );
            }
        }
    }
}
void DebugRenderer::renderContactPoints(
    const std::vector<physics::debug::Contact>& contacts,
    const std::vector<physics::debug::SpeculativeContact>& speculativeContacts) const
{
    if (!engineState->getShowContactPoints()) return;

    debugShapeShader->setInt("debug.objectType", 2);
    debugShapeShader->setBool("debug.useUniformColor", true);
    debugShapeShader->setVec3("debug.uColor", glm::vec3(0, 250, 154));

    // skip depth test
    glDisable(GL_DEPTH_TEST);
    for (const physics::debug::Contact& contact : contacts) {
        for (size_t index = 0;
            index < contact.pointCount;
            ++index) {
            if (contact.points[index].warmStarted) {
                debugShapeShader->setVec3("debug.uColor", glm::vec3(250, 0, 0)); // röd för warm-startade punkter
            } else {
                debugShapeShader->setVec3("debug.uColor", glm::vec3(0, 250, 154)); // grön för nya kontaktpunkter
            }

            renderContactPoint(
                *debugShapeShader,
                VAO_contactPoint,
                contact.points[index].worldPosition
            );
        }
    }
    
    for (const physics::debug::SpeculativeContact& specContact :
        speculativeContacts) {
        debugShapeShader->setVec3("debug.uColor", glm::vec3(0, 255, 0)); // grön för spekulativa kontaktpunkter
        renderContactPoint(
            *debugShapeShader,
            VAO_contactPoint,
            specContact.worldPosition
        );
    }
    glEnable(GL_DEPTH_TEST);
}

//----------------------------------------
//    Render Substep Islands
//----------------------------------------
void DebugRenderer::renderSweptAABBs(
    const std::vector<physics::AABB>& sweeps,
    const glm::vec3& color)
{
    debugShapeShader->use();
    debugShapeShader->setBool("debug.useUniformColor", true);
    glBindVertexArray(aabbRenderer.sVAO);

    for (const physics::AABB& sweptSrc : sweeps) {
        physics::AABB swept = sweptSrc;

        swept.worldCenter =
            (swept.worldMin + swept.worldMax) * 0.5f;

        swept.worldHalfExtents =
            (swept.worldMax - swept.worldMin) * 0.5f;

        aabbRenderer.updateModel(swept, false);
        aabbRenderer.render(
            color,
            *debugShapeShader
        );
    }
}

//----------------------------------------
//    Render BVHs
//----------------------------------------
void DebugRenderer::renderBVHs(const physics::Engine& physics) {
    if (engineState->getShowBVH_awake()) {
        renderBVH(
            physics.getDebugBvh(physics::debug::BvhType::Awake),
            BVH_COLORS.awakeNode,
            BVH_COLORS.awakeLeaf
        );
    }
    if (engineState->getShowBVH_asleep()) {
        renderBVH(
            physics.getDebugBvh(physics::debug::BvhType::Asleep),
            BVH_COLORS.asleepNode,
            BVH_COLORS.asleepLeaf
        );
    }
    if (engineState->getShowBVH_static()) {
        renderBVH(
            physics.getDebugBvh(physics::debug::BvhType::Static),
            BVH_COLORS.staticNode,
            BVH_COLORS.staticLeaf
        );
    }
    if (engineState->getShowBVH_terrain()) {
        renderBVH(
            physics.getTerrainDebugBvh(),
            BVH_COLORS.terrainLeaf,
            BVH_COLORS.terrainLeaf
        );
    }
}

void DebugRenderer::renderBVH(
    const physics::debug::Bvh& bvh,
    const glm::vec3& nodeColor,
    const glm::vec3& leafColor) {
    debugShapeShader->use();
    debugShapeShader->setBool("debug.useUniformColor", true);

    glBindVertexArray(aabbRenderer.sVAO);

    glm::vec3 color;

    for (const physics::debug::BvhNode& node : bvh.nodes) {
        if (node.isLeaf) {
            glLineWidth(BVH_LEAF_LINE_WIDTH);
            color = leafColor;
        }
        else {
            glLineWidth(BVH_NODE_LINE_WIDTH);
            color = nodeColor;
        }

        aabbRenderer.updateModel(node.bounds, false);
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
