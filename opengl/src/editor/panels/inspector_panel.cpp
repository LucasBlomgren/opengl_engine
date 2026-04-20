#include "pch.h"
#include "inspector_panel.h"
#include "editor/panel.h"
#include "imgui.h"
#include "game/game_object.h"
#include "graphics/mesh/mesh_manager.h"
#include "graphics/mesh/mesh.h"
#include "graphics/renderer/renderer.h"
#include "graphics/textures/texture_manager.h"

bool DrawSection(const char* title, ImVec2 innerSize = ImVec2(0, 0))
{
    ImGui::Spacing();

    ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.20f, 0.22f, 0.26f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.26f, 0.28f, 0.33f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImVec4(0.18f, 0.20f, 0.24f, 1.0f));

    ImGuiTreeNodeFlags flags =
        ImGuiTreeNodeFlags_Framed |
        ImGuiTreeNodeFlags_SpanAvailWidth |
        ImGuiTreeNodeFlags_NoTreePushOnOpen |
        ImGuiTreeNodeFlags_DefaultOpen;

    bool open = ImGui::TreeNodeEx(title, flags);

    ImGui::PopStyleColor(3);

    if (!open)
        return false;

    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10, 10));

    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.07f, 0.08f, 0.10f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.18f, 0.20f, 0.24f, 1.0f));

    ImGuiChildFlags child_flags = ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_Borders;
    std::string childId = std::string("##child_") + title;
    ImGui::BeginChild(childId.c_str(), ImVec2(0, 0), child_flags);

    return true;
}

void EndSection()
{
    ImGui::EndChild();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(3);
}

void Editor::InspectorPanel::OnImGuiRender(const PanelContext& ctx)
{
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 5.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

    if (!ImGui::Begin("Inspector", nullptr))
    {
        ImGui::End();
        ImGui::PopStyleVar(2);
        return;
    }

    ImGui::Spacing();
    ImGui::Spacing();

    if (!ctx.objectIsSelected)
    {
        ImGui::TextDisabled("No object selected");
        ImGui::End();
        ImGui::PopStyleVar(2);
        return;
    }

    GameObject* obj = ctx.world->getGameObject(ctx.selectedObjectHandle);
    RigidBody* rb = ctx.world->getRigidBody(ctx.selectedObjectHandle);
    Transform* rootTransform = ctx.world->getTransform(obj->rootTransformHandle);

    if (!obj || !rb || !rootTransform)
    {
        ImGui::TextDisabled("Selected object is invalid");
        ImGui::End();
        ImGui::PopStyleVar(2);
        return;
    }

    ImGui::Text("ID: %d | Name: %s", obj->id, obj->name.c_str());
    ImGui::Spacing();

    const ImGuiTableFlags tblFlags =
        ImGuiTableFlags_SizingFixedFit;

    auto BeginPropertyTable = [&](const char* id) -> bool
        {
            if (!ImGui::BeginTable(id, 2, tblFlags))
                return false;

            ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthFixed, 90.0f);
            ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
            return true;
        };

    auto RowLabelOnly = [&](const char* label)
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(label);
            ImGui::TableSetColumnIndex(1);
        };

    auto RowText = [&](const char* label, const char* value)
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(label);

            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(value);
        };

    auto RowCheckbox = [&](const char* label, const char* id, bool& v) -> bool
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(label);

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(-FLT_MIN);
            return ImGui::Checkbox(id, &v);
        };

    auto RowDragFloat = [&](const char* label, const char* id, float& v,
        float speed, float vmin, float vmax) -> bool
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(label);

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(-FLT_MIN);

            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.16f, 0.17f, 0.20f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0.20f, 0.21f, 0.24f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_FrameBgActive, ImVec4(0.22f, 0.23f, 0.26f, 1.0f));

            bool changed = ImGui::DragFloat(id, &v, speed, vmin, vmax);

            ImGui::PopStyleColor(3);
            return changed;
        };

    auto RowDragFloat3 = [&](const char* label, const char* id, glm::vec3& v,
        float speed, float vmin, float vmax, int amountDec) -> bool
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(label);

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(-FLT_MIN);
            std::string fmt = "%." + std::to_string(amountDec) + "f";
            return ImGui::DragFloat3(id, &v.x, speed, vmin, vmax, fmt.c_str());
        };

    auto RowDragFloat3Colored = [&](const char* label,
        const char* idX,
        const char* idY,
        const char* idZ,
        glm::vec3& v,
        float speed,
        float vmin,
        float vmax,
        const char* fmt) -> bool
        {
            ImGui::TableNextRow(ImGuiTableRowFlags_None, ImGui::GetFrameHeight() + 8.0f);
            ImGui::TableSetColumnIndex(0);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(label);

            ImGui::TableSetColumnIndex(1);

            const float labelToFieldGap = 6.0f;   // mellan X och fältet
            const float axisGroupGap = 16.0f;  // mellan X-gruppen, Y-gruppen, Z-gruppen

            float fullWidth = ImGui::GetContentRegionAvail().x;
            float labelW = ImGui::CalcTextSize("X").x;

            float itemW =
                (fullWidth
                    - 3.0f * labelW
                    - 3.0f * labelToFieldGap
                    - 2.0f * axisGroupGap) / 3.0f;

            bool changed = false;

            ImGui::PushItemWidth(itemW);

            ImGui::TextColored(ImVec4(0.90f, 0.25f, 0.25f, 1.0f), "X");
            ImGui::SameLine(0.0f, labelToFieldGap);
            changed |= ImGui::DragFloat(idX, &v.x, speed, vmin, vmax, fmt);

            ImGui::SameLine(0.0f, axisGroupGap);
            ImGui::TextColored(ImVec4(0.25f, 0.85f, 0.35f, 1.0f), "Y");
            ImGui::SameLine(0.0f, labelToFieldGap);
            changed |= ImGui::DragFloat(idY, &v.y, speed, vmin, vmax, fmt);

            ImGui::SameLine(0.0f, axisGroupGap);
            ImGui::TextColored(ImVec4(0.25f, 0.55f, 0.95f, 1.0f), "Z");
            ImGui::SameLine(0.0f, labelToFieldGap);
            changed |= ImGui::DragFloat(idZ, &v.z, speed, vmin, vmax, fmt);

            ImGui::PopItemWidth();

            return changed;
        };

    auto RowColorEdit3 = [&](const char* label, const char* id, glm::vec3& color) -> bool
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(label);

            ImGui::TableSetColumnIndex(1);
            return ImGui::ColorEdit3(id, &color.x, ImGuiColorEditFlags_DisplayRGB);
        };

    auto RebuildRigidBodyAABB = [&]()
        {
            if (rb->colliderHandles.empty())
                return;

            Collider* mainCollider = ctx.world->getCollider(rb->colliderHandles[0]);
            if (!mainCollider)
                return;

            rb->aabb = mainCollider->getAABB();

            if (rb->isCompound())
            {
                for (size_t i = 1; i < rb->colliderHandles.size(); ++i)
                {
                    Collider* c = ctx.world->getCollider(rb->colliderHandles[i]);
                    if (!c) continue;

                    rb->aabb.growToInclude(c->getAABB().worldMin);
                    rb->aabb.growToInclude(c->getAABB().worldMax);
                }

                rb->aabb.worldCenter = (rb->aabb.worldMin + rb->aabb.worldMax) * 0.5f;
                rb->aabb.worldHalfExtents = (rb->aabb.worldMax - rb->aabb.worldMin) * 0.5f;
                rb->aabb.setSurfaceArea();
            }
        };

    auto SyncAllCollidersFromRoot = [&]()
        {
            for (ColliderHandle& handle : rb->colliderHandles)
            {
                Collider* collider = ctx.world->getCollider(handle);
                if (!collider) continue;

                Transform* localTransform = ctx.world->getTransform(collider->localTransformHandle);
                if (!localTransform) continue;

                collider->pose.combineIntoColliderPose(*rootTransform, *localTransform);
                collider->updateAABB(collider->pose);
                collider->updateCollider(collider->pose);
            }

            RebuildRigidBodyAABB();
        };

    auto SyncSingleSubPartCollider = [&](int subPartIndex)
        {
            if (subPartIndex < 0)
                return;
            if (subPartIndex >= (int)rb->colliderHandles.size())
                return;

            Collider* collider = ctx.world->getCollider(rb->colliderHandles[subPartIndex]);
            if (!collider)
                return;

            Transform* localTransform = ctx.world->getTransform(collider->localTransformHandle);
            if (!localTransform)
                return;

            collider->pose.combineIntoColliderPose(*rootTransform, *localTransform);
            collider->updateAABB(collider->pose);
            collider->updateCollider(collider->pose);

            RebuildRigidBodyAABB();
        };

    auto RebuildObjectRenderBatches = [&]()
        {
            ctx.renderer->removeObjectFromBatch(ctx.selectedObjectHandle);
            ctx.renderer->addObjectToBatch(ctx.selectedObjectHandle);
        };

    //----------------------------------------
    // 1. Transform
    //----------------------------------------
    if (DrawSection("Transform"))
    {
        if (BeginPropertyTable("TransformTable"))
        {
            // Position
            {
                glm::vec3& pos = rootTransform->position;
                if (RowDragFloat3Colored("Position",
                    "##posx", "##posy", "##posz",
                    pos, 0.1f, -1000.f, 1000.f, "%.2f"))
                {
                    rootTransform->updateCache();
                    SyncAllCollidersFromRoot();
                }
            }

            // Scale
            {
                glm::vec3& scale = rootTransform->scale;
                if (RowDragFloat3Colored("Scale",
                    "##scalex", "##scaley", "##scalez",
                    scale, 0.1f, 0.1f, 1000.f, "%.2f"))
                {
                    rootTransform->updateCache();
                    SyncAllCollidersFromRoot();
                }

                if (ImGui::IsItemDeactivatedAfterEdit())
                {
                    if (!rb->colliderHandles.empty())
                    {
                        Collider* collider = ctx.world->getCollider(rb->colliderHandles[0]);
                        if (collider)
                        {
                            rb->calculateInverseInertia(collider->type, *collider, *rootTransform);
                            rb->updateInertiaWorld(*rootTransform);
                        }
                    }
                }
            }

            // Rotation
            {
                glm::vec3 uiDeg = glm::degrees(glm::eulerAngles(rootTransform->orientation));
                glm::vec3 lastUiDeg = uiDeg;

                if (RowDragFloat3Colored("Rotation",
                    "##rotx", "##roty", "##rotz",
                    uiDeg, 0.5f, -720.f, 720.f, "%.2f"))
                {
                    glm::vec3 deltaRad = glm::radians(uiDeg - lastUiDeg);

                    glm::quat dq =
                        glm::angleAxis(deltaRad.x, glm::vec3(1, 0, 0)) *
                        glm::angleAxis(deltaRad.y, glm::vec3(0, 1, 0)) *
                        glm::angleAxis(deltaRad.z, glm::vec3(0, 0, 1));

                    rootTransform->orientation = glm::normalize(rootTransform->orientation * dq);
                    rootTransform->updateCache();
                    SyncAllCollidersFromRoot();
                }
            }

            ImGui::EndTable();
        }

        EndSection();
    }

    //----------------------------------------
    // 2. Physics
    //----------------------------------------
    float physicsHeight = 150.0f;
    if (rb->type == BodyType::Dynamic)
        physicsHeight = 240.0f;
    else if (rb->type == BodyType::Kinematic)
        physicsHeight = 145.0f;

    if (DrawSection("Physics"))
    {
        if (BeginPropertyTable("PhysicsTable"))
        {
            // Body type
            {
                const char* bodyTypeItems[] = { "Dynamic", "Kinematic", "Static" };

                int currentType = 0;
                switch (rb->type)
                {
                case BodyType::Dynamic:   currentType = 0; break;
                case BodyType::Kinematic: currentType = 1; break;
                case BodyType::Static:    currentType = 2; break;
                }

                RowLabelOnly("Body Type");
                ImGui::SetNextItemWidth(-FLT_MIN);

                if (ImGui::Combo("##body_type", &currentType, bodyTypeItems, IM_ARRAYSIZE(bodyTypeItems)))
                {
                    BroadphaseBucket target = BroadphaseBucket::Awake;

                    switch (currentType)
                    {
                    case 0:
                        rb->type = BodyType::Dynamic;
                        if (rb->mass <= 0.0f) rb->mass = 1.0f;
                        rb->invMass = 1.0f / rb->mass;
                        rb->sleepCounterThreshold = 1.5f;

                        if (rb->allowSleep && rb->asleep)
                        {
                            target = BroadphaseBucket::Asleep;
                            rb->setAsleep(*rootTransform);
                        }
                        else
                        {
                            target = BroadphaseBucket::Awake;
                            rb->setAwake();
                        }
                        break;

                    case 1:
                        rb->type = BodyType::Kinematic;
                        rb->invMass = 0.0f;
                        rb->linearVelocity = glm::vec3(0.0f);
                        rb->angularVelocity = glm::vec3(0.0f);
                        rb->setAwake();
                        target = BroadphaseBucket::Awake;
                        break;

                    case 2:
                        rb->type = BodyType::Static;
                        rb->mass = 0.0f;
                        rb->invMass = 0.0f;
                        rb->linearVelocity = glm::vec3(0.0f);
                        rb->angularVelocity = glm::vec3(0.0f);
                        target = BroadphaseBucket::Static;
                        break;
                    }

                    rootTransform->updateCache();

                    if (!rb->colliderHandles.empty())
                    {
                        Collider* collider = ctx.world->getCollider(rb->colliderHandles[0]);
                        if (collider)
                        {
                            rb->calculateInverseInertia(collider->type, *collider, *rootTransform);
                            rb->updateInertiaWorld(*rootTransform);
                        }
                    }

                    ctx.physicsEngine->queueMove(obj->rigidBodyHandle, target);
                }
            }

            if (rb->type != BodyType::Static)
            {
                glm::vec3 linVel = rb->linearVelocity;
                //if (RowDragFloat3Colored("Linear vel", "##lin_vel", linVel, 0.1f, -1000.f, 1000.f, 1))
                if (RowDragFloat3Colored("Linear vel",
                    "##lin_velx", "##lin_vely", "##lin_velz",
                    linVel, 0.1f, -1000.f, 1000.f, "%.2f"))
                {
                    rb->linearVelocity = linVel;
                }

                glm::mat3 rotationMatrix = glm::mat3_cast(rootTransform->orientation);
                glm::mat3 invRotationMatrix = glm::transpose(rotationMatrix);
                glm::vec3 localAngVel = invRotationMatrix * rb->angularVelocity;
                if (RowDragFloat3Colored("Angular vel",
                    "##ang_velx", "##ang_vely", "##ang_velz",
                    localAngVel, 0.1f, -1000.f, 1000.f, "%.2f"))
                {
                    // spara tillbaka i world space
                    rb->angularVelocity = rotationMatrix * localAngVel;
                }
            }

            if (rb->type == BodyType::Dynamic)
            {
                float mass = rb->mass;
                if (RowDragFloat("Mass", "##mass", mass, 1.0f, 0.1f, 1000000.0f))
                {
                    rb->mass = mass;
                    rb->invMass = 1.0f / mass;
                }

                bool allowSleep = rb->allowSleep;
                if (RowCheckbox("Allow sleep", "##allow_sleep", allowSleep))
                {
                    rb->allowSleep = allowSleep;
                    if (!rb->allowSleep)
                    {
                        BroadphaseBucket target = rb->asleep ? BroadphaseBucket::Asleep : BroadphaseBucket::Awake;
                        ctx.physicsEngine->queueMove(obj->rigidBodyHandle, target);
                    }
                }

                bool asleep = rb->asleep;
                ImGui::BeginDisabled(!rb->allowSleep);
                if (RowCheckbox("Asleep", "##asleep", asleep))
                {
                    BroadphaseBucket target = asleep ? BroadphaseBucket::Asleep : BroadphaseBucket::Awake;
                    if (rb->asleep) {
                        rb->setAsleep(*rootTransform);
                    } else {
                        rb->setAwake();
                    }
                    ctx.physicsEngine->queueMove(obj->rigidBodyHandle, target);
                }
                ImGui::EndDisabled();

                bool allowGravity = rb->allowGravity;
                if (RowCheckbox("Gravity", "##allow_gravity", allowGravity))
                {
                    rb->allowGravity = allowGravity;
                }

                if (ImGui::IsItemDeactivatedAfterEdit())
                {
                    if (!rb->colliderHandles.empty())
                    {
                        Collider* collider = ctx.world->getCollider(rb->colliderHandles[0]);
                        if (collider)
                        {
                            rb->calculateInverseInertia(collider->type, *collider, *rootTransform);
                            rb->updateInertiaWorld(*rootTransform);
                        }
                    }
                }
            }

            ImGui::EndTable();
        }

        EndSection();
    }

    //----------------------------------------
    // 3. SubParts
    //----------------------------------------
    if (DrawSection("Parts"))
    {
        int comboIndex = ctx.editorMain->selectedSubPartIndex + 1; // +1 because combo has "None" as first item, then actual parts start from index 1

        int comboItemCount = (int)obj->parts.size() + 1; // +1 for "None"
        if (comboIndex >= comboItemCount)
            comboIndex = 0;

        if (BeginPropertyTable("SubPartsTable"))
        {
            std::vector<std::string> subPartNames;
            subPartNames.reserve(obj->parts.size());
            for (const SubPart& part : obj->parts)
                subPartNames.push_back(part.name);

            std::vector<const char*> itemPtrs;
            std::string none = "None";
            itemPtrs.reserve(obj->parts.size() + 1);

            itemPtrs.push_back(none.c_str());
            for (const std::string& name : subPartNames)
                itemPtrs.push_back(name.c_str());

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted("Selected");

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(-FLT_MIN);

            if (itemPtrs.empty())
            {
                ImGui::TextDisabled("No subparts");
            }
            else
            {
                if (ImGui::Combo("##selected_subpart", &comboIndex, itemPtrs.data(), (int)itemPtrs.size()))
                {
                    ctx.editorMain->selectedSubPartIndex = comboIndex - 1;
                }
            }

            // keep EditorMain in sync even when combo didn't change this frame
            ctx.editorMain->selectedSubPartIndex = comboIndex - 1;

            ImGui::EndTable();
        }

        //-------------------------------------------------
        // If a subpart is selected, show its properties
        //-------------------------------------------------
        int subPartIndex = comboIndex - 1; // -1 = None, 0..N-1 = actual part index

        if (subPartIndex >= 0 && subPartIndex < (int)obj->parts.size())
        {
            SubPart& selectedPart = obj->parts[subPartIndex];
            Transform* localTransform = ctx.world->getTransform(selectedPart.localTransformHandle);
            Collider* collider = ctx.world->getCollider(selectedPart.colliderHandle);

            //----------------------------------------
            // 3.1 SubPart Transform (Local)
            //----------------------------------------
            if (BeginPropertyTable("SubPartLocalTransformTable"))
            {
                glm::vec3 localPos = localTransform->position;
                if (RowDragFloat3Colored("Position",
                    "##sub_local_posx", "##sub_local_posy", "##sub_local_posz",
                    localPos, 0.1f, -1000.f, 1000.f, "%.2f"))
                {
                    localTransform->position = localPos;
                    localTransform->updateCache();
                    SyncSingleSubPartCollider(subPartIndex);
                }

                glm::vec3 localScale = localTransform->scale;
                if (RowDragFloat3Colored("Scale",
                    "##sub_local_scalex", "##sub_local_scaley", "##sub_local_scalez",
                    localScale, 0.1f, 0.01f, 1000.f, "%.2f"))
                {
                    localTransform->scale = localScale;
                    localTransform->updateCache();
                    SyncSingleSubPartCollider(subPartIndex);
                }

                glm::vec3 uiDeg = glm::degrees(glm::eulerAngles(localTransform->orientation));
                glm::vec3 lastUiDeg = uiDeg;

                if (RowDragFloat3Colored("Rotation",
                    "##sub_local_rotx", "##sub_local_roty", "##sub_local_rotz",
                    uiDeg, 0.5f, -720.f, 720.f, "%.2f"))
                {
                    glm::vec3 deltaRad = glm::radians(uiDeg - lastUiDeg);

                    glm::quat dq =
                        glm::angleAxis(deltaRad.x, glm::vec3(1, 0, 0)) *
                        glm::angleAxis(deltaRad.y, glm::vec3(0, 1, 0)) *
                        glm::angleAxis(deltaRad.z, glm::vec3(0, 0, 1));

                    localTransform->orientation = glm::normalize(localTransform->orientation * dq);
                    localTransform->updateCache();
                    SyncSingleSubPartCollider(subPartIndex);
                }

                ImGui::EndTable();
            }

            //----------------------------------------
            // 3.2 SubPart Collider
            //----------------------------------------
            if (BeginPropertyTable("SubPartColliderTable"))
            {
                // Collider type
                {
                    int colliderChoice = 0;
                    const char* colliderItems[] = { "Cuboid", "Sphere" };

                    if (collider->type == ColliderType::CUBOID)        colliderChoice = 0;
                    else if (collider->type == ColliderType::SPHERE)   colliderChoice = 1;

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextUnformatted("Collider");

                    ImGui::TableSetColumnIndex(1);

                    float avail = ImGui::GetContentRegionAvail().x;
                    float btnW = ImGui::CalcTextSize("Load").x + ImGui::GetStyle().FramePadding.x * 2.0f;
                    float spacing = ImGui::GetStyle().ItemSpacing.x;

                    ImGui::SetNextItemWidth(avail - btnW - spacing);
                    if (ImGui::Combo("##sub_collider", &colliderChoice, colliderItems, IM_ARRAYSIZE(colliderItems)))
                    {
                        if (selectedPart.mesh)
                        {
                            std::vector<glm::vec3> verticesPositions;
                            for (const Vertex& vertex : selectedPart.mesh->vertices) {
                                verticesPositions.push_back(vertex.position);
                            }

                            if (colliderChoice == 0) {
                                OOBB box(verticesPositions, collider->pose);
                                collider->type = ColliderType::CUBOID;
                                collider->shape = box;
                            }
                            else if (colliderChoice == 1) {
                                Sphere sphere(collider->pose);
                                collider->type = ColliderType::SPHERE;
                                collider->shape = sphere;
                            }

                            collider->aabb.init(verticesPositions);
                            collider->updateAABB(collider->pose);
                            collider->updateCollider(collider->pose);
                            RebuildRigidBodyAABB();
                        }
                    }
                }
                ImGui::EndTable();
            }

            //----------------------------------------
            // 3.3 SubPart Render
            //----------------------------------------
            if (BeginPropertyTable("SubPartRenderTable"))
            {
                // Mesh
                {
                    int meshChoice = 0;
                    const char* meshItems[] = { "Cube", "Sphere", "Teapot", "Pylon", "Tank" };

                    if (selectedPart.mesh == ctx.meshManager->getMesh("cube"))          meshChoice = 0;
                    else if (selectedPart.mesh == ctx.meshManager->getMesh("sphere"))   meshChoice = 1;
                    else if (selectedPart.mesh == ctx.meshManager->getMesh("teapot"))   meshChoice = 2;
                    else if (selectedPart.mesh == ctx.meshManager->getMesh("pylon"))    meshChoice = 3;
                    else if (selectedPart.mesh == ctx.meshManager->getMesh("tank"))     meshChoice = 4;

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextUnformatted("Mesh");

                    ImGui::TableSetColumnIndex(1);

                    float avail = ImGui::GetContentRegionAvail().x;
                    float btnW = ImGui::CalcTextSize("Load").x + ImGui::GetStyle().FramePadding.x * 2.0f;
                    float spacing = ImGui::GetStyle().ItemSpacing.x;

                    ImGui::SetNextItemWidth(avail - btnW - spacing);
                    if (ImGui::Combo("##sub_mesh", &meshChoice, meshItems, IM_ARRAYSIZE(meshItems)))
                    {
                        Mesh* mesh = nullptr;
                        switch (meshChoice)
                        {
                        case 0: mesh = ctx.meshManager->getMesh("cube");   break;
                        case 1: mesh = ctx.meshManager->getMesh("sphere"); break;
                        case 2: mesh = ctx.meshManager->getMesh("teapot"); break;
                        case 3: mesh = ctx.meshManager->getMesh("pylon");  break;
                        case 4: mesh = ctx.meshManager->getMesh("tank");   break;
                        }

                        if (mesh)
                        {
                            selectedPart.mesh = mesh;
                            RebuildObjectRenderBatches();

                            std::vector<glm::vec3> verticesPositions;
                            for (const Vertex& vertex : mesh->vertices) {
                                verticesPositions.push_back(vertex.position);
                            }
                            
                            // rebuild collider shape based on new mesh vertices if collider exists
                            if (collider)
                            {
                                if (collider->type == ColliderType::CUBOID) {
                                    OOBB box(verticesPositions, collider->pose);
                                    collider->shape = box;
                                }
                                else if (collider->type == ColliderType::SPHERE) {
                                    Sphere sphere(collider->pose);
                                    collider->shape = sphere;
                                }
                            }

                            collider->aabb.init(verticesPositions);
                            collider->updateAABB(collider->pose);
                            collider->updateCollider(collider->pose);
                            RebuildRigidBodyAABB();
                        }
                    }
                }

                // Texture
                {
                    int textureChoice = 0;
                    const char* textureItems[] = { "Plain", "Crate", "UVmap", "Terrain" };

                    if (selectedPart.textureId == -1)                                               textureChoice = 0;
                    else if (selectedPart.textureId == ctx.textureManager->getTexture("crate"))     textureChoice = 1;
                    else if (selectedPart.textureId == ctx.textureManager->getTexture("uvmap"))     textureChoice = 2;
                    else if (selectedPart.textureId == ctx.textureManager->getTexture("terrain2"))  textureChoice = 3;

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextUnformatted("Texture");

                    ImGui::TableSetColumnIndex(1);

                    float avail = ImGui::GetContentRegionAvail().x;
                    float btnW = ImGui::CalcTextSize("Load").x + ImGui::GetStyle().FramePadding.x * 2.0f;
                    float spacing = ImGui::GetStyle().ItemSpacing.x;

                    ImGui::SetNextItemWidth(avail - btnW - spacing);
                    if (ImGui::Combo("##sub_texture", &textureChoice, textureItems, IM_ARRAYSIZE(textureItems)))
                    {
                        int texId = -1;
                        switch (textureChoice)
                        {
                        case 0: texId = 999; break;
                        case 1: texId = ctx.textureManager->getTexture("crate");    break;
                        case 2: texId = ctx.textureManager->getTexture("uvmap");    break;
                        case 3: texId = ctx.textureManager->getTexture("terrain2"); break;
                        }

                        if (texId != -1)
                        {
                            selectedPart.textureId = texId;
                            RebuildObjectRenderBatches();
                        }
                    }
                }

                // Tint / color
                {
                    glm::vec3 color = selectedPart.color;
                    if (RowColorEdit3("Color", "##sub_color", color))
                    {
                        selectedPart.color = color;
                        RebuildObjectRenderBatches();
                    }
                }

                // See-through
                {
                    bool seeThrough = selectedPart.seeThrough;
                    if (RowCheckbox("See through", "##sub_see_through", seeThrough))
                    {
                        selectedPart.seeThrough = seeThrough;
                        RebuildObjectRenderBatches();
                    }
                }

                ImGui::EndTable();
            }
        }

        EndSection();
    }

    ImGui::End();
    ImGui::PopStyleVar(2);
}