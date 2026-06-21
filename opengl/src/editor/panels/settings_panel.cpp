#include "pch.h"
#include "settings_panel.h"
#include "editor/panel.h"
#include "imgui.h"
#include "engine/engine_state.h"
#include "scene_builder.h"
#include "skybox/skybox_manager.h"

void Editor::SettingsPanel::OnImGuiRender(const PanelContext& ctx)
{
    ImGui::Begin("Settings");

    // ---------------------------------
    //    PHYSICS SIMULATION SETTINGS
    // ---------------------------------
    ImGui::SeparatorText("Physics Simulation");
    ImGui::Spacing();

    ImGui::Text("Simulation speed");
    float simulationSpeed = ctx.engineState->getSimulationSpeed();
    if (ImGui::SliderFloat("##SimulationSpeed", &simulationSpeed, 0.01f, 2.0f, "%.2f")) {
        ctx.engineState->setSimulationSpeed(simulationSpeed);
    }

    ImGui::Text("Max substeps");
    int maxSubsteps = ctx.physicsEngine->maxSubsteps;
    if (ImGui::SliderInt("##MaxSubsteps", &maxSubsteps, 1, 16)) {
        ctx.physicsEngine->maxSubsteps = maxSubsteps;
    }

    ImGui::Text("PGS iterations");
    int pgsIterations = ctx.physicsEngine->pgsIterations;
    if (ImGui::SliderInt("##PGSIterations", &pgsIterations, 1, 64)) {
        ctx.physicsEngine->pgsIterations = pgsIterations;
    }



    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();

    // ---------------------------------
    //    DEBUG RENDERING SETTINGS
    // ---------------------------------
    ImGui::SeparatorText("Debug rendering");
    ImGui::Spacing();

    if (ImGui::BeginTable("SettingsColumns", 2, ImGuiTableFlags_SizingStretchSame))
    {
        // -- LEFT COLUMN ----------------
        ImGui::TableNextColumn();

        bool showAABB = ctx.engineState->getShowAABB();
        if (ImGui::Checkbox("Bounding boxes", &showAABB)) {
            ctx.engineState->toggleShowAABB();
        }

        bool showColliders = ctx.engineState->getShowColliders();
        if (ImGui::Checkbox("Colliders", &showColliders)) {
            ctx.engineState->toggleShowColliders();
        }

        bool wireframes = ctx.engineState->getShowWireframes();
        if (ImGui::Checkbox("Wireframes", &wireframes)) {
            ctx.engineState->toggleShowWireframes();
        }

        // -- RIGHT COLUMN ----------------
        ImGui::TableNextColumn();

        bool showCollisionNormals = ctx.engineState->getShowCollisionNormals();
        if (ImGui::Checkbox("Contact normals", &showCollisionNormals)) {
            ctx.engineState->toggleShowCollisionNormals();
        }

        bool showContactPoints = ctx.engineState->getShowContactPoints();
        if (ImGui::Checkbox("Contact points", &showContactPoints)) {
            ctx.engineState->toggleShowContactPoints();
        }

        bool showObjectLocalAxes = ctx.engineState->getShowObjectLocalNormals();
        if (ImGui::Checkbox("Local axes", &showObjectLocalAxes)) {
            ctx.engineState->toggleShowObjectLocalNormals();
        }

        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::Spacing();
    // ---------------------------------
    //    BVH
    // ---------------------------------
    if (ImGui::BeginTable("BVHColumns", 2, ImGuiTableFlags_SizingStretchSame))
    {
        // -- LEFT COLUMN ----------------
        ImGui::TableNextColumn();

        bool showBVH_awake = ctx.engineState->getShowBVH_awake();
        if (ImGui::Checkbox("Awake BVH", &showBVH_awake)) {
            ctx.engineState->toggleShowBVH_awake();
        }

        bool showBVH_asleep = ctx.engineState->getShowBVH_asleep();
        if (ImGui::Checkbox("Asleep BVH", &showBVH_asleep)) {
            ctx.engineState->toggleShowBVH_asleep();
        }

        // -- RIGHT COLUMN ----------------
        ImGui::TableNextColumn();

        bool showBVH_static = ctx.engineState->getShowBVH_static();
        if (ImGui::Checkbox("Static BVH", &showBVH_static)) {
            ctx.engineState->toggleShowBVH_static();
        }

        bool showBVH_terrain = ctx.engineState->getShowBVH_terrain();
        if (ImGui::Checkbox("Terrain BVH", &showBVH_terrain)) {
            ctx.engineState->toggleShowBVH_terrain();
        }

        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();

    // ---------------------------------
    //    SCENE SETTINGS
    // ---------------------------------
    ImGui::SeparatorText("Scene");
    ImGui::Spacing();

    if (ImGui::Button("Toggle Skybox")) {
        ctx.skyboxManager->toggleTexture();
    }

    if (ImGui::Button("Toggle Day/Night")) {
        ctx.sceneBuilder->toggleDayNight();
    }

    if (ImGui::Button("Toggle Lightsources")) {
        ctx.sceneBuilder->toggleLightsState();
    }

    static int currentItem = 0;
    const char* items[] = {
        "Test", "Terrain", "Tall structure", "Castle",
    };

    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();

    ImGui::Combo("##scene_select", &currentItem, items, IM_ARRAYSIZE(items));

    bool playerMode = ctx.engineState->isPlayerMode();
    if (ImGui::Button("Load")) {
        switch (currentItem) {
        case 0: ctx.sceneBuilder->createScene(0, playerMode, 0); break;
        case 1: ctx.sceneBuilder->createScene(1, playerMode, 0); break;
        case 2: ctx.sceneBuilder->createScene(2, playerMode, 0); break;
        case 3: ctx.sceneBuilder->createScene(3, playerMode, 0); break;
        //case 4: ctx.sceneBuilder->createScene(4, playerMode, 0); break;
        //case 5: ctx.sceneBuilder->createScene(5, playerMode, 0); break;
        //case 6: ctx.sceneBuilder->createScene(6, playerMode, 0); break;
        //case 7: ctx.sceneBuilder->createScene(7, playerMode, 0); break;
        //case 8: ctx.sceneBuilder->createScene(8, playerMode, 0); break;
        default: break;
        }
    }

    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();

    // ---------------------------------
    //    Shoot projectiles
    // ---------------------------------
    ImGui::SeparatorText("Editor shooting");
    ImGui::Spacing();

    bool shootContinuously = ctx.editorMain->shootContinuously;
    if (ImGui::Checkbox("Continuous", &shootContinuously)) {
        ctx.editorMain->shootContinuously = !ctx.editorMain->shootContinuously;
    }

    ImGui::Text("Cooldown");
    float shootCooldown = ctx.editorMain->shootCooldown;
    if (ImGui::SliderFloat("##Cooldown", &shootCooldown, 0.001f, 1.0f, "%.3f")) {
        ctx.editorMain->shootCooldown = shootCooldown;
    }

    ImGui::Text("Mass");
    float shootMass = ctx.editorMain->shootMass;
    if (ImGui::SliderFloat("##Mass", &shootMass, 1.0f, 1000.0f, "%.0f")) {
        ctx.editorMain->shootMass = shootMass;
    }

    ImGui::Text("Size");
    float shootSize = ctx.editorMain->shootSize;
    if (ImGui::SliderFloat("##Size", &shootSize, 1.0f, 100.0f, "%.0f")) {
        ctx.editorMain->shootSize = shootSize;
    }

    ImGui::Text("Velocity");
    float shootVelocity = ctx.editorMain->shootVelocity;
    if (ImGui::SliderFloat("##Velocity", &shootVelocity, 1.0f, 1000.0f, "%.0f")) {
        ctx.editorMain->shootVelocity = shootVelocity;
    }

    ImGui::End();
}