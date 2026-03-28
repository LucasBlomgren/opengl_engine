#include "pch.h"
#include "settings_panel.h"
#include "editor/panel.h"
#include "imgui.h"
#include "imgui_manager.h"
#include "time.h"
#include "core/engine_state.h"
#include "scene_builder.h"
#include "world.h"
#include "skybox/skybox_manager.h"

void Editor::SettingsPanel::OnImGuiRender(const PanelContext& ctx)
{
    ImGui::Begin("Settings");

    // ---------------------------------
    //    OBJECT RENDERING SETTINGS
    // ---------------------------------
    ImGui::SeparatorText("Objects");
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

    // ---------------------------------
    //    BVH
    // ---------------------------------
    ImGui::Spacing();
    ImGui::SeparatorText("BVH");
    ImGui::Spacing();

    if (ImGui::BeginTable("BVHColumns", 2, ImGuiTableFlags_SizingStretchSame))
    {
        // -- LEFT COLUMN ----------------
        ImGui::TableNextColumn();

        bool showBVH_awake = ctx.engineState->getShowBVH_awake();
        if (ImGui::Checkbox("Awake", &showBVH_awake)) {
            ctx.engineState->toggleShowBVH_awake();
        }

        bool showBVH_asleep = ctx.engineState->getShowBVH_asleep();
        if (ImGui::Checkbox("Asleep", &showBVH_asleep)) {
            ctx.engineState->toggleShowBVH_asleep();
        }

        // -- RIGHT COLUMN ----------------
        ImGui::TableNextColumn();

        bool showBVH_static = ctx.engineState->getShowBVH_static();
        if (ImGui::Checkbox("Static", &showBVH_static)) {
            ctx.engineState->toggleShowBVH_static();
        }

        bool showBVH_terrain = ctx.engineState->getShowBVH_terrain();
        if (ImGui::Checkbox("Terrain", &showBVH_terrain)) {
            ctx.engineState->toggleShowBVH_terrain();
        }

        ImGui::EndTable();
    }

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
        "Test", "Empty", "Main", "Terrain",
        "Tall structure", "Tumbler", "Castle",
        "Invisible container", "Pile shape"
    };

    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();

    ImGui::Combo("", &currentItem, items, IM_ARRAYSIZE(items));

    if (ImGui::Button("Load")) {
        switch (currentItem) {
        case 0: ctx.sceneBuilder->createScene(6); break;
        case 1: ctx.sceneBuilder->createScene(7); break;
        case 2: ctx.sceneBuilder->createScene(0); break;
        case 3: ctx.sceneBuilder->createScene(1); break;
        case 4: ctx.sceneBuilder->createScene(2); break;
        case 5: ctx.sceneBuilder->createScene(3); break;
        case 6: ctx.sceneBuilder->createScene(4); break;
        case 7: ctx.sceneBuilder->createScene(5); break;
        case 8: ctx.sceneBuilder->createScene(8); break;
        default: break;
        }
    }

    ImGui::End();
}