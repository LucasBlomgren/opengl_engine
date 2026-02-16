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

	ImGui::SeparatorText("Objects");
	ImGui::Spacing();
	bool showCollisionNormals = ctx.engineState->getShowCollisionNormals();
	if (ImGui::Checkbox("Collision normals", &showCollisionNormals)) {
		ctx.engineState->toggleShowCollisionNormals();
	}
	bool showObjectLocalNormals = ctx.engineState->getShowObjectLocalNormals();
	if (ImGui::Checkbox("Local normals", &showObjectLocalNormals)) {
		ctx.engineState->toggleShowObjectLocalNormals();
	}
	bool showAABB = ctx.engineState->getShowAABB();
	if (ImGui::Checkbox("Bounding boxes", &showAABB)) {
		ctx.engineState->toggleShowAABB();
	}
	bool showColliders = ctx.engineState->getShowColliders();
	if (ImGui::Checkbox("Colliders", &showColliders)) {
		ctx.engineState->toggleShowColliders();
	}
	bool showContactPoints = ctx.engineState->getShowContactPoints();
	if (ImGui::Checkbox("Contacts", &showContactPoints)) {
		ctx.engineState->toggleShowContactPoints();
	}

	ImGui::NewLine();
	ImGui::SeparatorText("BVH");
	ImGui::Spacing();
	bool getShowBVH_awake = ctx.engineState->getShowBVH_awake();
	if (ImGui::Checkbox("Awake", &getShowBVH_awake)) {
		ctx.engineState->toggleShowBVH_awake();
	}
	bool getShowBVH_asleep = ctx.engineState->getShowBVH_asleep();
	if (ImGui::Checkbox("Asleep", &getShowBVH_asleep)) {
		ctx.engineState->toggleShowBVH_asleep();
	}
	bool getShowBVH_static = ctx.engineState->getShowBVH_static();
	if (ImGui::Checkbox("Static", &getShowBVH_static)) {
		ctx.engineState->toggleShowBVH_static();
	}
	bool getShowBVH_terrain = ctx.engineState->getShowBVH_terrain();
	if (ImGui::Checkbox("Terrain", &getShowBVH_terrain)) {
		ctx.engineState->toggleShowBVH_terrain();
	}

	ImGui::NewLine();
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
	const char* items[] = { "Test", "Empty", "Main", "Terrain", "Tall structure", "Tumbler", "Castle", "Invisible container", "Pile shape" };

	ImGui::SetNextWindowSizeConstraints(ImVec2(0, 0), ImVec2(FLT_MAX, ImGui::GetTextLineHeightWithSpacing() * 20)); // ~20 rader
	ImGui::Combo("##quality", &currentItem, items, IM_ARRAYSIZE(items));
	ImGui::SameLine();
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