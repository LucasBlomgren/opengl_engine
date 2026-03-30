#include "pch.h"
#include "panels/performance_panel.h"
#include "panels/settings_panel.h"
#include "panels/inspector_panel.h"

#include "imgui.h"
#include "panel_manager.h"
#include "scene_builder.h"
#include "physics/physics.h"

// Initialize panels
void Editor::PanelManager::init() {
	panels.push_back(std::make_shared<Editor::PerformancePanel>());
	panels.push_back(std::make_shared<Editor::SettingsPanel>());
	panels.push_back(std::make_shared<Editor::InspectorPanel>());
}

// Render all panels
void Editor::PanelManager::renderPanels(float deltaTime) {
	auto& io = ImGui::GetIO();
	ctx.deltaTime = deltaTime;
	ctx.fps = io.Framerate;
	ctx.amountObjects = world->getGameObjectsMap().dense().size();

	const DebugData data = physicsEngine->getDebugData();
	ctx.amountAwakeObjects = data.awake;
	ctx.amountAsleepObjects = data.asleep;
	ctx.amountStaticObjects = data.Static;
	ctx.amountTerrainTris = data.terrainTris;
	ctx.amountCollisions = data.collisions;

	for (const auto& panel : panels) {
		panel->OnImGuiRender(ctx);
	}
}
