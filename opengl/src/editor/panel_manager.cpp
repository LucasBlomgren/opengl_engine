#include "pch.h"
#include "panels/performance_panel.h"
#include "panels/settings_panel.h"
#include "panels/inspector_panel.h"

#include "imgui.h"
#include "panel_manager.h"
#include "scene_builder.h"
#include "physics/physics.h" // <-- Fix for E0020: ObjAmountData definition

// Initialize panels
void Editor::PanelManager::init() {
	panels.push_back(std::make_shared<Editor::PerformancePanel>());
	panels.push_back(std::make_shared<Editor::SettingsPanel>());
	panels.push_back(std::make_shared<Editor::InspectorPanel>());
}

// Render all panels
void Editor::PanelManager::renderPanels() {
	auto& io = ImGui::GetIO();
	ctx.deltaTime = io.DeltaTime;
	ctx.fps = io.Framerate;
	ctx.amountObjects = sceneBuilder->getDynamicObjects().size();

	const ObjAmountData objAmounts = physicsEngine->getObjectAmounts();
	ctx.amountAwakeObjects = objAmounts.totalAwake;
	ctx.amountAsleepObjects = objAmounts.totalAsleep;
	ctx.amountStaticObjects = objAmounts.totalStatic;
	ctx.amountTerrainTris = objAmounts.totalTerrainTris;

	for (const auto& panel : panels) {
		panel->OnImGuiRender(ctx);
	}
}
