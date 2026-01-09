#include "pch.h"
#include "panels/performance_panel.h"

#include "imgui.h"
#include "panel_manager.h"
#include "scene_builder.h"

// Initialize panels
void Editor::PanelManager::init() {
	panels.push_back(std::make_shared<Editor::PerformancePanel>());
}

// Render all panels
void Editor::PanelManager::renderPanels() {
	auto& io = ImGui::GetIO();
	ctx.deltaTime = io.DeltaTime;
	ctx.fps = io.Framerate;
	ctx.amountObjects = sceneBuilder->getDynamicObjects().size();

	for (const auto& panel : panels) {
		ImGui::Begin(panel->GetName());
		panel->OnImGuiRender(ctx);
		ImGui::End();
	}
}
