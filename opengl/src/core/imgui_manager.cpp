#include "pch.h"
#include "imgui_manager.h"

void ImGuiManager::addInputRouter(InputRouter& router) {
	router.add(this);
}

void ImGuiManager::handleInput(const InputFrame& in, const InputContext& ctx, Consumed& c, FrameWants& wants) {
	//ImGuiIO* io = &ImGui::GetIO();
	//if (!c.mouse and io->WantCaptureMouse) {
	//	c.mouse = true;
	//	wants.cameraLook = false;
	//	wants.captureMouse = false;
	//}
	//if (!c.keyboard and io->WantCaptureKeyboard) {
	//	c.keyboard = true;
	//}
}

void ImGuiManager::init(GLFWwindow* window) {
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();

	ImGuiIO& io = ImGui::GetIO();
	(void)io;

	// init backends
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330");

	ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
}

void ImGuiManager::newFrame() {
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}

void ImGuiManager::render() {
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ImGuiManager::shutdown() {
	ImGui::DestroyPlatformWindows();
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}