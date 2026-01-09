#include "pch.h"
#include "imgui_manager.h"

void ImGuiManager::addInputRouter(InputRouter& router) {
	router.add(this);
}

void ImGuiManager::handleInput(const InputFrame& in, const InputContext& ctx, Consumed& c, FrameWants& wants) {
	if (!c.mouse and io->WantCaptureMouse) {
		c.mouse = true;
		wants.cameraLook = false;
		wants.captureMouse = false;
	}
	if (!c.keyboard and io->WantCaptureKeyboard) {
		c.keyboard = true;
	}
}

void ImGuiManager::init(
	GLFWwindow* window, 
	EngineState& es, 
	SceneBuilder& sb, 
	MeshManager& mm, 
	Renderer& r, 
	TextureManager& tm,
	SkyboxManager& sm) 
{
	this->engineState = &es;
	this->sceneBuilder = &sb;
	this->meshManager = &mm;
	this->renderer = &r;
	this->textureManager = &tm;
	this->skyboxManager = &sm;

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	io = &ImGui::GetIO(); 
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
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}

//--------------------------------
//           Main UI
// -------------------------------
void ImGuiManager::mainUI(float deltaTime, FrameTimers& frameTimers, GpuTimers& gpuT, size_t amountObjects) {

	ImGui::DockSpaceOverViewport();

	settingsUI();

}

//--------------------------------
//          Settings UI
// -------------------------------
void ImGuiManager::settingsUI() {
	ImGui::Begin("Settings");

	ImGui::SeparatorText("Objects");
	ImGui::Spacing();
	bool showNormals = engineState->getShowNormals();
	if (ImGui::Checkbox("Normals", &showNormals)) {
		engineState->toggleShowNormals();
	}
	bool showAABB = engineState->getShowAABB();
	if (ImGui::Checkbox("Bounding boxes", &showAABB)) {
		engineState->toggleShowAABB();
	}
	bool showColliders = engineState->getShowColliders();
	if (ImGui::Checkbox("Colliders", &showColliders)) {
		engineState->toggleShowColliders();
	}
	bool showContactPoints = engineState->getShowContactPoints();
	if (ImGui::Checkbox("Contacts", &showContactPoints)) {
		engineState->toggleShowContactPoints();
	}

	ImGui::NewLine();
	ImGui::SeparatorText("BVH");
	ImGui::Spacing();
	bool getShowBVH_awake = engineState->getShowBVH_awake();
	if (ImGui::Checkbox("Awake", &getShowBVH_awake)) {
		engineState->toggleShowBVH_awake();
	}
	bool getShowBVH_asleep = engineState->getShowBVH_asleep();
	if (ImGui::Checkbox("Asleep", &getShowBVH_asleep)) {
		engineState->toggleShowBVH_asleep();
	}
	bool getShowBVH_static = engineState->getShowBVH_static();
	if (ImGui::Checkbox("Static", &getShowBVH_static)) {
		engineState->toggleShowBVH_static();
	}
	bool getShowBVH_terrain = engineState->getShowBVH_terrain();
	if (ImGui::Checkbox("Terrain", &getShowBVH_terrain)) {
		engineState->toggleShowBVH_terrain();
	}

	ImGui::NewLine();
	ImGui::SeparatorText("Scene");
	ImGui::Spacing();
	if (ImGui::Button("Toggle Skybox")) {
		skyboxManager->toggleTexture();
	}
	if (ImGui::Button("Toggle Day/Night")) {
		sceneBuilder->toggleDayNight();
	}
	if (ImGui::Button("Toggle Lightsources")) {
		sceneBuilder->toggleLightsState();
	}

	static int currentItem = 0;
	const char* items[] = { "Test", "Empty", "Main", "Terrain", "Tall structure", "Tumbler", "Castle", "Invisible container", "Pile shape"};

	ImGui::SetNextWindowSizeConstraints(ImVec2(0, 0), ImVec2(FLT_MAX, ImGui::GetTextLineHeightWithSpacing() * 20)); // ~20 rader
	ImGui::Combo("##quality", &currentItem, items, IM_ARRAYSIZE(items));
	ImGui::SameLine();
	if (ImGui::Button("Load")) {
		switch (currentItem) {
		case 0: sceneBuilder->createScene(6); break;
		case 1: sceneBuilder->createScene(7); break;
		case 2: sceneBuilder->createScene(0); break;
		case 3: sceneBuilder->createScene(1); break;
		case 4: sceneBuilder->createScene(2); break;
		case 5: sceneBuilder->createScene(3); break;
		case 6: sceneBuilder->createScene(4); break;
		case 7: sceneBuilder->createScene(5); break;
		case 8: sceneBuilder->createScene(8); break;
		default: break;
		}
	}

	ImGui::End();
}

//--------------------------------
//       Selected object UI
// -------------------------------
void ImGuiManager::selectedObjectUI(GameObject* objPtr)
{
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 5.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

	ImGui::Begin("Inspector", nullptr);

	ImGui::Spacing();
	ImGui::Spacing();

	if (!objPtr) {
		ImGui::TextDisabled("No object selected");
		ImGui::End();
		ImGui::PopStyleVar(2);
		return;
	}

	GameObject& obj = *objPtr;

	ImGui::Text("Object ID: %d", obj.id);
	ImGui::Spacing();

	// --- Helpers for 2-col inspector layout ---
	const ImGuiTableFlags tblFlags =
		ImGuiTableFlags_SizingFixedFit |
		ImGuiTableFlags_BordersInnerV |
		ImGuiTableFlags_RowBg;

	auto BeginSection = [&](const char* title) {
		ImGui::Spacing();
		ImGui::SeparatorText(title);
		ImGui::Spacing();
		ImGui::BeginTable(title, 2, tblFlags);
		ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthFixed, 95.0f);
		ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
		};

	auto EndSection = [&]() {
		ImGui::EndTable();
		};

	auto RowText = [&](const char* label, const char* value) {
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted(label);

		ImGui::TableSetColumnIndex(1);
		ImGui::TextUnformatted(value);
		};

	auto RowCheckbox = [&](const char* label, const char* id, bool& v) -> bool {
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
			return ImGui::DragFloat(id, &v, speed, vmin, vmax);
		};

	auto RowDragFloat3 = [&](const char* label, const char* id, glm::vec3& v,
		float speed, float vmin, float vmax) -> bool
		{
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			ImGui::AlignTextToFramePadding();
			ImGui::TextUnformatted(label);

			ImGui::TableSetColumnIndex(1);
			ImGui::SetNextItemWidth(-FLT_MIN);
			return ImGui::DragFloat3(id, &v.x, speed, vmin, vmax);
		};

	// -----------------
	// Render section
	// -----------------
	BeginSection("Render");

	// Mesh
	{
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Mesh");

		ImGui::TableSetColumnIndex(1);
		ImGui::SetNextItemWidth(-FLT_MIN);
		static int currentItem = 0;
		const char* items[] = { "Cube","Sphere","Teapot","Pylon","Tank" };

		ImGui::TableSetColumnIndex(1);

		float avail = ImGui::GetContentRegionAvail().x;
		float btnW = ImGui::CalcTextSize("Load").x + ImGui::GetStyle().FramePadding.x * 2.0f;
		float spacing = ImGui::GetStyle().ItemSpacing.x;

		ImGui::SetNextItemWidth(avail - btnW - spacing);
		ImGui::Combo("##mesh", &currentItem, items, IM_ARRAYSIZE(items));

		ImGui::SameLine();
		if (ImGui::Button("Load##mesh"))
		{
			Mesh* mesh = nullptr;
			switch (currentItem) {
			case 0: mesh = meshManager->getMesh("cube"); break;
			case 1: mesh = meshManager->getMesh("sphere"); break;
			case 2: mesh = meshManager->getMesh("teapot"); break;
			case 3: mesh = meshManager->getMesh("pylon"); break;
			//case 4: mesh = meshManager->getMesh("girl"); break;
			case 4: mesh = meshManager->getMesh("tank"); break;
			}
			if (mesh) {
				obj.mesh = mesh;
				obj.initMesh();
				obj.initCollider();
				renderer->removeObjectFromBatch(&obj);
				renderer->addObjectToBatch(&obj);
			}
		}
	}

	// Texture
	{
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Texture");

		ImGui::TableSetColumnIndex(1);
		ImGui::SetNextItemWidth(-FLT_MIN);

		// Determine current item based on textureId
		static int currentItem = 0;
		if (obj.textureId == textureManager->getTexture("crate")) {
			currentItem = 1;
		}
		else if (obj.textureId == textureManager->getTexture("uvmap")) {
			currentItem = 2;
		}
		else if (obj.textureId == textureManager->getTexture("terrain2")) {
			currentItem = 3;
		}
		else {
			currentItem = 0; // Plain
		}

		// Options
		const char* items[] = { "Plain", "Crate", "UVmap", "Terrain"};

		ImGui::TableSetColumnIndex(1);

		float avail = ImGui::GetContentRegionAvail().x;
		float btnW = ImGui::CalcTextSize("Load").x + ImGui::GetStyle().FramePadding.x * 2.0f;
		float spacing = ImGui::GetStyle().ItemSpacing.x;

		ImGui::SetNextItemWidth(avail - btnW - spacing);
		ImGui::Combo("##texture", &currentItem, items, IM_ARRAYSIZE(items));

		ImGui::SameLine();
		if (ImGui::Button("Load##texture"))
		{
			int texId = -1;
			switch (currentItem) {
			case 0: texId = 999; break;
			case 1: texId = textureManager->getTexture("crate"); break;
			case 2: texId = textureManager->getTexture("uvmap"); break;
			case 3: texId = textureManager->getTexture("terrain2"); break;
			}
			if (texId != -1) {
				obj.textureId = texId;
				renderer->removeObjectFromBatch(&obj);
				renderer->addObjectToBatch(&obj);
			}
		}
	}

	// Color
	{
		glm::vec3 color = obj.color * 255.f;
		if (RowDragFloat3("Color", "##color", color, 1.0f, 0.f, 255.f)) {
			obj.color = color;
			obj.color /= 255.0f;
		}
	}

	EndSection();

	// -----------------
	// Transform section
	// -----------------
	BeginSection("Transform");

	// Position
	{
		glm::vec3 pos = obj.position;
		if (RowDragFloat3("Position", "##pos", pos, 0.1f, -1000.f, 1000.f)) {
			obj.position = pos;
			obj.modelMatrixDirty = true;
			obj.aabbDirty = true;
			obj.setModelMatrix();
			obj.updateAABB();
			obj.updateCollider();
		}
	}

	// Scale
	{
		glm::vec3 scale = obj.scale;
		if (RowDragFloat3("Scale", "##scale", scale, 0.1f, 0.5f, 1000.f)) {
			obj.scale = scale;
			obj.modelMatrixDirty = true;
		}
		if (ImGui::IsItemDeactivatedAfterEdit()) {
			obj.calculateInverseInertia();
		}
	}

	// Rotation (stored UI state)
	{
		glm::vec3 uiDeg = glm::degrees(glm::eulerAngles(obj.orientation));
		glm::vec3 lastUiDeg = uiDeg;

		if (RowDragFloat3("Rotation", "##rot", uiDeg, 0.5f, -720.f, 720.f)) {
			glm::vec3 deltaRad = glm::radians(uiDeg - lastUiDeg);

			glm::quat dq =
				glm::angleAxis(deltaRad.x, glm::vec3(1, 0, 0)) *
				glm::angleAxis(deltaRad.y, glm::vec3(0, 1, 0)) *
				glm::angleAxis(deltaRad.z, glm::vec3(0, 0, 1));

			obj.orientation = glm::normalize(obj.orientation * dq);
			obj.modelMatrixDirty = true;

			lastUiDeg = uiDeg;
		}
	}

	EndSection();

	// --------------
	// Physics section
	// --------------
	BeginSection("Physics");

	// Static checkbox
	{
		bool isStatic = obj.isStatic;
		if (RowCheckbox("Static", "##static", isStatic)) {
			obj.isStatic = isStatic;
			if (isStatic) {
				obj.mass = 0.0f;
				obj.invMass = 0.0f;
				obj.linearVelocity = glm::vec3(0.0f);
				obj.angularVelocity = glm::vec3(0.0f);
				obj.allowGravity = false;
				obj.allowSleep = false;
			}
			else {
				obj.mass = 1.0f;
				obj.invMass = 1.0f / obj.mass;
				obj.allowGravity = true;
				obj.linearVelocity = glm::vec3(0.0f);
				obj.angularVelocity = glm::vec3(0.0f);

				obj.allowSleep = true;
				obj.sleepCounterThreshold = 1.5f;
			}
		}
		if (ImGui::IsItemDeactivatedAfterEdit()) {
			obj.calculateInverseInertia();
		}
	}

	// Mass
	{
		float mass = obj.mass;
		if (RowDragFloat("Mass", "##mass", mass, 1.0f, 0.1f, 1000000.f)) {
			if (!obj.isStatic) {
				obj.mass = mass;
				obj.invMass = 1.0f / mass;
			}
		}
		if (ImGui::IsItemDeactivatedAfterEdit()) {
			obj.calculateInverseInertia();
		}
	}

	// Linear velocity
	{
		glm::vec3 vel = obj.linearVelocity;
		if (RowDragFloat3("Linear vel", "##linvel", vel, 0.1f, -1000.f, 1000.f)) {
			obj.linearVelocity = vel;
		}
	}

	// Angular velocity
	{
		glm::vec3 angVel = obj.angularVelocity;
		if (RowDragFloat3("Angular vel", "##angvel", angVel, 0.1f, -1000.f, 1000.f)) {
			obj.angularVelocity = angVel;
		}
	}

	EndSection();

	ImGui::End();
	ImGui::PopStyleVar(2);
}