#include "pch.h"
#include "inspector_panel.h"
#include "editor/panel.h"
#include "imgui.h"
#include "imgui_manager.h"
#include "time.h"
#include "core/engine_state.h"
#include "game/game_object.h"
#include "graphics/mesh/mesh_manager.h"
#include "graphics/renderer/renderer.h"
#include "graphics/textures/texture_manager.h"


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

	if (!ctx.selectedObject) {
		ImGui::TextDisabled("No object selected");
		ImGui::End();
		ImGui::PopStyleVar(2);
		return;
	}

	GameObject& obj = *ctx.selectedObject;

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
			case 0: mesh = ctx.meshManager->getMesh("cube"); break;
			case 1: mesh = ctx.meshManager->getMesh("sphere"); break;
			case 2: mesh = ctx.meshManager->getMesh("teapot"); break;
			case 3: mesh = ctx.meshManager->getMesh("pylon"); break;
				//case 4: mesh = meshManager->getMesh("girl"); break;
			case 4: mesh = ctx.meshManager->getMesh("tank"); break;
			}
			if (mesh) {
				obj.mesh = mesh;
				obj.initMesh();
				obj.initCollider();
				ctx.renderer->removeObjectFromBatch(&obj);
				ctx.renderer->addObjectToBatch(&obj);
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


		// Options
		static int currentItem = 0;
		const char* items[] = { "Plain", "Crate", "UVmap", "Terrain" };

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
			case 1: texId = ctx.textureManager->getTexture("crate"); break;
			case 2: texId = ctx.textureManager->getTexture("uvmap"); break;
			case 3: texId = ctx.textureManager->getTexture("terrain2"); break;
			}
			if (texId != -1) {
				obj.textureId = texId;
				ctx.renderer->removeObjectFromBatch(&obj);
				ctx.renderer->addObjectToBatch(&obj);
			}
		}
	}

	// Color
	{
		glm::vec3 color = obj.color * 255.f;
		if (RowDragFloat3("Color", "##color", color, 1.0f, 0.f, 255.f, 0)) {
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
		if (RowDragFloat3("Position", "##pos", pos, 0.1f, -1000.f, 1000.f, 2)) {
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
		if (RowDragFloat3("Scale", "##scale", scale, 0.1f, 0.1f, 1000.f, 1)) {
			obj.scale = scale;
			obj.modelMatrixDirty = true;
			obj.aabbDirty = true;
			obj.setModelMatrix();
			obj.updateAABB();
			obj.updateCollider();
		}
		if (ImGui::IsItemDeactivatedAfterEdit()) {
			obj.calculateInverseInertia();
		}
	}

	// Rotation (stored UI state)
	{
		glm::vec3 uiDeg = glm::degrees(glm::eulerAngles(obj.orientation));
		glm::vec3 lastUiDeg = uiDeg;

		if (RowDragFloat3("Rotation", "##rot", uiDeg, 0.5f, -720.f, 720.f, 2)) {
			glm::vec3 deltaRad = glm::radians(uiDeg - lastUiDeg);

			glm::quat dq =
				glm::angleAxis(deltaRad.x, glm::vec3(1, 0, 0)) *
				glm::angleAxis(deltaRad.y, glm::vec3(0, 1, 0)) *
				glm::angleAxis(deltaRad.z, glm::vec3(0, 0, 1));

			obj.orientation = glm::normalize(obj.orientation * dq);
			obj.modelMatrixDirty = true;
			obj.aabbDirty = true;
			obj.setModelMatrix();
			obj.updateAABB();
			obj.updateCollider();

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
		if (RowDragFloat3("Linear vel", "##linvel", vel, 0.1f, -1000.f, 1000.f, 1)) {
			obj.linearVelocity = vel;
		}
	}

	// Angular velocity
	{
		glm::vec3 angVel = obj.angularVelocity;
		if (RowDragFloat3("Angular vel", "##angvel", angVel, 0.1f, -1000.f, 1000.f, 1)) {
			obj.angularVelocity = angVel;
		}
	}

	EndSection();

	ImGui::PopStyleVar(2);
	ImGui::End();
}