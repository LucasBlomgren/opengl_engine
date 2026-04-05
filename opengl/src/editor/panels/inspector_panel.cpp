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

	if (!ctx.objectIsSelected) {
		ImGui::TextDisabled("No object selected");
		ImGui::End();
		ImGui::PopStyleVar(2);
		return;
	}

	GameObject* obj = ctx.world->getGameObject(ctx.selectedObjectHandle);
	RigidBody* rb = ctx.world->getRigidBody(ctx.selectedObjectHandle);
	Collider* collider = ctx.world->getCollider(ctx.selectedObjectHandle);

	ImGui::Text("Object ID: %d", obj->id);
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
				obj->mesh = mesh;
				//obj->initMesh();
				//obj->initCollider();
				ctx.renderer->removeObjectFromBatch(ctx.selectedObjectHandle);
				ctx.renderer->addObjectToBatch(ctx.selectedObjectHandle);
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
				obj->textureId = texId;
				ctx.renderer->removeObjectFromBatch(ctx.selectedObjectHandle);
				ctx.renderer->addObjectToBatch(ctx.selectedObjectHandle);
			}
		}
	}

	// Color
	{
		glm::vec3 color = obj->color * 255.f;
		if (RowDragFloat3("Color", "##color", color, 1.0f, 0.f, 255.f, 0)) {
			obj->color = color;
			obj->color /= 255.0f;
		}
	}

	EndSection();

	// -----------------
	// Transform section
	// -----------------
	BeginSection("Transform");

	// Position
	{
		glm::vec3 pos = obj->transform.position;
		if (RowDragFloat3("Position", "##pos", pos, 0.1f, -1000.f, 1000.f, 2)) {
			obj->transform.position = pos;
			obj->transform.updateCache();
			collider->updateAABB(obj->transform);
			collider->updateCollider(obj->transform);
		}
	}

	// Scale
	{
		if (collider->type == ColliderType::SPHERE) {
			float scale = obj->transform.scale.x;
			if (RowDragFloat("Scale", "##scale", scale, 0.1f, 0.1f, 1000.f)) {
				obj->transform.scale = glm::vec3(scale);
				obj->transform.updateCache();
				collider->updateAABB(obj->transform);
				collider->updateCollider(obj->transform);
			}
			if (ImGui::IsItemDeactivatedAfterEdit()) {
				obj->transform.updateCache();
				rb->calculateInverseInertia(collider->type, *collider, obj->transform);
				rb->updateInertiaWorld(obj->transform);
				rb->invRadius = 1.0f / (0.5f * glm::length(obj->transform.scale));
			}
		} 
		else {
			glm::vec3 scale = obj->transform.scale;
			if (RowDragFloat3("Scale", "##scale", scale, 0.1f, 0.1f, 1000.f, 2)) {
				obj->transform.scale = scale;
				obj->transform.updateCache();
				collider->updateAABB(obj->transform);
				collider->updateCollider(obj->transform);
			}
			if (ImGui::IsItemDeactivatedAfterEdit()) {
				obj->transform.updateCache();
				rb->calculateInverseInertia(collider->type, *collider, obj->transform);
				rb->updateInertiaWorld(obj->transform);
				rb->invRadius = 1.0f / (0.5f * glm::length(obj->transform.scale));
			}
		}
	}

	// Rotation (stored UI state)
	{
		glm::vec3 uiDeg = glm::degrees(glm::eulerAngles(obj->transform.orientation));
		glm::vec3 lastUiDeg = uiDeg;

		if (RowDragFloat3("Rotation", "##rot", uiDeg, 0.5f, -720.f, 720.f, 2)) {
			glm::vec3 deltaRad = glm::radians(uiDeg - lastUiDeg);

			glm::quat dq =
				glm::angleAxis(deltaRad.x, glm::vec3(1, 0, 0)) *
				glm::angleAxis(deltaRad.y, glm::vec3(0, 1, 0)) *
				glm::angleAxis(deltaRad.z, glm::vec3(0, 0, 1));

			obj->transform.orientation = glm::normalize(obj->transform.orientation * dq);
			obj->transform.updateCache();
			collider->updateAABB(obj->transform);
			collider->updateCollider(obj->transform);

			lastUiDeg = uiDeg;
		}
	}

	EndSection();

	// --------------
	// Physics section
	// --------------
	BeginSection("Physics");

	// RigidBody type
	{
		const char* bodyTypeItems[] = { "Dynamic", "Kinematic", "Static" };

		int currentType = 0;
		switch (rb->type) {
		case BodyType::Dynamic:   currentType = 0; break;
		case BodyType::Kinematic: currentType = 1; break;
		case BodyType::Static:    currentType = 2; break;
		}

		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Body Type");

		ImGui::TableSetColumnIndex(1);
		ImGui::SetNextItemWidth(-FLT_MIN);

		if (ImGui::Combo("##bodyType", &currentType, bodyTypeItems, IM_ARRAYSIZE(bodyTypeItems))) {
			BroadphaseBucket target;

			switch (currentType) {
			case 0:
				rb->type = BodyType::Dynamic;
				if (rb->mass <= 0.0f) rb->mass = 1.0f;
				rb->invMass = 1.0f / rb->mass;
				rb->sleepCounterThreshold = 1.5f;

				if (rb->allowSleep and rb->asleep) {
					target = BroadphaseBucket::Asleep;
				}
				else {
					target = BroadphaseBucket::Awake;
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

			obj->transform.updateCache();
			rb->calculateInverseInertia(collider->type, *collider, obj->transform);
			rb->updateInertiaWorld(obj->transform);
			ctx.physicsEngine->queueMove(obj->rigidBodyHandle, target);
		}
	}

	// Dynamic body options
	if (rb->type == BodyType::Dynamic)
	{
		// AllowSleep checkbox
		{
			bool allowSleep = rb->allowSleep;
			if (RowCheckbox("Allow sleep", "##allowSleep", allowSleep)) {
				rb->allowSleep = allowSleep;

				if (!rb->allowSleep) {
					BroadphaseBucket target = BroadphaseBucket::Awake;
					ctx.physicsEngine->queueMove(obj->rigidBodyHandle, target);
				}
			}
		}

		// Asleep checkbox
		{
			bool asleep = rb->asleep;

			ImGui::BeginDisabled(!rb->allowSleep);
			if (RowCheckbox("Asleep", "##asleep", asleep)) {
				BroadphaseBucket target;

				rb->asleep = asleep;
				if (rb->allowSleep && asleep) {
					rb->asleep = true;
					target = BroadphaseBucket::Asleep;
				}
				else {
					rb->asleep = false;
					target = BroadphaseBucket::Awake;
				}

				ctx.physicsEngine->queueMove(obj->rigidBodyHandle, target);
			}
			ImGui::EndDisabled();
		}

		// AllowGravity checkbox
		{
			bool allowGravity = rb->allowGravity;
			if (RowCheckbox("Gravity", "##allowGravity", allowGravity)) {
				rb->allowGravity = allowGravity;
				if (allowGravity) {
					rb->allowGravity = true;
				}
				else {
					rb->allowGravity = false;
				}
			}
		}

		// Mass
		{
			float mass = rb->mass;
			if (RowDragFloat("Mass", "##mass", mass, 1.0f, 0.1f, 1000000.f)) {
				if (rb->type != BodyType::Static) {
					rb->mass = mass;
					rb->invMass = 1.0f / mass;
				}
			}
			if (ImGui::IsItemDeactivatedAfterEdit()) {
				rb->calculateInverseInertia(collider->type, *collider, obj->transform);
				rb->updateInertiaWorld(obj->transform);
			}
		}
	}

	if (rb->type != BodyType::Static)
	{
		// Linear velocity
		{
			glm::vec3 vel = rb->linearVelocity;
			if (RowDragFloat3("Linear vel", "##linvel", vel, 0.1f, -1000.f, 1000.f, 1)) {
				rb->linearVelocity = vel;
			}
		}

		// Angular velocity
		{
			glm::vec3 angVel = rb->angularVelocity;
			if (RowDragFloat3("Angular vel", "##angvel", angVel, 0.1f, -1000.f, 1000.f, 1)) {
				rb->angularVelocity = angVel;
			}
		}
	}

	EndSection();

	ImGui::PopStyleVar(2);
	ImGui::End();
}