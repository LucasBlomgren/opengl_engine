#include "pch.h"
#include "performance_panel.h"
#include "editor/panel.h"
#include "imgui.h"
#include "imgui_manager.h"
#include "time.h"

void Editor::PerformancePanel::OnImGuiRender(const PanelContext& ctx)
{
	ImGui::Begin("Performance");

	// Ringbuffer för frametime i ms
	static float ft_ms[400] = {};
	static int   ft_i = 0;

	static float sampleAcc = 0.0f;
	static float lastDtMs = 0.0f;

	const float sampleHz = 60.0f;
	const float sampleDt = 1.0f / sampleHz;

	sampleAcc += ctx.deltaTime;
	lastDtMs = ctx.deltaTime * 1000.0f;

	int ticks = (int)(sampleAcc / sampleDt);
	if (ticks > 0)
	{
		sampleAcc -= ticks * sampleDt;
		ticks = std::min(ticks, 100);

		for (int i = 0; i < ticks; ++i)
		{
			ft_ms[ft_i] = lastDtMs; // håll senaste värdet
			ft_i = (ft_i + 1) % IM_ARRAYSIZE(ft_ms);
		}
	}

	// UI-smoothade värden
	static float dt_ui = 0.0f;
	static float framerate_ui = 0.0f;

	static float renderSub_ui = 0.0f;
	static float editor_ui = 0.0f;
	static float imgui_ui = 0.0f;
	static float phys_ui = 0.0f;

	static float cpu_total_ms = 0.0f;
	static float gpu_total_ms = 0.0f;

	static float gpuShadowMs_ui = 0.0f;
	static float gpuMainMs_ui = 0.0f;
	static float gpuDebugMs_ui = 0.0f;

	static float preStep_ui = 0.0f;
	static float bvhUpdate_ui = 0.0f;
	static float broadphase_ui = 0.0f;
	static float narrowphase_ui = 0.0f;
	static float contactCollect_ui = 0.0f;
	static float collisionResolve_ui = 0.0f;
	static float postStep_ui = 0.0f;

	// --- update 20 Hz ---
	uiTimer += ctx.deltaTime;
	if (uiTimer >= 0.2f)
	{
		uiTimer = 0.0f;

		::FrameTimers* frameTimers = ctx.frameTimers;
		::GpuTimers* gpuTimers = ctx.gpuTimers;

		ImGuiIO& io = ImGui::GetIO();
		framerate_ui = io.Framerate;
		dt_ui = ctx.deltaTime * 1000.0f;

		gpu_total_ms = gpuTimers->totalMs();
		gpuShadowMs_ui = gpuTimers->shadowMs;
		gpuMainMs_ui = gpuTimers->mainMs;
		gpuDebugMs_ui = gpuTimers->debugMs;

		renderSub_ui = frameTimers->get("Render");
		editor_ui = frameTimers->get("Editor");
		imgui_ui = frameTimers->get("ImGui");
		phys_ui = frameTimers->get("Physics");
		cpu_total_ms = phys_ui + renderSub_ui + editor_ui + imgui_ui;

		preStep_ui = frameTimers->get("Pre step");
		bvhUpdate_ui = frameTimers->get("BVH update");
		broadphase_ui = frameTimers->get("Broadphase");
		narrowphase_ui = frameTimers->get("Narrowphase");
		contactCollect_ui = frameTimers->get("Contact collection");
		collisionResolve_ui = frameTimers->get("Collision resolution");
		postStep_ui = frameTimers->get("Post step");
	}

	// --- helpers ---
	auto BarCell = [&](float ms, float denom_ms)
		{
			float rawPct = (denom_ms > 0.f) ? (ms / denom_ms) : 0.f;
			float barPct = std::clamp(rawPct, 0.0f, 1.0f);

			float wFull = ImGui::GetContentRegionAvail().x;
			float barW = wFull * 0.8f;
			float barH = 12.0f;

			ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.75f);
			ImGui::ProgressBar(barPct, ImVec2(barW, barH), "");
			ImGui::PopStyleVar();

			if (ImGui::IsItemHovered())
			{
				ImGui::BeginTooltip();
				ImGui::Text("Time:  %.2f ms", ms);
				ImGui::Text("Share: %.1f%%", rawPct * 100.0f);
				ImGui::EndTooltip();
			}
		};

	auto Row = [&](const char* name, float ms, float denom_ms)
		{
			ImGui::TableNextRow();

			ImGui::TableNextColumn();
			ImGui::TextUnformatted(name);

			ImGui::TableNextColumn();
			ImGui::Text("%.2f ms", ms);

			ImGui::TableNextColumn();
			BarCell(ms, denom_ms);
		};

	// ---- Header ----
	ImGui::Text("Frame %.2f ms | %.0f FPS", dt_ui, framerate_ui);
	// Tooltip för headern (CPU/GPU total)
	if (ImGui::IsItemHovered())
	{
		ImGui::BeginTooltip();
		ImGui::Text("CPU  %.2f ms", cpu_total_ms);
		ImGui::Text("GPU  %.2f ms", gpu_total_ms);
		ImGui::EndTooltip();
	}
	//ImGui::SameLine();
	ImGui::TextDisabled("Objects: %zu", ctx.amountObjects);

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();

	// ---- Plot ----
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
	ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 0));
	ImGui::PushStyleColor(ImGuiCol_PlotLines, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));

	ImGui::PlotLines("##ft", ft_ms, IM_ARRAYSIZE(ft_ms), ft_i, nullptr, 0.0f, 16.6f, ImVec2(-1, 100));

	ImGui::PopStyleColor(2);
	ImGui::PopStyleVar();

	ImGui::Separator();

	// ---- Table ----
	ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(6, 3));
	ImGuiTableFlags tflags =
		ImGuiTableFlags_RowBg |
		ImGuiTableFlags_SizingFixedFit |
		ImGuiTableFlags_NoBordersInBody;

	if (ImGui::BeginTable("perf_tbl", 3, tflags))
	{
		ImGui::TableSetupColumn("Pass", ImGuiTableColumnFlags_WidthFixed, 110.0f);
		ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 72.0f);
		ImGui::TableSetupColumn("Share", ImGuiTableColumnFlags_WidthStretch);
		ImGui::TableHeadersRow();

		// --- GPU group header ---
		ImGui::TableNextRow();
		ImGui::TableNextColumn();
		ImGui::TextDisabled("GPU");
		if (ImGui::IsItemHovered())
		{
			ImGui::BeginTooltip();
			ImGui::Text("CPU  %.2f ms", cpu_total_ms);
			ImGui::Text("GPU  %.2f ms", gpu_total_ms);
			ImGui::EndTooltip();
		}
		ImGui::TableNextColumn(); ImGui::TableNextColumn();

		Row("Shadow", gpuShadowMs_ui, gpu_total_ms);
		Row("Main", gpuMainMs_ui, gpu_total_ms);
		Row("Debug", gpuDebugMs_ui, gpu_total_ms);

		// --- CPU group header ---
		ImGui::TableNextRow();
		ImGui::TableNextColumn();
		ImGui::TextDisabled("CPU");
		if (ImGui::IsItemHovered())
		{
			ImGui::BeginTooltip();
			ImGui::Text("CPU  %.2f ms", cpu_total_ms);
			ImGui::Text("GPU  %.2f ms", gpu_total_ms);
			ImGui::EndTooltip();
		}
		ImGui::TableNextColumn(); ImGui::TableNextColumn();

		// Render submit som vanlig rad (share av CPU total)
		Row("Render submit", renderSub_ui, cpu_total_ms);
		Row("Editor", editor_ui, cpu_total_ms);
		Row("ImGui", imgui_ui, cpu_total_ms);

		// === Physics (expanderbar) ===
		ImGui::TableNextRow();

		ImGui::TableNextColumn();
		bool physicsOpen = ImGui::TreeNodeEx("Physics",
			ImGuiTreeNodeFlags_SpanFullWidth | ImGuiTreeNodeFlags_FramePadding,
			"Physics");

		ImGui::TableNextColumn();
		ImGui::Text("%.2f ms", phys_ui);

		ImGui::TableNextColumn();
		BarCell(phys_ui, cpu_total_ms);

		if (physicsOpen)
		{
			// Hämta subtimers (byt keys till dina riktiga)
			struct Sub { const char* name; float ms; };
			Sub subs[] = {
				{"Pre",      preStep_ui},
				{"BVH",      bvhUpdate_ui},
				{"Broad",    broadphase_ui},
				{"Narrow",   narrowphase_ui},
				{"Contacts", contactCollect_ui},
				{"Solver",   collisionResolve_ui},
				{"Post",     postStep_ui}
			};

			ImGui::Spacing();
			ImGui::Spacing();
			ImGui::Spacing();

			//ImGui::TableNextRow(0, ImGui::GetTextLineHeightWithSpacing());
			ImGui::TableNextRow();

			for (auto& s : subs)
			{
				ImGui::TableNextColumn();
				ImGui::Indent(20.0f);
				ImGui::TextUnformatted(s.name);   // TextUnformatted: för färdig sträng
				ImGui::Unindent(20.0f);

				ImGui::TableNextColumn();
				ImGui::Text("%.2f ms", s.ms);

				ImGui::TableNextColumn();
				BarCell(s.ms, phys_ui); // share inom Physics
			}

			ImGui::TreePop();
		}

		ImGui::EndTable();
	}

	ImGui::PopStyleVar();
	ImGui::End();
}