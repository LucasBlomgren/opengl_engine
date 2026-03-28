#include "pch.h"
#include "performance_panel.h"
#include "editor/panel.h"
#include "imgui.h"
#include "imgui_manager.h"
#include "core/timer.h"
#include "core/engine_state.h"

#include <algorithm>
#include <cstdio>
#include <string>

namespace
{
    std::string FormatCompactCount(size_t value)
    {
        char buf[32];

        if (value >= 1000000)
        {
            std::snprintf(buf, sizeof(buf), "%.1fm", value / 1000000.0);
            return buf;
        }
        if (value >= 1000)
        {
            std::snprintf(buf, sizeof(buf), "%.1fk", value / 1000.0);
            return buf;
        }

        std::snprintf(buf, sizeof(buf), "%zu", value);
        return buf;
    }
}

void Editor::PerformancePanel::OnImGuiRender(const PanelContext& ctx)
{
    // ----------------------------------------------------
    //   Persistent UI state
    // ----------------------------------------------------
    static bool showFullProfiler = false;

    // ----------------------------------------------------
    //   Frametime history
    // ----------------------------------------------------
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
            ft_ms[ft_i] = lastDtMs;
            ft_i = (ft_i + 1) % IM_ARRAYSIZE(ft_ms);
        }
    }

    // ----------------------------------------------------
    //   Smoothed values (updates at 20 Hz)
    // ----------------------------------------------------
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

    const std::string objCompact = FormatCompactCount(ctx.amountObjects);

    // ----------------------------------------------------
    //   Shared helpers
    // ----------------------------------------------------
    auto PushOverlayStyle = []()
        {
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 6.0f));
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0f, 2.0f));
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6.0f, 4.0f));
            ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(4.0f, 2.0f));
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);

            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(1.f, 1.f, 1.f, 0.08f));
            ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(1.f, 1.f, 1.f, 0.10f));
            ImGui::PushStyleColor(ImGuiCol_TableRowBg, ImVec4(0.f, 0.f, 0.f, 0.00f));
            ImGui::PushStyleColor(ImGuiCol_TableRowBgAlt, ImVec4(1.f, 1.f, 1.f, 0.03f));
        };

    auto PopOverlayStyle = []()
        {
            ImGui::PopStyleColor(4);
            ImGui::PopStyleVar(6);
        };

    auto TotalTooltip = [&]()
        {
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text("CPU  %.2f ms", cpu_total_ms);
                ImGui::Text("GPU  %.2f ms", gpu_total_ms);
                ImGui::Separator();
                ImGui::TextUnformatted("Double click: open full profiler");
                ImGui::EndTooltip();
            }
        };

    auto DrawSparkline = [&](float width, float height)
        {
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 0));
            ImGui::PushStyleColor(ImGuiCol_PlotLines, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));

            const float fpsBottom = 144.0f;
            const float fpsTop = 30.0f;
            const float msMin = 1000.0f / fpsBottom;
            const float msMax = 1000.0f / fpsTop;

            ImGui::PlotLines("##ft", ft_ms, IM_ARRAYSIZE(ft_ms), ft_i, nullptr, msMin, msMax, ImVec2(width, height));

            ImGui::PopStyleColor(2);
            ImGui::PopStyleVar();
        };

    auto DrawSummaryTable = [&](const char* tableId)
        {
            ImGuiTableFlags flags =
                ImGuiTableFlags_SizingStretchSame |
                ImGuiTableFlags_NoBordersInBody |
                ImGuiTableFlags_NoPadOuterX;

            if (ImGui::BeginTable(tableId, 2, flags))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Objects: %zu", ctx.amountObjects);
                ImGui::TableNextColumn();
                ImGui::Text("Awake: %zu", ctx.amountAwakeObjects);

                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Static: %zu", ctx.amountStaticObjects);
                ImGui::TableNextColumn();
                ImGui::Text("Asleep: %zu", ctx.amountAsleepObjects);

                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Terrain: %zu", ctx.amountTerrainTris);
                ImGui::TableNextColumn();
                ImGui::Text("Collisions: %zu", ctx.amountCollisions);

                ImGui::EndTable();
            }
        };

    auto BarCell = [&](float ms, float denom_ms, float alpha = 0.65f, float height = 10.0f)
        {
            const float rawPct = (denom_ms > 0.0f) ? (ms / denom_ms) : 0.0f;
            const float barPct = std::clamp(rawPct, 0.0f, 1.0f);
            const float width = ImGui::GetContentRegionAvail().x;

            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, alpha);
            ImGui::ProgressBar(barPct, ImVec2(width, height), "");
            ImGui::PopStyleVar();

            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text("Time:  %.2f ms", ms);
                ImGui::Text("Share: %.1f%%", rawPct * 100.0f);
                ImGui::EndTooltip();
            }
        };

    auto DrawTimingTable = [&](const char* tableId, bool showHeaders)
        {
            ImGuiTableFlags flags =
                ImGuiTableFlags_RowBg |
                ImGuiTableFlags_SizingStretchProp |
                ImGuiTableFlags_NoBordersInBody |
                ImGuiTableFlags_NoPadOuterX;

            if (!ImGui::BeginTable(tableId, 3, flags))
                return;

            ImGui::TableSetupColumn("Pass", ImGuiTableColumnFlags_WidthFixed, 78.0f);
            ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 52.0f);
            ImGui::TableSetupColumn("Share", ImGuiTableColumnFlags_WidthStretch);

            if (showHeaders)
                ImGui::TableHeadersRow();

            auto SimpleRow = [&](const char* name, float ms, float denom_ms)
                {
                    ImGui::TableNextRow();

                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(name);

                    ImGui::TableNextColumn();
                    ImGui::Text("%.2f", ms);

                    ImGui::TableNextColumn();
                    BarCell(ms, denom_ms);
                };

            auto GroupLabelRow = [&](const char* label, float totalMs)
                {
                    ImGui::TableNextRow();

                    ImGui::TableNextColumn();
                    ImGui::TextDisabled("%s", label);

                    ImGui::TableNextColumn();
                    ImGui::TextDisabled("%.2f", totalMs);

                    ImGui::TableNextColumn();
                };

            GroupLabelRow("GPU", gpu_total_ms);
            SimpleRow("Shadow", gpuShadowMs_ui, gpu_total_ms);
            SimpleRow("Main", gpuMainMs_ui, gpu_total_ms);
            SimpleRow("Debug", gpuDebugMs_ui, gpu_total_ms);

            GroupLabelRow("CPU", cpu_total_ms);
            SimpleRow("Render", renderSub_ui, cpu_total_ms);
            SimpleRow("Editor", editor_ui, cpu_total_ms);
            SimpleRow("ImGui", imgui_ui, cpu_total_ms);

            char physicsLabel[128];
            std::snprintf(physicsLabel, sizeof(physicsLabel), "Physics##%s", tableId);

            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            bool physicsOpen = ImGui::TreeNodeEx(
                physicsLabel,
                ImGuiTreeNodeFlags_SpanFullWidth |
                ImGuiTreeNodeFlags_FramePadding |
                ImGuiTreeNodeFlags_DefaultOpen,
                "Physics"
            );

            ImGui::TableNextColumn();
            ImGui::Text("%.2f", phys_ui);

            ImGui::TableNextColumn();
            BarCell(phys_ui, cpu_total_ms);

            if (physicsOpen)
            {
                struct Sub { const char* name; float ms; };
                Sub subs[] = {
                    {"Pre",    preStep_ui},
                    {"BVH",    bvhUpdate_ui},
                    {"Broad",  broadphase_ui},
                    {"Narrow", narrowphase_ui},
                    {"Sort",   contactCollect_ui},
                    {"Solve",  collisionResolve_ui},
                    {"Post",   postStep_ui}
                };

                for (const auto& s : subs)
                {
                    ImGui::TableNextRow();

                    ImGui::TableNextColumn();
                    ImGui::Indent(10.0f);
                    ImGui::TextUnformatted(s.name);
                    ImGui::Unindent(10.0f);

                    ImGui::TableNextColumn();
                    ImGui::Text("%.2f", s.ms);

                    ImGui::TableNextColumn();
                    BarCell(s.ms, phys_ui);
                }

                ImGui::TreePop();
            }

            ImGui::EndTable();
        };

    // ----------------------------------------------------
    //  Compact always-visible HUD
    // ----------------------------------------------------
    ImVec2 hudPos(12.0f, 12.0f);
    ImVec2 hudSize(0.0f, 0.0f);

    PushOverlayStyle();
    ImGui::SetNextWindowBgAlpha(0.35f);

    ImGuiWindowFlags hudFlags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav |
        ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::Begin("##PerformanceHUD", nullptr, hudFlags);

    ImGui::Text("%.0f FPS   %.2f ms", framerate_ui, dt_ui);
    TotalTooltip();

    ImGui::TextDisabled("CPU %.2f   GPU %.2f", cpu_total_ms, gpu_total_ms);
    ImGui::TextDisabled("Obj %s   Col %zu", objCompact.c_str(), ctx.amountCollisions);

    DrawSparkline(190.0f, 18.0f);

    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByPopup) &&
        ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
    {
        showFullProfiler = !showFullProfiler;
    }

    if (ImGui::BeginPopupContextWindow("PerformanceHudContext"))
    {
        if (ImGui::MenuItem("Open full profiler"))
            showFullProfiler = true;

        ImGui::EndPopup();
    }

    hudPos = ImGui::GetWindowPos();
    hudSize = ImGui::GetWindowSize();

    ImGui::End();
    PopOverlayStyle();

   
    // ----------------------------------------------------
    //  Full profiler window
    // ----------------------------------------------------
    if (showFullProfiler)
    {
        ImGui::SetNextWindowSize(ImVec2(460.0f, 520.0f), ImGuiCond_FirstUseEver);

        if (ImGui::Begin("Profiler", &showFullProfiler))
        {
            ImGui::Text("%.0f FPS   Frame %.2f ms", framerate_ui, dt_ui);
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text("CPU  %.2f ms", cpu_total_ms);
                ImGui::Text("GPU  %.2f ms", gpu_total_ms);
                ImGui::EndTooltip();
            }

            ImGui::Spacing();

            DrawSparkline(ImGui::GetContentRegionAvail().x, 70.0f);

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            DrawSummaryTable("FullProfilerSummaryTable");

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            DrawTimingTable("FullProfilerTimingTable", true);
        }
        ImGui::End();
    }
}