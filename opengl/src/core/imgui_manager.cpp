#include "pch.h"
#include "imgui_manager.h"

void ImGuiManager::init(GLFWwindow* window, EngineState& es, SceneBuilder& sb) {
    this->engineState = &es;
    this->sceneBuilder = &sb;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    io = &ImGui::GetIO(); 
    (void)io;

    // init backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

void ImGuiManager::newFrame() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void ImGuiManager::setInputMode(bool cameraMode) {
    if (cameraMode) {
        io->MouseDown[0] = false; 
        io->MouseDown[1] = false;
        io->MouseDown[2] = false;
        io->MousePos = ImVec2(-FLT_MAX, -FLT_MAX);
    }
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

void ImGuiManager::mainUI(float deltaTime, FrameTimers& frameTimers, GpuTimers& gpuT, size_t amountObjects) {
    ImGui::Begin("Engine", nullptr, ImGuiWindowFlags_NoTitleBar);

    if (ImGui::CollapsingHeader("Performance", ImGuiTreeNodeFlags_DefaultOpen))
        performanceUI(deltaTime, frameTimers, gpuT, amountObjects);

    ImGui::NewLine();

    if (ImGui::CollapsingHeader("Settings", ImGuiTreeNodeFlags_DefaultOpen))
        settingsUI();

    ImGui::End();
}

void ImGuiManager::settingsUI() {
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
    static int currentItem = 0;
    const char* items[] = { "Test", "Empty", "Main", "Terrain", "Tall structure", "Tumbler", "Castle", "Invisible container" };
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
        default: break;
        }
    }

    //static glm::vec3 pos{ 0.0f, 1.0f, 2.0f };
    //ImGui::SliderFloat3("Position", (float*)&pos, -10.0f, 10.0f);
}

void ImGuiManager::performanceUI(float deltaTime, FrameTimers& frameTimers, GpuTimers& gpuT, size_t amountObjects) {
    // Ringbuffer för frametime i ms
    static float ft_ms[400] = {};
    static int   ft_i = 0;

    static float sampleAcc = 0.0f;
    static float lastDtMs = 0.0f;

    const float sampleHz = 60.0f;
    const float sampleDt = 1.0f / sampleHz;

    sampleAcc += deltaTime;
    lastDtMs = deltaTime * 1000.0f;

    int ticks = (int)(sampleAcc / sampleDt);
    if (ticks > 0) {
        sampleAcc -= ticks * sampleDt;
        ticks = std::min(ticks, 100); // valfritt clamp

        for (int i = 0; i < ticks; ++i) {
            ft_ms[ft_i] = lastDtMs;                       // håll senaste värdet
            ft_i = (ft_i + 1) % IM_ARRAYSIZE(ft_ms);
        }
    }

    static float dt_ui          = 0.0f;
    static float framerate_ui   = 0.0f;
    static float phys_ui        = 0.0f;
    static float renderSub_ui   = 0.0f;
    static float cpu_total_ms   = 0.0f;
    static float gpu_total_ms   = 0.0f;
    static float gpuShadowMs_ui = 0.0f;
    static float gpuMainMs_ui   = 0.0f;
    static float gpuDebugMs_ui  = 0.0f;

    uiTimer += deltaTime;
    if (uiTimer >= 0.2f) {      // 20 Hz
        uiTimer = 0.0f;

        framerate_ui    = io->Framerate;
        dt_ui           = deltaTime * 1000.0f;

        phys_ui         = frameTimers.get("Physics");
        renderSub_ui    = frameTimers.get("Render");

        cpu_total_ms    = phys_ui + renderSub_ui;
        gpu_total_ms    = gpuT.totalMs();

        gpuShadowMs_ui  = gpuT.shadowMs;
        gpuMainMs_ui    = gpuT.mainMs;
        gpuDebugMs_ui   = gpuT.debugMs;
    }

    struct PassStat { const char* name; float ms; };
    PassStat gpu[] = { {"Shadow", gpuShadowMs_ui}, {"Main", gpuMainMs_ui}, {"Debug", gpuDebugMs_ui} };
    PassStat cpu[] = { {"Physics", phys_ui}, {"Render submit", renderSub_ui} };

    auto RowBar = [&](const char* name, float ms, float denom_ms)
        {
            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            ImGui::TextUnformatted(name);

            ImGui::TableNextColumn();
            ImGui::Text("%.2f ms", ms);

            ImGui::TableNextColumn();
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
                ImGui::Text("%s", name);
                ImGui::Separator();
                ImGui::Text("Time:  %.2f ms", ms);
                ImGui::Text("Share: %.0f%%", rawPct * 100.0f);
                ImGui::EndTooltip();
            }
        };

    // ---- Header ----
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Frame %.2f ms | %.0f FPS", dt_ui, framerate_ui);
    ImGui::SameLine();
    ImGui::TextDisabled(" | Objects: %zu", amountObjects);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Tooltip med resten
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::Text("CPU  %.2f ms", cpu_total_ms);
        ImGui::Text("GPU  %.2f ms", gpu_total_ms);
        ImGui::EndTooltip();
    }

    // ---- Plot ----
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));                              // mindre “box”
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 0));                                // transparent bakgrund
    ImGui::PushStyleColor(ImGuiCol_PlotLines, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled)); // subtil linje

    ImGui::PlotLines("##ft", ft_ms, IM_ARRAYSIZE(ft_ms), ft_i, nullptr,0.0f, 16.6f, ImVec2(-1, 100)); // 16.6ms = 60fps-budget

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
        ImGui::TableSetupColumn("Pass");
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 72.0f);
        ImGui::TableSetupColumn("Share", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::TextDisabled("GPU");
        ImGui::TableNextColumn(); ImGui::TableNextColumn();
        for (auto& p : gpu) RowBar(p.name, p.ms, gpu_total_ms);

        ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::TextDisabled("CPU");
        ImGui::TableNextColumn(); ImGui::TableNextColumn();
        for (auto& p : cpu) RowBar(p.name, p.ms, cpu_total_ms);

        ImGui::EndTable();
    }
    ImGui::PopStyleVar();
}