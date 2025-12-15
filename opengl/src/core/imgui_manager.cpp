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

    //static glm::vec3 pos{ 0.0f, 1.0f, 2.0f };
    //ImGui::SliderFloat3("Position", (float*)&pos, -10.0f, 10.0f);
}

void ImGuiManager::performanceUI(float deltaTime, FrameTimers& frameTimers, GpuTimers& gpuT, size_t amountObjects)
{
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

    static float phys_ui = 0.0f;
    static float renderSub_ui = 0.0f;

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
    uiTimer += deltaTime;
    if (uiTimer >= 0.2f)
    {
        uiTimer = 0.0f;

        framerate_ui = io->Framerate;
        dt_ui = deltaTime * 1000.0f;

        renderSub_ui = frameTimers.get("Render");
        phys_ui = frameTimers.get("Physics");

        cpu_total_ms = phys_ui + renderSub_ui;
        gpu_total_ms = gpuT.totalMs();

        gpuShadowMs_ui = gpuT.shadowMs;
        gpuMainMs_ui = gpuT.mainMs;
        gpuDebugMs_ui = gpuT.debugMs;

        preStep_ui = frameTimers.get("Pre step");
        bvhUpdate_ui = frameTimers.get("BVH update");
        broadphase_ui = frameTimers.get("Broadphase");
        narrowphase_ui = frameTimers.get("Narrowphase");
        contactCollect_ui = frameTimers.get("Contact collection");
        collisionResolve_ui = frameTimers.get("Collision resolution");
        postStep_ui = frameTimers.get("Post step");
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
    ImGui::SameLine();
    ImGui::TextDisabled(" | Objects: %zu", amountObjects);

    // Tooltip för headern (CPU/GPU total)
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::Text("CPU  %.2f ms", cpu_total_ms);
        ImGui::Text("GPU  %.2f ms", gpu_total_ms);
        ImGui::EndTooltip();
    }

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
        ImGui::TableNextColumn(); ImGui::TextDisabled("GPU");
        ImGui::TableNextColumn(); ImGui::TableNextColumn();

        Row("Shadow", gpuShadowMs_ui, gpu_total_ms);
        Row("Main", gpuMainMs_ui, gpu_total_ms);
        Row("Debug", gpuDebugMs_ui, gpu_total_ms);

        // --- CPU group header ---
        ImGui::TableNextRow();
        ImGui::TableNextColumn(); ImGui::TextDisabled("CPU");
        ImGui::TableNextColumn(); ImGui::TableNextColumn();

        // Render submit som vanlig rad (share av CPU total)
        Row("Render submit", renderSub_ui, cpu_total_ms);

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

            for (auto& s : subs)
            {
                ImGui::TableNextRow();

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
}

void ImGuiManager::selectedObjectUI(GameObject* objPtr)
{
    ImGui::Begin("Selected Object", nullptr, ImGuiWindowFlags_NoTitleBar);

    if (!objPtr) {
        ImGui::TextDisabled("No object selected");
        ImGui::End();
        return;
    }

    GameObject& obj = *objPtr;
    ImGui::Text("Position: (%.2f, %.2f, %.2f)", obj.position.x, obj.position.y, obj.position.z);

    glm::vec3 scale = obj.scale;
    if (ImGui::DragFloat3("Scale", &scale.x, 0.1f, 0.5f, 1000.f)) {
        obj.scale = scale;
    }
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        obj.calculateInverseInertia();
    }

    glm::vec3 uiEulerDeg = glm::degrees(glm::eulerAngles(obj.orientation));
    if (ImGui::DragFloat3("Rotation (deg)", &uiEulerDeg.x, 0.5f, -180.f, 180.f)) {
        obj.orientation = glm::quat(glm::radians(uiEulerDeg));
    }

    ImGui::End();
}