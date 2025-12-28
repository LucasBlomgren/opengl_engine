#include "pch.h"
#include "imgui_manager.h"

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

//--------------------------------
//           Main UI
// -------------------------------
void ImGuiManager::mainUI(float deltaTime, FrameTimers& frameTimers, GpuTimers& gpuT, size_t amountObjects) {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImVec2 pos = vp->WorkPos;    // tar hänsyn till ev. menu bar/dockspace
    ImVec2 size = vp->WorkSize;  // “arbetsytan” i viewporten

    const float w = 310.0f;      // din fasta bredd, ändra som du vill

    ImGui::SetNextWindowPos(pos, ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(w, size.y), ImGuiCond_Always);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 5.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::Begin("Engine", nullptr,
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize |          // rekommenderas när du styr size själv
        ImGuiWindowFlags_NoMove);            // rekommenderas när du styr pos själv

    if (ImGui::CollapsingHeader("Performance", ImGuiTreeNodeFlags_DefaultOpen))
        performanceUI(deltaTime, frameTimers, gpuT, amountObjects);

    ImGui::NewLine();

    if (ImGui::CollapsingHeader("Settings", ImGuiTreeNodeFlags_DefaultOpen))
        settingsUI();

    ImGui::End();
	ImGui::PopStyleVar(2);
}

//--------------------------------
//          Settings UI
// -------------------------------
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

    //static glm::vec3 pos{ 0.0f, 1.0f, 2.0f };
    //ImGui::SliderFloat3("Position", (float*)&pos, -10.0f, 10.0f);
}

//--------------------------------
//         Performance UI
// -------------------------------
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
    uiTimer += deltaTime;
    if (uiTimer >= 0.2f)
    {
        uiTimer = 0.0f;

        framerate_ui = io->Framerate;
        dt_ui = deltaTime * 1000.0f;

        gpu_total_ms = gpuT.totalMs();
        gpuShadowMs_ui = gpuT.shadowMs;
        gpuMainMs_ui = gpuT.mainMs;
        gpuDebugMs_ui = gpuT.debugMs;

        renderSub_ui = frameTimers.get("Render");
        editor_ui = frameTimers.get("Editor");
        imgui_ui = frameTimers.get("ImGui");
        phys_ui = frameTimers.get("Physics");
        cpu_total_ms = phys_ui + renderSub_ui + editor_ui + imgui_ui;

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
    // Tooltip för headern (CPU/GPU total)
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::Text("CPU  %.2f ms", cpu_total_ms);
        ImGui::Text("GPU  %.2f ms", gpu_total_ms);
        ImGui::EndTooltip();
    }
    ImGui::SameLine();
    ImGui::TextDisabled(" | Objects: %zu", amountObjects);

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
}

//--------------------------------
//       Selected object UI
// -------------------------------
void ImGuiManager::selectedObjectUI(GameObject* objPtr)
{
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImVec2 workPos = vp->WorkPos;
    ImVec2 workSize = vp->WorkSize;

    const float w = 310.0f;

    const float pad = 0.0f;
    ImVec2 rightPos(workPos.x + workSize.x - w - pad, workPos.y + pad);
    ImGui::SetNextWindowSize(ImVec2(w, workSize.y - 2 * pad), ImGuiCond_Always);
    ImGui::SetNextWindowPos(rightPos, ImGuiCond_Always);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 5.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::Begin("Selected Object", nullptr,
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove
        // | ImGuiWindowFlags_NoSavedSettings  // (valfritt) om du inte vill att imgui.ini påverkar
    );

    if (!ImGui::CollapsingHeader("Inspector", ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_Framed)) {
        ImGui::End();
        ImGui::PopStyleVar(2);
        return;
    }

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
        static int currentItem = 0;
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
        static bool init = false;
        static glm::vec3 uiDeg;
        static glm::vec3 lastUiDeg;

        if (!init) {
            uiDeg = glm::degrees(glm::eulerAngles(obj.orientation));
            lastUiDeg = uiDeg;
            init = true;
        }

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
        if (RowCheckbox("Static", "##static", isStatic)) {      // får dubbel gravitation för den hamnar i både statisk och dynamisk bvh 
            obj.isStatic = isStatic;
            if (isStatic) {
                obj.mass = 0.0f;
                obj.invMass = 0.0f;
                obj.linearVelocity = glm::vec3(0.0f);
                obj.angularVelocity = glm::vec3(0.0f);
                obj.allowGravity = false;
                obj.asleep = false;
            }
            else {
                obj.mass = 1.0f;
                obj.invMass = 1.0f / obj.mass;
                obj.allowGravity = true;
                obj.linearVelocity = glm::vec3(0.0f);
                obj.angularVelocity = glm::vec3(0.0f);

                obj.allowSleep = false; // // hack for static object made dynamic (because cant move from static bvh to dynamic/asleep bvh)
                obj.sleepCounterThreshold = FLT_MAX;
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