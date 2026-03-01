# opengl_engine
A lightweight C++/OpenGL real-time engine built from scratch with a focus on physics, performance, and handling many objects in real time.

## Demos (click to watch on YouTube)

<p align="center">
  <a href="https://www.youtube.com/watch?v=hdSbTTQnSto" target="_blank" rel="noopener noreferrer">
    <img src="media/pyramid.gif" width="250" />
  </a>
  
  <a href="https://www.youtube.com/watch?v=rpU5otEmwG8" target="_blank" rel="noopener noreferrer">
    <img src="media/terrain.gif" width="250" />
  </a>
  &nbsp;&nbsp;&nbsp;
  <a href="https://www.youtube.com/watch?v=9N-Z2AVxhxM" target="_blank" rel="noopener noreferrer">
    <img src="media/tumbler.gif" width="250" />
  </a>
</p>

<p align="center">
  <sub><b>Pyramid stack</b></sub>
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <sub><b>Terrain + BVH debug</b></sub>
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <sub><b>Physics based tumbler</b></sub>
</p>

<br/>

<p align="center">
  <a href="https://www.youtube.com/watch?v=LhIm9BXafNY" target="_blank" rel="noopener noreferrer">
    <img src="media/objectmanip.gif" width="250" />
  </a>
  &nbsp;&nbsp;&nbsp;
  <a href="https://www.youtube.com/watch?v=CNvF_xG1Sus" target="_blank" rel="noopener noreferrer">
    <img src="media/inspector.gif" width="250" />
  </a>
  &nbsp;&nbsp;&nbsp;
  <a href="https://www.youtube.com/watch?v=7ER52cchTe0" target="_blank" rel="noopener noreferrer">
    <img src="media/debug.gif" width="250" />
  </a>
</p>

<p align="center">
  <sub><b>Object manipulation</b></sub>
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <sub><b>Object inspector</b></sub>
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <sub><b>Debug visualizations</b></sub>
</p>

## 1) General
**opengl_engine** (name subject to change) is a personal engine project focused on building core real-time systems end-to-end. It’s primarily a learning + experimentation repository: features evolve quickly, APIs may change, and the main goal is to iterate on design, performance, and debuggability - especially for physics-heavy scenes with lots of objects.

## Build & Run
- Platform: **Windows**
- IDE: **Visual Studio** (open `opengl_engine.sln`)
- Recommended: **x64 / Release**

Dependencies (GLFW/GLAD/GLM/ImGui/stb) are vendored under `opengl/Linking/`.

## 2) Physics
- Custom rigid body simulation (`rigid_body`, `physics`, `physics_world`)
- Broad-phase collision management (`broadphase_manager`, `broadphase_pairs`)
- BVH acceleration structures, including terrain support (`bvh`, `bvh_terrain`, `treetree_query`)
- Narrow-phase collision detection using SAT (`sat`)
- Collision manifolds + contact generation with multi-contact support (`collision_manifold`)
- Sleeping/awake handling to reduce simulation cost in large scenes (integrated in the physics update flow)
- Raycasting utilities (`raycast`)
- Collider primitives: AABB, OOBB, sphere, triangle (`aabb`, `oobb`, `sphere`, `tri`, `collider`)

## 3) Renderer
- OpenGL renderer responsible for scene drawing (`renderer`)
- Shader system + built-in GLSL shaders (`shader_manager`, `default`, `shadow`, `skybox`, debug shaders)
- Mesh loading and asset management (`mesh_loader`, `mesh_manager`, `mesh`)
- Texture handling (`texture_manager`)
- Lighting and shadow systems (`light`, `light_manager`, `shadow_manager`)
- Debug rendering utilities for visualizing engine state (AABB/OOBB, normals, lines, contact points, etc.)  
  (`aabb_renderer`, `oobb_renderer`, `normals_renderer`, `draw_line`, `render_contact_points`, `arrow_renderer`, ...)

## 4) Editor
- Editor UI and integration (`editor_main`, `imgui_manager`)
- Panel framework + built-in panels (`panel`, `panel_manager`, `inspector_panel`, `performance_panel`, `settings_panel`)
- Viewport rendering infrastructure (`viewport_fbo`)
- Interaction workflows intended to support fast iteration while developing the engine (inspection + debug visualization + object selection/raycast integration)

## License
MIT (see `LICENSE`).
