# opengl_engine
A lightweight C++/OpenGL real-time engine built from scratch to explore engine architecture, rendering, physics, and editor tooling.

## 1) General
**opengl_engine** is a personal engine project focused on building core real-time systems end-to-end. It’s primarily a learning + experimentation repository: features evolve quickly, APIs may change, and the main goal is to iterate on design, performance, and debuggability.

## 2) Physics
- Custom rigid body simulation (linear + angular motion)
- Broad-phase collision detection (e.g., sweep-and-prune style approach)
- Narrow-phase collision detection (SAT-based)
- Contact generation with support for multiple contact points
- Sleeping/awake handling to reduce simulation cost in large scenes
- Debug output for contact points and contact normals

## 3) Renderer
- OpenGL-based renderer for drawing scene objects
- Basic material/texture pipeline (engine-side asset handling)
- Debug rendering utilities (colliders, normals, overlays)
- Designed with refactoring in mind: separating render responsibilities from gameplay/physics logic

## 4) Editor
- Editor-style object selection and hover picking (raycast-based workflow)
- In-engine debug visualization toggles (e.g., show colliders / contacts / normals)
- Tools and UI intended to support fast iteration while developing the engine
