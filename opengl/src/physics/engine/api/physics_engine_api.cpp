#include "pch.h"

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

namespace physics {

//=====================================
// Construction and ownership
//=====================================
Engine::Engine() : impl(std::make_unique<internal::EngineImpl>()) {}
Engine::~Engine() = default;
Engine::Engine(Engine&&) noexcept = default;
Engine& Engine::operator=(Engine&&) noexcept = default;

//=====================================
// Facade: Initialization and scene lifetime
//=====================================
void Engine::init(FrameTimers* frameTimers) {
    impl->init(frameTimers);
}
void Engine::setupScene(
    const std::vector<Triangle>& terrainTriangles) {
    impl->setupScene(terrainTriangles);
}
void Engine::clear() {
    impl->clear();
}

//=====================================
// Facade: Simulation
//=====================================
void Engine::prepareStepLoop() {
    impl->prepareStepLoop();
}
void Engine::step(float deltaTime, EngineState& engine) {
    impl->step(deltaTime, engine);
}

//=====================================
// Facade: Scene-wide commands
//=====================================
void Engine::sleepAllObjects() {
    impl->submitSleepAllObjects();
}

void Engine::awakenAllObjects() {
    impl->submitAwakenAllObjects();
}

//=====================================
// Facade: Solver configuration
//=====================================
int Engine::getPgsIterations() const {
    return impl->getPgsIterations();
}

void Engine::setPgsIterations(int iterations) {
    impl->setPgsIterations(iterations);
}

}
