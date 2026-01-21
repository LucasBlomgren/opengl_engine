#include "pch.h"
#include "normals_renderer.h"
#include "arrow_renderer.h"

bool NormalsRenderer::sInitialized = false;

void NormalsRenderer::init() {
    if (sInitialized) return;
    sInitialized = true;

    // Flytta så att tail (y=-0.5) hamnar vid y=0
    const glm::mat4 TailToOrigin = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.66f, 0.0f));

    const glm::mat4 YtoX = glm::rotate(glm::mat4(1.0f), -glm::half_pi<float>(), glm::vec3(0, 0, 1)); // Y -> X
    const glm::mat4 YtoZ = glm::rotate(glm::mat4(1.0f), glm::half_pi<float>(), glm::vec3(1, 0, 0));  // Y -> Z

    // VIKTIGT: TailToOrigin först, sedan rotation till axeln
    // (så "flytta längs pilens Y" följer med när vi mappar till X/Z)
    modelX = YtoX * TailToOrigin;
    modelZ = YtoZ * TailToOrigin;
    modelY *= TailToOrigin;
}