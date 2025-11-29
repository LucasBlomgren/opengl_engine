#include "pch.h"
#include "shadow_manager.h"

void ShadowManager::bind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
}

void ShadowManager::unbind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}