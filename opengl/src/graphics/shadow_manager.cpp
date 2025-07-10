#include "shadow_manager.h"

void ShadowManager::bind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glViewport(0, 0, shadow_width, shadow_height);
}

void ShadowManager::unbind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}