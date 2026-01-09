#include "pch.h"

namespace Editor
{
// Framebuffer Object for rendering the viewport
class ViewportFBO {
public:
    ViewportFBO(int w, int h) { create(w, h); }
    ~ViewportFBO() { destroy(); }

    void resizeIfNeeded(int w, int h) {
        if (w <= 0 || h <= 0) return;
        if (w == width && h == height) return;
        destroy();
        create(w, h);
    }

    void bind() const {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glViewport(0, 0, width, height);
    }

    void unbind(int windowW, int windowH) const {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, windowW, windowH);
    }

    GLuint fbo = 0;
    GLuint colorTex = 0;
    GLuint depthRbo = 0;
    int width = 0, height = 0;

private:
    void create(int w, int h) {
        width = w; height = h;

        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);

        // Color texture
        glGenTextures(1, &colorTex);
        glBindTexture(GL_TEXTURE_2D, colorTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTex, 0);

        // Depth buffer
        glGenRenderbuffers(1, &depthRbo);
        glBindRenderbuffer(GL_RENDERBUFFER, depthRbo);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depthRbo);

        // Sanity check
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            std::cerr << "[ViewportFBO] Framebuffer is not complete!" << std::endl;
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void destroy() {
        if (depthRbo) glDeleteRenderbuffers(1, &depthRbo);
        if (colorTex) glDeleteTextures(1, &colorTex);
        if (fbo) glDeleteFramebuffers(1, &fbo);
        depthRbo = 0; colorTex = 0; fbo = 0;
    }
    };
}