#pragma once
#include <glad/glad.h>  
#include <GLFW/glfw3.h>

class ShadowManager {
public:
    ShadowManager(unsigned int width, unsigned int height)
        : width(width), height(height)
    {
        // 1) Skapa FBO
        glGenFramebuffers(1, &depthMapFBO);

        // 2) Skapa depth‐textur
        glGenTextures(1, &depthMap);
        glBindTexture(GL_TEXTURE_2D, depthMap);

        // Viktigt: internformat DEPTH_COMPONENT
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);

        // Filtrering
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Compare‐mode: REF_TO_TEXTURE aktiverar sampler2DShadow
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
        // Compare‐func: LEQUAL eller LESS (beroende på bias och artefakter)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);

        // Wrap: CLAMP_TO_BORDER + border‐color för att undvika falska skuggor
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        float borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

        // 3) Fäst texturen som depth‐attachment på FBO
        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);

        // Ingen färgrendering till denna FBO
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    unsigned int width;
    unsigned int height;

    unsigned int depthMap;
    unsigned int depthMapFBO;

    void bind() const;
    void unbind() const;
};