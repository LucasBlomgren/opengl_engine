#pragma once
#include <glad/glad.h>  
#include <GLFW/glfw3.h>

class ShadowManager {
    unsigned int shadow_width; 
    unsigned int shadow_height;
    unsigned int depthMap;
    unsigned int depthMapFBO;

public:
    ShadowManager(unsigned int width, unsigned int height)
        : shadow_width(width), shadow_height(height)
    {
        glGenFramebuffers(1, &depthMapFBO); 

        glGenTextures(1, &depthMap); 
        glBindTexture(GL_TEXTURE_2D, depthMap); 
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, shadow_width, shadow_height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void bind() const;
    void unbind() const;
};