#include "draw_line.h"

#include <vector>
#include "shader.h"

unsigned int setupLine() {
    // Din kod för att skapa och konfigurera VAO
    unsigned int VAO;
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
    glBindVertexArray(0); // Återställ bindningen
    return VAO;
}

void drawLine(const Shader& shader, unsigned int& VAO, const glm::vec3& lineStart, const glm::vec3& lineEnd, const glm::vec3& color)
{
    // Skicka start- och slutpunkt till shadern
    shader.setVec3("debug.lineStart", lineStart);
    shader.setVec3("debug.lineEnd", lineEnd);

    shader.setBool("debug.useUniformColor", true);
    shader.setVec3("debug.uColor", color);

    // Rendera linjen
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINES, 0, 2);  // GL_LINES används för att rita linjen
    glBindVertexArray(0);
}