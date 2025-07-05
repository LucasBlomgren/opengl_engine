#include "draw_contact_points.h"

unsigned int setupContactPoint() {
    // Hörn för en kub centrerad i origo med storlek 1
    float halfSize = 0.03f;
    std::vector<float> vertices = {
        // Bakre yta
        -halfSize, -halfSize, -halfSize,  // p1
         halfSize,  halfSize, -halfSize,  // p2
         halfSize, -halfSize, -halfSize,  // p3
         halfSize,  halfSize, -halfSize,  // p4
        -halfSize, -halfSize, -halfSize,  // p5
        -halfSize,  halfSize, -halfSize,  // p6

        // Främre yta
        -halfSize, -halfSize,  halfSize,  // p7
         halfSize, -halfSize,  halfSize,  // p8
         halfSize,  halfSize,  halfSize,  // p9
         halfSize,  halfSize,  halfSize,  // p10
        -halfSize,  halfSize,  halfSize,  // p11
        -halfSize, -halfSize,  halfSize,  // p12

        // Vänster yta
        -halfSize,  halfSize,  halfSize,  // p13
        -halfSize,  halfSize, -halfSize,  // p14
        -halfSize, -halfSize, -halfSize,  // p15
        -halfSize, -halfSize, -halfSize,  // p16
        -halfSize, -halfSize,  halfSize,  // p17
        -halfSize,  halfSize,  halfSize,  // p18

        // Höger yta
         halfSize,  halfSize,  halfSize,  // p19
         halfSize, -halfSize, -halfSize,  // p20
         halfSize,  halfSize, -halfSize,  // p21
         halfSize, -halfSize, -halfSize,  // p22
         halfSize,  halfSize,  halfSize,  // p23
         halfSize, -halfSize,  halfSize,  // p24

         // Botten yta
         -halfSize, -halfSize, -halfSize,  // p25
          halfSize, -halfSize, -halfSize,  // p26
          halfSize, -halfSize,  halfSize,  // p27
          halfSize, -halfSize,  halfSize,  // p28
         -halfSize, -halfSize,  halfSize,  // p29
         -halfSize, -halfSize, -halfSize,  // p30

         // Övre yta
         -halfSize,  halfSize, -halfSize,  // p31
          halfSize,  halfSize,  halfSize,  // p32
          halfSize,  halfSize, -halfSize,  // p33
          halfSize,  halfSize,  halfSize,  // p34
         -halfSize,  halfSize, -halfSize,  // p35
         -halfSize,  halfSize,  halfSize   // p36
    };

    unsigned int VAO, VBO;
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    return VAO;
}

void drawContactPoint(const Shader& shader, unsigned int& VAO, const glm::vec3& contactPoint) 
{
    shader.setInt("debug.objectType", 1);
    shader.setVec3("debug.contactPointOffset", contactPoint);
    shader.setVec3("debug.uColor", glm::vec3(0, 250, 154));
    shader.setBool("debug.useUniformColor", true);

    //glClear(GL_DEPTH_BUFFER_BIT);
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 36);  


}