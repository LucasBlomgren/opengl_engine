#define GLM_ENABLE_EXPERIMENTAL

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <stb/stb_image.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/io.hpp>
#include <glm/gtx/component_wise.hpp>


#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <algorithm>
#include <unordered_set>
#include <optional>
#include <unordered_map>

#include "camera.h"
#include "physics.h"
#include "sceneBuilder.h"
#include "shader.h"
#include "mesh.h"
#include "drawContactPoints.h"
#include "xyzObject.h"
#include "worldFrame.h"
#include "vertex.h"
#include "drawLine.h"

// Överlagra operator << för glm::vec3
std::ostream& operator<<(std::ostream& os, const glm::vec3& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouseButtonCallback(GLFWwindow* window, int key, int action, int mods);

// settings
const unsigned int SCR_WIDTH = 1920;
const unsigned int SCR_HEIGHT = 1080;

// camera
Camera camera(glm::vec3(-50.0f, 200.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;	
float lastFrame = 0.0f;

std::vector<Vertex> cubeVertices = {
    {glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,0,-1), glm::vec2(0.0f, 0.0f)},  // FRONT
    {glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,0,-1), glm::vec2(1.0f, 0.0f)},
    {glm::vec3(0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,0,-1), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,0,-1), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(-0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,0,-1), glm::vec2(0.0f, 1.0f)},
    {glm::vec3(0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,0,-1), glm::vec2(0.0f, 0.0f)},

    {glm::vec3(-0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,0,1), glm::vec2(0.0f, 0.0f)},  // BACK
    {glm::vec3(0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,0,1), glm::vec2(1.0f, 0.0f)},
    {glm::vec3(0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,0,1), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,0,1), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(-0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,0,1), glm::vec2(0.0f, 1.0f)},
    {glm::vec3(-0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,0,1), glm::vec2(0.0f, 0.0f)},

    {glm::vec3(-0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(-1,0,0), glm::vec2(1.0f, 0.0f)},  // LEFT
    {glm::vec3(-0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(-1,0,0), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(-1,0,0), glm::vec2(0.0f, 1.0f)},
    {glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(-1,0,0), glm::vec2(0.0f, 1.0f)},
    {glm::vec3(-0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(-1,0,0), glm::vec2(0.0f, 0.0f)},
    {glm::vec3(-0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(-1,0,0), glm::vec2(1.0f, 0.0f)},

    {glm::vec3(0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(1,0,0), glm::vec2(1.0f, 0.0f)},  // RIGHT
    {glm::vec3(0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(1,0,0), glm::vec2(0.0f, 0.0f)},
    {glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(1,0,0), glm::vec2(0.0f, 1.0f)},
    {glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(1,0,0), glm::vec2(0.0f, 1.0f)},
    {glm::vec3(0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(1,0,0), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(1,0,0), glm::vec2(1.0f, 0.0f)},

    {glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,-1,0), glm::vec2(0.0f, 1.0f)},  // BOT
    {glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,-1,0), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,-1,0), glm::vec2(1.0f, 0.0f)},
    {glm::vec3(0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,-1,0), glm::vec2(1.0f, 0.0f)},
    {glm::vec3(-0.5f, -0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,-1,0), glm::vec2(0.0f, 0.0f)},
    {glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,-1,0), glm::vec2(0.0f, 1.0f)},

    {glm::vec3(-0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,1,0), glm::vec2(0.0f, 1.0f)},  // TOP 
    {glm::vec3(-0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,1,0), glm::vec2(0.0f, 0.0f)},
    {glm::vec3(0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,1,0), glm::vec2(1.0f, 0.0f)},
    {glm::vec3(0.5f,  0.5f,  0.5f), glm::vec3(1,1,1), glm::vec3(0,1,0), glm::vec2(1.0f, 0.0f)},
    {glm::vec3(0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,1,0), glm::vec2(1.0f, 1.0f)},
    {glm::vec3(-0.5f,  0.5f, -0.5f), glm::vec3(1,1,1), glm::vec3(0,1,0), glm::vec2(0.0f, 1.0f)}
};
std::vector<unsigned int> indices = {
    0, 1, 2,
    3, 4, 5,
    6, 7, 8,
    9, 10, 11,
    12, 13, 14,
    15, 16, 17,
    18, 19, 20,
    21, 22, 23,
    24, 25, 26,
    27, 28, 29,
    30, 31, 32,
    33, 34, 35
};

bool FPS = 1;
bool paused = false;
bool showAabb;
bool showContactPoints;
bool showCollisionNormal;
bool showNormals;

std::string pressedKey = "-1";

PhysicsEngine physicsEngine;
SceneBuilder sceneBuilder;

std::vector<Mesh> meshList;

int main()
{
    camera.Yaw += 130;
    camera.Pitch -= 30;
    camera.ProcessMouseMovement(0, 0);

    // glfw: initialize and configure
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // glfw window creation
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    auto start_time = std::chrono::high_resolution_clock::now();
    int frames = 0;
    int last_second = 0;

    unsigned int VAO_line = setupLine();
    unsigned int VAO_xyz = setup_xyzObject();
    unsigned int VAO_worldFrame = setup_worldFrame();
    unsigned int VAO_contactPoint = setupContactPoint();

    // create & use shader
    Shader shader("object.vs", "object.fs");

    // texture 1
    unsigned int texture1;
    glGenTextures(1, &texture1);
    glBindTexture(GL_TEXTURE_2D, texture1);
    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // load image, create texture and generate mipmaps
    int width, height, nrChannels;
    stbi_set_flip_vertically_on_load(true); // tell stb_image.h to flip loaded texture's on the y-axis.
    unsigned char* data = stbi_load("crate.jpg", &width, &height, &nrChannels, 4);
    if (data) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    else {
        std::cout << "Failed to load texture" << std::endl;
    }
    stbi_image_free(data);

    // texture 2
    unsigned int texture2;
    glGenTextures(1, &texture2);
    glBindTexture(GL_TEXTURE_2D, texture2);
    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // load image, create texture and generate mipmaps
    stbi_set_flip_vertically_on_load(true); // tell stb_image.h to flip loaded texture's on the y-axis.
    unsigned char* data2 = stbi_load("UV0.png", &width, &height, &nrChannels, 4);
    if (data2) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data2);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    else {
        std::cout << "Failed to load texture" << std::endl;
    }
    stbi_image_free(data2);

    // tell opengl for each sampler to which texture unit it belongs to (only has to be done once)
    shader.setInt("texture1", 0);
    shader.setInt("texture2", 1);

    shader.use();

    std::random_device rd;
    std::mt19937 g(rd());

    // create objects
    //(PhysicsEngine& physicsEngine, std::vector<Mesh>& meshList, std::vector<Vertex>& cubeVertices, std::vector<unsigned int>& indices)
    sceneBuilder.createScene(physicsEngine, meshList, cubeVertices, indices);

    // main loop
    while (!glfwWindowShouldClose(window))
    {
        if (pressedKey == "H") {
            sceneBuilder.createScene(physicsEngine, meshList, cubeVertices, indices);
            pressedKey = "-1";
        }
        if (pressedKey == "Mouse1") {
            sceneBuilder.createObject(physicsEngine, meshList, camera.Position + camera.Front * 30.0f, glm::vec3(10, 10, 10), 1, 0, 0, cubeVertices, indices);
            meshList[sceneBuilder.objectId - 1].linearVelocity = camera.Front * 500.0f;
            pressedKey = "-1";
        }

        auto current_time = std::chrono::high_resolution_clock::now();
        // per-frame time logic
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        // projection matrix 
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 10000.0f);
        shader.setMat4("projection", projection);

        // camera/view transformation
        glm::mat4 view = camera.GetViewMatrix();
        shader.setMat4("view", view);

        // physics step
        if (!paused)
            physicsEngine.step(window, meshList, deltaTime, showNormals, g);
        
        //shader.setVec3("lightColor", glm::vec3(lightStrength, lightStrength, lightStrength));
        //shader.setVec3("lightPos", lightPos);
        //shader.setVec3("viewPos", camera.Position);

        // render
        for (Mesh& object : meshList)
        {
            if (object.floorTexture) {
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, texture2);
            }
            else {
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, texture1);
            }
            object.drawMesh(shader);

            if (showAabb) 
                object.AABB.draw(shader, object.colliding);
            if (showNormals) 
                object.OOBB.drawNormals(shader, VAO_line, object.position);
        }
        if (showContactPoints) {
            //glDisable(GL_DEPTH_TEST);
            shader.setBool("useUniformColor", true);
            shader.setBool("useTexture", false);
            shader.setVec3("uColor", glm::vec3(0, 250, 154));
            shader.setBool("isContactPoint", true);

            //for (auto& pair : contactCache) {
            //    Contact& contact = pair.second;
            //    for (int i = 0; i < contact.counter; i++)
            //        drawContactPoint(shader, VAO_contactPoint, contact.points[i].globalCoord);
            //}
            shader.setBool("isContactPoint", false);
            //glEnable(GL_DEPTH_TEST);
        }
        draw_xyzObject(shader, VAO_xyz);
        draw_worldFrame(shader, VAO_worldFrame);

        // render light source
        //light->setModelMatrix();
        //shader.setMat4("model", light->modelMatrix);
        //shader.setVec3("uColor", glm::vec3(255,255,255));
        //glBindVertexArray(light->VAO);
        //glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

        // cout FPS
        if (FPS) {
            float seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
            if (seconds > last_second)
            {
                last_second = seconds;
                std::cout << "FPS: " << frames << "\n";
                std::cout << "Objects: " << meshList.size() << "\n";
                frames = 0;
            };
            frames++;
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwTerminate();
    return 0;
}

void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    //Mesh& obj = meshList[1];
    //Mesh& obj = *light;
    //float speed = 750;
    //if(!paused)
    //{
    //    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
    //        obj.addForce(glm::vec3(0, 0, 1) * speed);
    //    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
    //        obj.addForce(glm::vec3(0, 0, -1) * speed);
    //    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
    //        obj.addForce(glm::vec3(1, 0, 0) * speed);
    //    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
    //        obj.addForce(glm::vec3(-1, 0, 0) * speed);
    //    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
    //        obj.addForce(glm::vec3(0, 1, 0) * speed);
    //    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
    //        obj.addForce(glm::vec3(0, -1, 0) * speed);
    //    if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) {
    //        obj.linearVelocity = glm::vec3();
    //        obj.angularVelocity = glm::vec3();
    //    }
    //}


    //Mesh& obj = meshList[1];
    //float speed = 0.01;
    //if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
    //    obj.position += glm::vec3(0, 0, 1) * speed;
    //if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
    //    obj.position -= glm::vec3(0, 0, 1) * speed;
    //if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
    //    obj.position += glm::vec3(1, 0, 0) * speed;
    //if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
    //    obj.position -= glm::vec3(1, 0, 0) * speed;
    //if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
    //    obj.position += glm::vec3(0, 1, 0) * speed;
    //if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
    //    obj.position -= glm::vec3(0, 1, 0) * speed;

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
}
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_Q)
            showAabb = !showAabb;
        if (key == GLFW_KEY_E)
            showContactPoints = !showContactPoints;
        if (key == GLFW_KEY_R)
            showNormals = !showNormals;
        if (key == GLFW_KEY_T)
            showCollisionNormal = !showCollisionNormal;

        if (key == GLFW_KEY_G)
            paused = !paused;

        if (key == GLFW_KEY_H)
            pressedKey = 'H';
    }
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS) {
        pressedKey = "Mouse1";
    }
}