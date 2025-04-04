#define GLM_ENABLE_EXPERIMENTAL

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/io.hpp>
#include <glm/gtx/component_wise.hpp>
#include <stb/stb_image.h>

#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <algorithm>
#include <unordered_set>
#include <optional>
#include <unordered_map>

#include "camera.h"
#include "shader.h"
#include "mesh.h"
#include "SAT.h"
#include "collisionManifold.h"
#include "drawContactPoints.h"
#include "xyzObject.h"
#include "sweepnprune.h"
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
void createObject(glm::vec3 pos, glm::vec3 size, float mass, bool isStatic, bool floorTexture);
glm::quat rotateCubeWithQuaternion(GLFWwindow* window, glm::quat currentOrientation, float deltaTime);
void createScene();


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

std::vector<Mesh> meshList;
std::vector<Edge> allEdgesX;
std::vector<Edge> allEdgesY;
std::vector<Edge> allEdgesZ;
std::unordered_map<size_t, Contact> contactCache;

bool FPS = 1;
bool paused = false;
bool showAabb;
bool showContactPoints;
bool showCollisionNormal;
bool showNormals;

int objectId = 0;
int amountObjects = 5;
int amountStacks = 1;

float lightStrength = 100.0f;
glm::vec3 lightStartingPos{ 250,180,200 };
glm::vec3 lightPos = lightStartingPos;
Mesh* light = nullptr;

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

    light = new Mesh(-1, cubeVertices, indices, lightPos, glm::vec3(10, 1, 10), 1, 0, 0);
    light->hasGravity = false;

    // create objects
    createScene();

    // main loop
    while (!glfwWindowShouldClose(window))
    {
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
        float num_iterations = 1;
        if (!paused)
        for (int i = 0; i < num_iterations; i++)
        {
            float strength = float(i + 1) / num_iterations;

            // update objects pos etc.
            for (Mesh& object : meshList)
            {
                if (object.id == 1)
                    object.orientation = rotateCubeWithQuaternion(window, object.orientation, deltaTime);

                object.updatePos(num_iterations, deltaTime);
                object.updateAABB();
                object.colliding = false;

                if(!object.isStatic)
                    object.OOBB_shouldUpdate = true;
                if (showNormals) 
                    object.updateOOBB();
            }
            light->updatePos(num_iterations, deltaTime);
            lightPos = light->position;

            // Broad phase
            updateEdgePos(meshList, allEdgesX, allEdgesY, allEdgesZ);

            float varianceX = calculateVariance(allEdgesX);
            float varianceY = calculateVariance(allEdgesY);
            float varianceZ = calculateVariance(allEdgesZ);

            std::vector<Edge>* selectedEdges = findMaxVarianceAxis(varianceX, varianceY, varianceZ, allEdgesX, allEdgesY, allEdgesZ);
            insertionSort(*selectedEdges);
            std::vector<std::pair<int, int>> collisionCouplesList = findOverlap(*selectedEdges);

            int axisOrder;
            if (selectedEdges == &allEdgesX) { axisOrder = 0; }
            else if (selectedEdges == &allEdgesY) { axisOrder = 1; }
            else { axisOrder = 2; }

            for (const std::pair<int, int>& collisionCouple : collisionCouplesList)
            {
                if (meshList[collisionCouple.first].asleep and meshList[collisionCouple.second].asleep)
                    continue;

                Mesh& objA = meshList[collisionCouple.first];
                Mesh& objB = meshList[collisionCouple.second];

                if (checkOtherAxes(axisOrder, objA, objB))
                {
                    objA.updateOOBB(); 
                    objB.updateOOBB();

                    // Narrow phase
                    glm::vec3 collisionNormal;
                    float depth = std::numeric_limits<float>::max();
                    int collisionNormalOwner = 0;
                    if (IntersectPolygons(objA, objB, collisionNormal, depth, collisionNormalOwner))
                    {
                        objA.colliding = true; 
                        objB.colliding = true;
                        objA.collisionPoint = objA.position;
                        objB.collisionPoint = objB.position;

                        glm::vec3 direction = objB.position - objA.position;
                        if (glm::dot(direction, collisionNormal) < 0) 
                            collisionNormal = -collisionNormal;

                        // contactPoints
                        Mesh* objA_ptr = &objA;
                        Mesh* objB_ptr = &objB;

                        Contact contact = createContact(contactCache, objA, objB, collisionNormal, collisionNormalOwner);

                        if (contact.counter == 0) {
                            std::cout << "No contact points found" << std::endl;
                            std::cout << depth << std::endl;
                            continue;
                        }

                        // render normal
                        if (showCollisionNormal) {

                            glm::vec3 lineStart = glm::vec3(200, 60, 200);
                            glm::vec3 lineEnd = lineStart + collisionNormal * 40.0f;

                            // 4. Rita linjen
                            drawLine(shader, VAO_line, lineStart, lineEnd, glm::vec3(1.0f, 0.0f, 0.0f)); // t.ex. röd färgdrawLine(shader, VAO_line, )
                        }

                        //objA.setAsleep(deltaTime);
                        //objB.setAsleep(deltaTime);

                        //if (objA.asleep and objB.asleep)
                        //    continue;

                        std::shuffle(contact.points.begin(), contact.points.begin() + contact.counter, g);

                        // PGS solver
                        int maxIterations = 20;
                        for (int i = 0; i < maxIterations; i++) {
                            bool converged = true;

                            for (int j = 0; j < contact.counter; j++) {
                                ContactPoint& cp = contact.points[j];

                                glm::vec3 relativeVelocity = (objB.linearVelocity + glm::cross(objB.angularVelocity, cp.rB)) -
                                    (objA.linearVelocity + glm::cross(objA.angularVelocity, cp.rA));

                                float normalVelocity = glm::dot(relativeVelocity, contact.normal);

                                // Beräkna impulsen med redan uträkna target bounce velocity
                                float J = -(normalVelocity - cp.targetBounceVelocity);
                                J *= cp.m_eff;

                                // Clamp
                                float temp = cp.accumulatedImpulse;
                                cp.accumulatedImpulse = glm::max(temp + J, 0.0f);
                                float deltaImpulse = cp.accumulatedImpulse - temp;

                                // Add normal to impulse
                                glm::vec3 deltaNormalImpulse = deltaImpulse * contact.normal;

                                // Apply impulses
                                if (glm::length(deltaNormalImpulse) > 1e-6f) { 
                                    objA.linearVelocity -= deltaNormalImpulse * objA.invMass;
                                    objA.angularVelocity -= objA.inverseInertia * glm::cross(cp.rA, deltaNormalImpulse);

                                    objB.linearVelocity += deltaNormalImpulse * objB.invMass;
                                    objB.angularVelocity += objB.inverseInertia * glm::cross(cp.rB, deltaNormalImpulse);

                                    // impulse was changed
                                    converged = false; 
                                }

                                //--------------- Pre-calculate for friction ----------------
                                // Project tangential velocity to the tangent plane
                                float v_t1 = glm::dot(relativeVelocity, cp.t1);
                                float v_t2 = glm::dot(relativeVelocity, cp.t2);
                                // Combine the tangential velocities to a vector
                                float vtMagnitude = glm::sqrt(v_t1 * v_t1 + v_t2 * v_t2);

                                // Beräkna effektiv massa längs cp.t1 och cp.t2 (som tidigare)
                                float k_t1 = (objA.invMass + objB.invMass)
                                    + glm::dot(glm::cross(cp.rA, cp.t1), objA.inverseInertia * glm::cross(cp.rA, cp.t1))
                                    + glm::dot(glm::cross(cp.rB, cp.t1), objB.inverseInertia * glm::cross(cp.rB, cp.t1));
                                float invMassT1 = 1.0f / k_t1;

                                float k_t2 = (objA.invMass + objB.invMass)
                                    + glm::dot(glm::cross(cp.rA, cp.t2), objA.inverseInertia * glm::cross(cp.rA, cp.t2))
                                    + glm::dot(glm::cross(cp.rB, cp.t2), objB.inverseInertia * glm::cross(cp.rB, cp.t2));
                                float invMassT2 = 1.0f / k_t2;

                                // Beräkna preliminära impulser i varje riktning:
                                float J1 = -v_t1 * invMassT1;
                                float J2 = -v_t2 * invMassT2;

                                // ---------- Static friction ----------
                                if (vtMagnitude < 0.5f)
                                {
                                    // Den totala önskade friktionsimpulsen i tangentplanet:
                                    glm::vec3 desiredFrictionImpulse = (J1 * cp.t1) + (J2 * cp.t2);

                                    // Beräkna maximal statisk friktionsimpuls (mu_static * |accumulatedImpulse|)
                                    float mu_static = 0.9f; // Justera beroende på material
                                    float maxStaticImpulse = mu_static * fabs(cp.accumulatedImpulse);

                                    float impulseMag = glm::length(desiredFrictionImpulse);
                                    if (impulseMag > maxStaticImpulse) {
                                        desiredFrictionImpulse *= (maxStaticImpulse / impulseMag);
                                    }

                                    // Applicera den statiska friktionsimpulsen:
                                    objA.linearVelocity -= desiredFrictionImpulse * objA.invMass;
                                    objA.angularVelocity -= objA.inverseInertia * glm::cross(cp.rA, desiredFrictionImpulse);

                                    objB.linearVelocity += desiredFrictionImpulse * objB.invMass;
                                    objB.angularVelocity += objB.inverseInertia * glm::cross(cp.rB, desiredFrictionImpulse);
                                }

                                // ---------- Dynamisk friktion (när kontakt glider) ----------
                                else
                                {
                                    // Uppdatera de ackumulerade friktionsimpulserna (använd samma klampningsmetod som du gör idag)
                                    float oldFrictionImpulse1 = cp.accumulatedFrictionImpulse1;
                                    float newFrictionImpulse1 = oldFrictionImpulse1 + J1;
                                    float oldFrictionImpulse2 = cp.accumulatedFrictionImpulse2;
                                    float newFrictionImpulse2 = oldFrictionImpulse2 + J2;

                                    // Klampar impulsen så att total friktion inte överskrider mu_d * |accumulatedImpulse|
                                    float mu_dynamic = 0.5f;  // Exempelvärde – justera efter material
                                    float maxFriction = mu_dynamic * fabs(cp.accumulatedImpulse);
                                    float lengthSq = newFrictionImpulse1 * newFrictionImpulse1 + newFrictionImpulse2 * newFrictionImpulse2;
                                    float maxFrictionSq = maxFriction * maxFriction;
                                    if (lengthSq > maxFrictionSq) {
                                        float scale = maxFriction / sqrt(lengthSq);
                                        newFrictionImpulse1 *= scale;
                                        newFrictionImpulse2 *= scale;
                                    }

                                    float deltaFrictionImpulse1 = newFrictionImpulse1 - oldFrictionImpulse1;
                                    float deltaFrictionImpulse2 = newFrictionImpulse2 - oldFrictionImpulse2;
                                    cp.accumulatedFrictionImpulse1 = newFrictionImpulse1;
                                    cp.accumulatedFrictionImpulse2 = newFrictionImpulse2;

                                    // Bygg friktionsimpulsen i tangentplanet:
                                    glm::vec3 frictionImpulse = (deltaFrictionImpulse1 * cp.t1) + (deltaFrictionImpulse2 * cp.t2);

                                    // Applicera impulsen:
                                    if (glm::length(frictionImpulse) > 1e-6f) {
                                        objA.linearVelocity -= frictionImpulse * objA.invMass;
                                        objA.angularVelocity -= objA.inverseInertia * glm::cross(cp.rA, frictionImpulse);

                                        objB.linearVelocity += frictionImpulse * objB.invMass;
                                        objB.angularVelocity += objB.inverseInertia * glm::cross(cp.rB, frictionImpulse);

                                        converged = false;
                                    }
                                }

                                // ---------- TWIST FRICTION ----------
                                // Beräkna den relativa rotationshastigheten kring kontaktnormalen
                                // Detta är hur snabbt de roterar relativt varandra kring "n"
                                float relativeAngularSpeed = glm::dot((objB.angularVelocity - objA.angularVelocity), contact.normal);

                                // Beräkna effektiv massa för twist. 
                                // Här använder vi kropparnas inverseInertia (i world space) projicerade på kontaktnormalen.
                                float effectiveMassTwist = 1.0f / (glm::dot(contact.normal, objA.inverseInertia * contact.normal) +
                                    glm::dot(contact.normal, objB.inverseInertia * contact.normal));

                                // Beräkna preliminär twist impulse (i rotationsdomänen)
                                float twistImpulse = -relativeAngularSpeed * effectiveMassTwist;

                                // Klampa twistimpulsen baserat på en twist-friktionskoefficient. 
                                // Ofta används en formel liknande: maxTwistImpulse = mu_twist * fabs(cp.accumulatedImpulse)
                                // där cp.accumulatedImpulse är den totala normala impulsen.
                                float mu_twist = 0.9f; // Exempelvärde – justera efter behov
                                float maxTwistImpulse = mu_twist * fabs(cp.accumulatedImpulse);

                                // Ackumulera twistimpulsen
                                float oldTwistImpulse = cp.accumulatedTwistImpulse;
                                float newTwistImpulse = glm::clamp(oldTwistImpulse + twistImpulse, -maxTwistImpulse, maxTwistImpulse);
                                float deltaTwistImpulse = newTwistImpulse - oldTwistImpulse;
                                cp.accumulatedTwistImpulse = newTwistImpulse;

                                // Den twistimpuls vi applicerar är ett moment (angular impulse) kring kontaktnormalen
                                glm::vec3 twistImpulseVec = deltaTwistImpulse * contact.normal;

                                // Applicera twistimpulsen på kropparnas angulära hastigheter
                                if (glm::length(twistImpulseVec) > 1e-6f) {
                                    objA.angularVelocity -= objA.inverseInertia * twistImpulseVec;
                                    objB.angularVelocity += objB.inverseInertia * twistImpulseVec;
                                    converged = false;
                                }
                            }
                            // Om vi inte har någon impuls att applicera, är vi klara
                            if (converged)
                                break;   
                        }

                        // Bias impulses
                        for (int j = 0; j < contact.counter; j++) {
                            ContactPoint& cp = contact.points[j];
                            float penetrationError = glm::max(cp.depth - 0.01f, 0.0f);
                            float J_bias = cp.m_eff * (0.2f / deltaTime) * penetrationError;
                            glm::vec3 biasImpulseVec = J_bias * contact.normal;

                            objA.biasLinearVelocity -= biasImpulseVec * objA.invMass;
                            objB.biasLinearVelocity += biasImpulseVec * objB.invMass;
                        }
                    }
                }
            }
            int maxFramesWithoutCollision = 3;  // t.ex. behåll upp till 3 frames
            for (auto it = contactCache.begin(); it != contactCache.end(); ) {
                if (!it->second.wasUsedThisFrame) {
                    it->second.framesSinceUsed++;

                    // Ta bort manifold efter X antal frames utan kollisionsmatch
                    if (it->second.framesSinceUsed > maxFramesWithoutCollision) {
                        it = contactCache.erase(it);
                        continue;
                    }
                }
                else {
                    // Nollställ för nästa frame
                    it->second.wasUsedThisFrame = false;
                    it->second.framesSinceUsed = 0;  // Nollställ räknaren vid träff
                }
                ++it;
            }
        }


        shader.setVec3("lightColor", glm::vec3(lightStrength, lightStrength, lightStrength));
        shader.setVec3("lightPos", lightPos);
        shader.setVec3("viewPos", camera.Position);

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

            for (auto& pair : contactCache) {
                Contact& contact = pair.second;
                for (int i = 0; i < contact.counter; i++)
                    drawContactPoint(shader, VAO_contactPoint, contact.points[i].globalCoord);
            }
            shader.setBool("isContactPoint", false);
            //glEnable(GL_DEPTH_TEST);
        }
        draw_xyzObject(shader, VAO_xyz);
        draw_worldFrame(shader, VAO_worldFrame);

        // render light source
        light->setModelMatrix();
        shader.setMat4("model", light->modelMatrix);
        shader.setVec3("uColor", glm::vec3(255,255,255));
        glBindVertexArray(light->VAO);
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

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

void createObject(glm::vec3 pos, glm::vec3 size, float mass, bool isStatic, bool floorTexture)
{
    Mesh object(objectId, cubeVertices, indices, pos, size, mass, isStatic, floorTexture);

    meshList.emplace_back(object);
    objectId++;

    allEdgesX.push_back(object.AABB.Box.min.x);
    allEdgesX.push_back(object.AABB.Box.max.x);
    allEdgesY.push_back(object.AABB.Box.min.y);
    allEdgesY.push_back(object.AABB.Box.max.y);
    allEdgesZ.push_back(object.AABB.Box.min.z);
    allEdgesZ.push_back(object.AABB.Box.max.z);
}

void createScene()
{
    objectId = 0;
    meshList.clear();
    allEdgesX.clear();
    allEdgesY.clear();
    allEdgesZ.clear();
    contactCache.clear();

    // floor
    createObject(glm::vec3(250, -5, 250), glm::vec3(500, 10, 500), 0, 1, 1);

    // creating 100 tiles of floor 
    //for (int i = 0; i < 10; i++)
    //    for (int j = 0; j < 10; j++)
    //        createObject(glm::vec3(250+i*500, -5, 250+j * -500), glm::vec3(500, 10, 500), 0, 1, 1);

    // player controlled box
    createObject(glm::vec3(245, 60, 100), glm::vec3(10, 10, 10), 1, 0, 0);

    // slanted platform
    createObject(glm::vec3(245, 30, 100), glm::vec3(40, 2, 40), 0, 1, 0);
    meshList[objectId-1].orientation = glm::angleAxis(glm::radians(25.0f), glm::vec3(1.0f, 0.5f, 0.0f));

    //---catapult---
    // support
    createObject(glm::vec3(345, 15, 200), glm::vec3(5, 30, 20), 1, 1, 0);
    // plank
    int mass = 50;
    glm::vec3 size = glm::vec3(140, 2, 5);
    createObject(glm::vec3(345, 31, 200), size, mass, 0, 0);
    float I_x = (1.0f / 12.0f) * mass * (size.y * size.y + size.z * size.z);
    float I_y = (1.0f / 12.0f) * mass * (size.x * size.x + size.y * size.y);
    float I_z = (1.0f / 12.0f) * mass * (size.x * size.x + size.z * size.z);

    meshList[objectId-1].inverseInertia = glm::mat3(
        glm::vec3(1.0f / I_x, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f / I_y, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f / I_z)
    );
    // projectile
    createObject(glm::vec3(410, 34.5, 200), glm::vec3(5, 5, 5), 1, 0, 0);
    // counterweight
    createObject(glm::vec3(282.5, 200, 200), glm::vec3(12, 12, 12), 100, 0, 0);

    // box stacks
    for (int j = 0; j < amountStacks; j++)
        for (int i = 0; i < amountObjects; i++)
            createObject(glm::vec3(245 + j * 10.2, (10 * i) + 5, 245), glm::vec3(10, 10, 10), 1, 0, 0);

    // light
    lightPos = lightStartingPos;
    light->position = lightStartingPos;
    light->linearVelocity = glm::vec3(0, 0, 0);
}

void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    //Mesh& obj = meshList[1];
    Mesh& obj = *light;
    float speed = 750;
    if(!paused)
    {
        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
            obj.addForce(glm::vec3(0, 0, 1) * speed);
        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
            obj.addForce(glm::vec3(0, 0, -1) * speed);
        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
            obj.addForce(glm::vec3(1, 0, 0) * speed);
        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
            obj.addForce(glm::vec3(-1, 0, 0) * speed);
        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
            obj.addForce(glm::vec3(0, 1, 0) * speed);
        if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
            obj.addForce(glm::vec3(0, -1, 0) * speed);
        if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) {
            obj.linearVelocity = glm::vec3();
            obj.angularVelocity = glm::vec3();
        }
    }


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
            createScene();
    }
}

void mouseButtonCallback(GLFWwindow * window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS) {
        createObject(camera.Position + camera.Front*30.0f, glm::vec3(10, 10, 10), 1, 0, 0);
        meshList[objectId - 1].linearVelocity = camera.Front * 500.0f;
    }
}

// Uppdaterar en kvaternion-baserad rotation baserat på knapptryck (1–6).
// deltaTime används för att göra rotationen tidsberoende (smoother).
glm::quat rotateCubeWithQuaternion(GLFWwindow* window, glm::quat currentOrientation, float deltaTime)
{
    // Hur fort vi vill rotera (grader per sekund) – justerbart efter behov
    float rotationSpeed = 1.0;
    const float degreesPerSecond = 90.0f; // ex. 90 grader/s
    float angle = glm::radians(degreesPerSecond * rotationSpeed * deltaTime);
    // angle är nu i radianer, och motsvarar "hur många radianer vi roterar detta frame".

    // För att slippa if-satser med +1 / -1, kan vi göra såhär:

    // Rotera runt X-axeln?
    if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS)
    {
        // KEY_1 => -1 kring X
        glm::quat delta = glm::angleAxis(-angle, glm::vec3(1.0f, 0.0f, 0.0f));
        currentOrientation = currentOrientation * delta;
    }
    else if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS)
    {
        // KEY_2 => +1 kring X
        glm::quat delta = glm::angleAxis(+angle, glm::vec3(1.0f, 0.0f, 0.0f));
        currentOrientation = currentOrientation * delta;
    }

    // Rotera runt Y-axeln?
    if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS)
    {
        glm::quat delta = glm::angleAxis(-angle, glm::vec3(0.0f, 1.0f, 0.0f));
        currentOrientation = currentOrientation * delta;
    }
    else if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS)
    {
        glm::quat delta = glm::angleAxis(+angle, glm::vec3(0.0f, 1.0f, 0.0f));
        currentOrientation = currentOrientation * delta;
    }

    // Rotera runt Z-axeln?
    if (glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS)
    {
        glm::quat delta = glm::angleAxis(-angle, glm::vec3(0.0f, 0.0f, 1.0f));
        currentOrientation = currentOrientation * delta;
    }
    else if (glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS)
    {
        glm::quat delta = glm::angleAxis(+angle, glm::vec3(0.0f, 0.0f, 1.0f));
        currentOrientation = currentOrientation * delta;
    }

    // Normalisera för säkerhets skull (undvik drift från enhetskvaternion)
    currentOrientation = glm::normalize(currentOrientation);

    // Returnera den nya orienteringen
    return currentOrientation;
}