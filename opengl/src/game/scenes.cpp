#pragma once

#include "scene_builder.h"

void SceneBuilder::testScene() {
    int floorWidth = 1;
    int floorHeight = 1;

    for (int i = 0; i < floorWidth; i++)
        for (int j = 0; j < floorHeight; j++) {
            glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0f, 0.5f, 0.0f));
            createObject("uvmap", ColliderType::CUBOID, glm::vec3(25 + i * 50, -0.5, 25 + j * 50), glm::vec3(50, 1, 50), 0, 1, orientation);
        }


    glm::quat newOrient = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0f, 0.5f, 0.0f));
    float wWidth = 5.0f;
    float wHeight = 5.0f;
    float wLength = 5.0f;
    glm::vec3 color1 = glm::vec3(255, 0, 0);
    glm::vec3 color2 = glm::vec3(0, 255, 0);
    createObject("plain", ColliderType::MESH, glm::vec3(5,7,5), glm::vec3(wWidth, wHeight, wLength), 0, 0, newOrient, 0, 0, color1);
    createObject("plain", ColliderType::MESH, glm::vec3(9,7,9), glm::vec3(wWidth, wHeight, wLength), 0, 0, newOrient, 0, 0, color2);
}

void SceneBuilder::mainScene() {
    // ___________________________________________________________
    // ------------------------ floor tiles ----------------------
    int floorWidth = 5;
    int floorHeight = 5;

    for (int i = 0; i < floorWidth; i++)
        for (int j = 0; j < floorHeight; j++) {
            glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0f, 0.5f, 0.0f));
            createObject("uvmap", ColliderType::CUBOID, glm::vec3(25 + i * 50, -0.5, 25 + j * 50), glm::vec3(50, 1, 50), 0, 1, orientation);

        }

    // ___________________________________________________________
    // ------------------------ haloA -----------------------------
    float baseRotAng = 90.0f;
    glm::quat baseRot = glm::angleAxis(glm::radians(baseRotAng), glm::vec3(0, 0, 1));

    float wWidth = 50.0f;
    float wHeight = 5.0f;
    float wLength = 90.0f;
    float halfDepth = wLength * 0.5f;

    glm::vec3 desiredCenter = glm::vec3(125, 0, 125);

    glm::vec3 lastPos = glm::vec3(500, 500, 500);
    float lastAngle = 45.0f;
    glm::quat lastOrient = glm::angleAxis(glm::radians(lastAngle), glm::vec3(1, 0, 0));

    for (int i = 0; i < 72; ++i) {
        float newAngle = lastAngle - 5.0f;
        glm::quat newOrientLocal = glm::angleAxis(glm::radians(newAngle), glm::vec3(1, 0, 0));
        glm::vec3 frontTip1 = lastPos + lastOrient * glm::vec3(0, 0, halfDepth);
        glm::vec3 newPosLocal = frontTip1 + newOrientLocal * glm::vec3(0, 0, halfDepth);
        glm::vec3 newPos = baseRot * newPosLocal;
        glm::quat newOrient = baseRot * newOrientLocal;

        //glm::vec3 color = glm::vec3(randomRange(0, 1), randomRange(0, 255), randomRange(0, 255));
        glm::vec3 color = glm::vec3(0, 255, 0);
        createObject("plain", ColliderType::CUBOID, newPos, glm::vec3(wWidth, wHeight, wLength), 0, 1, newOrient, 0, 0, color);

        lastAngle = newAngle;
        lastOrient = newOrientLocal;
        lastPos = newPosLocal;

        haloA.push_back(dynamicObjects.back().id);
    }

    // calculate haloACenter
    glm::vec3 sum = glm::vec3(0.0f);
    for (int i = 0; i < haloA.size(); i++)
        sum += dynamicObjects[haloA[i]].position;
    haloACenter = sum / static_cast<float>(haloA.size());

    for (int i = 0; i < haloA.size(); i++) {
        GameObject& obj = dynamicObjects[haloA[i]];
        glm::vec3 relativePos = desiredCenter - haloACenter;
        obj.position += relativePos;
    }
    haloACenter = desiredCenter;

    // ___________________________________________________________
    // ------------------------ haloB -----------------------------
    baseRotAng = 90.0f;
    baseRot = glm::angleAxis(glm::radians(baseRotAng), glm::vec3(0, 0, 1));

    wWidth = 50.0f;
    wHeight = 5.0f;
    wLength = 100.0f;
    halfDepth = wLength * 0.5f;

    //desiredCenter = glm::vec3(125, 0, 125);

    lastPos = glm::vec3(500, 500, 500);
    lastAngle = 45.0f;
    lastOrient = glm::angleAxis(glm::radians(lastAngle), glm::vec3(1, 0, 0));

    for (int i = 0; i < 72; ++i) {
        float newAngle = lastAngle - 5.0f;
        glm::quat newOrientLocal = glm::angleAxis(glm::radians(newAngle), glm::vec3(1, 0, 0));
        glm::vec3 frontTip1 = lastPos + lastOrient * glm::vec3(0, 0, halfDepth);
        glm::vec3 newPosLocal = frontTip1 + newOrientLocal * glm::vec3(0, 0, halfDepth);
        glm::vec3 newPos = baseRot * newPosLocal;
        glm::quat newOrient = baseRot * newOrientLocal;

        //glm::vec3 color = glm::vec3(randomRange(0, 1), randomRange(0, 255), randomRange(0, 255));
        glm::vec3 color = glm::vec3(255, 0, 0);
        createObject("plain", ColliderType::CUBOID, newPos, glm::vec3(wWidth, wHeight, wLength), 0, 1, newOrient, 0, 0, color);

        lastAngle = newAngle;
        lastOrient = newOrientLocal;
        lastPos = newPosLocal;

        haloB.push_back(dynamicObjects.back().id);
    }

    // calculate haloBCenter
    sum = glm::vec3(0.0f);
    for (int i = 0; i < haloB.size(); i++)
        sum += dynamicObjects[haloB[i]].position;
    haloBCenter = sum / static_cast<float>(haloB.size());

    for (int i = 0; i < haloB.size(); i++) {
        GameObject& obj = dynamicObjects[haloB[i]];
        glm::vec3 relativePos = desiredCenter - haloBCenter;
        obj.position += relativePos;
    }
    haloBCenter = desiredCenter;

    // ___________________________________________________________
    // ------------------------ haloC -----------------------------
    baseRotAng = 90.0f;
    baseRot = glm::angleAxis(glm::radians(baseRotAng), glm::vec3(0, 0, 1));

    wWidth = 50.0f;
    wHeight = 5.0f;
    wLength = 110.0f;
    halfDepth = wLength * 0.5f;

    //desiredCenter = glm::vec3(125, 0, 125);

    lastPos = glm::vec3(500, 500, 500);
    lastAngle = 45.0f;
    lastOrient = glm::angleAxis(glm::radians(lastAngle), glm::vec3(1, 0, 0));

    for (int i = 0; i < 72; ++i) {
        float newAngle = lastAngle - 5.0f;
        glm::quat newOrientLocal = glm::angleAxis(glm::radians(newAngle), glm::vec3(1, 0, 0));
        glm::vec3 frontTip1 = lastPos + lastOrient * glm::vec3(0, 0, halfDepth);
        glm::vec3 newPosLocal = frontTip1 + newOrientLocal * glm::vec3(0, 0, halfDepth);
        glm::vec3 newPos = baseRot * newPosLocal;
        glm::quat newOrient = baseRot * newOrientLocal;

        //glm::vec3 color = glm::vec3(randomRange(0, 1), randomRange(0, 255), randomRange(0, 255));
        glm::vec3 color = glm::vec3(0, 0, 255);
        createObject("plain", ColliderType::CUBOID, newPos, glm::vec3(wWidth, wHeight, wLength), 0, 1, newOrient, 0, 0, color);

        lastAngle = newAngle;
        lastOrient = newOrientLocal;
        lastPos = newPosLocal;

        haloC.push_back(dynamicObjects.back().id);
    }

    // calculate haloCCenter
    sum = glm::vec3(0.0f);
    for (int i = 0; i < haloC.size(); i++)
        sum += dynamicObjects[haloC[i]].position;
    haloCCenter = sum / static_cast<float>(haloC.size());

    for (int i = 0; i < haloC.size(); i++) {
        GameObject& obj = dynamicObjects[haloC[i]];
        glm::vec3 relativePos = desiredCenter - haloCCenter;
        obj.position += relativePos;
    }
    haloCCenter = desiredCenter;

    // ___________________________________________________________
    // ------------------------ bridge-----------------------------
    wWidth = 5.0f;
    wHeight = 0.5f;
    wLength = 20.0f;
    float wDistance = 0.0f;
    halfDepth = wWidth * 0.5f;

    lastPos = glm::vec3(35, 75, 180);
    lastAngle = -90.0f;
    lastOrient = glm::angleAxis(glm::radians(lastAngle), glm::vec3(0, 0, 1));

    for (int i = 0; i < 36; ++i) {
        glm::quat newOrient = glm::angleAxis(glm::radians(lastAngle), glm::vec3(0, 0, 1));
        glm::vec3 frontTip1 = lastPos + lastOrient * glm::vec3(halfDepth, 0, 0);
        glm::vec3 newPos = frontTip1 + newOrient * glm::vec3(halfDepth, 0, 0);

        createObject("plain", ColliderType::CUBOID, newPos, glm::vec3(wWidth, wHeight, wLength), 0, 1, newOrient);
        lastAngle += 5.0f;
        lastOrient = newOrient;
        lastPos = newPos;
    }

    // falling pyramid
    int pHeight = 8;
    int pWidth = 10;
    float sWidth = 1.0f;
    float sLength = 1.0f;
    float sHeight = 1.0f;
    float sDist = 0;
    float pWidthCounter = pWidth;
    for (int y = 0; y < pHeight; y++) {
        for (int x = 0; x < pWidthCounter; x++) {
            for (int z = 0; z < pWidthCounter; z++) {
                float xPos = 140 + x * (sWidth + sDist) + y * (sWidth / 2 + sDist);
                float yPos = 85 + sHeight / 2 + y * sHeight;
                float zPos = 175 + z * (sWidth + sDist) + y * (sWidth / 2 + sDist);

                float xSize;
                float ySize;
                float zSize;
                xSize = sWidth;
                ySize = sHeight;
                zSize = sLength;

                //xSize = randomRange(1, 50);
                //ySize = randomRange(1, 50);
                //zSize = randomRange(1, 50);
                glm::vec3 color = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));

                createObject("plain", ColliderType::CUBOID, glm::vec3(xPos, yPos, zPos), glm::vec3(xSize, ySize, zSize), 1, 0, glm::quat(1, 0, 0, 0), 2.0f, 1, color);
            }
        }
        pWidthCounter -= 1;
    }

    // ___________________________________________________________
    // ------------------------ sloped platforms -----------------
    float slopeLeftX = 100.0f;
    float slopeLeftY = 25.0f;
    float slopeLeftZ = 60.0f;

    float slopeRightX = 100.0f;
    float slopeRightY = 45.0f;
    float slopeRightZ = 80.0f;

    float slopeWidth = 25.0f;
    float slopeHeight = 1.0f;
    float slopeLength = 30.0f;

    float railWidth = slopeHeight;
    float railHeight = 5.0f;
    float railLength = slopeLength;

    float distHeight = 40.0f;
    float angle = 40.0f;

    // left
    for (int i = 0; i < 3; i++) {
        glm::quat orientation = glm::angleAxis(glm::radians(angle), glm::vec3(1.0f, 0.0f, 0.0f));

        // main platform 
        glm::vec3 pos{ slopeLeftX, slopeLeftY + i * distHeight, slopeLeftZ };
        glm::vec3 size{ slopeWidth, slopeHeight, slopeLength };
        createObject("plain", ColliderType::CUBOID, pos, size, 0, 1, orientation);
        // guard rails

        pos = glm::vec3(slopeLeftX + (railWidth / 2) + (slopeWidth / 2), slopeLeftY + slopeHeight + i * distHeight, slopeLeftZ + slopeHeight);
        size = glm::vec3(railWidth, slopeHeight * railHeight, railLength);
        createObject("plain", ColliderType::CUBOID, pos, size, 0, 1, orientation);

        pos = glm::vec3(slopeLeftX - (railWidth / 2) - (slopeWidth / 2), slopeLeftY + slopeHeight + i * distHeight, slopeLeftZ + slopeHeight);
        size = glm::vec3(railWidth, slopeHeight * railHeight, railLength);
        createObject("plain", ColliderType::CUBOID, pos, size, 0, 1, orientation);
    }
    // right
    for (int i = 0; i < 3; i++) {
        glm::quat orientation = glm::angleAxis(glm::radians(-angle), glm::vec3(1.0f, 0.0f, 0.0f));
        // main platform
        glm::vec3 pos{ slopeRightX, slopeRightY + i * distHeight, slopeRightZ };
        glm::vec3 size{ slopeWidth, slopeHeight, slopeLength };
        createObject("plain", ColliderType::CUBOID, pos, size, 0, 1, orientation);

        // guard rails
        pos = glm::vec3(slopeRightX + (railWidth / 2) + (slopeWidth / 2), slopeRightY + slopeHeight + i * distHeight, slopeRightZ - slopeHeight);
        size = glm::vec3(railWidth, slopeHeight * railHeight, railLength);
        createObject("plain", ColliderType::CUBOID, pos, size, 0, 1, orientation);

        pos = glm::vec3(slopeRightX - (railWidth / 2) - (slopeWidth / 2), slopeRightY + slopeHeight + i * distHeight, slopeRightZ - slopeHeight);
        size = glm::vec3(railWidth, slopeHeight * railHeight, railLength);
        createObject("plain", ColliderType::CUBOID, pos, size, 0, 1, orientation);

    }
    // falling pyramid
    pHeight = 8;
    pWidth = 10;
    sWidth = 1.0f;
    sLength = 1.0f;
    sHeight = 1.0f;
    sDist = 0;
    pWidthCounter = pWidth;
    for (int y = 0; y < pHeight; y++) {
        for (int x = 0; x < pWidthCounter; x++) {
            for (int z = 0; z < pWidthCounter; z++) {
                float xPos = 95 + x * (sWidth + sDist) + y * (sWidth / 2 + sDist);
                float yPos = 145 + sHeight / 2 + y * sHeight;
                float zPos = 80 + z * (sWidth + sDist) + y * (sWidth / 2 + sDist);

                float xSize;
                float ySize;
                float zSize;
                xSize = sWidth;
                ySize = sHeight;
                zSize = sLength;

                //xSize = randomRange(1, 50);
                //ySize = randomRange(1, 50);
                //zSize = randomRange(1, 50);
                glm::vec3 color = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));

                createObject("plain", ColliderType::CUBOID, glm::vec3(xPos, yPos, zPos), glm::vec3(xSize, ySize, zSize), 1, 0, glm::quat(1, 0, 0, 0), 2.0f, 1, color);
            }
        }
        pWidthCounter -= 1;
    }

    // ___________________________________________________________
    // ------------------------ ramp -----------------------------
    glm::quat orientation1 = glm::angleAxis(glm::radians(-20.0f), glm::vec3(0.0f, 0.0f, -1.0f));
    createObject("uvmap", ColliderType::CUBOID, glm::vec3(80, 0, 120), glm::vec3(30, 0.5, 30), 0, 1, orientation1);
    GameObject& obj = dynamicObjects.back();
    obj.textureID = 999;

    // ___________________________________________________________
    // ------------------------ slanted platform -----------------
    glm::quat orientation = glm::angleAxis(glm::radians(25.0f), glm::vec3(1.0f, 0.5f, 0.0f));
    createObject("crate", ColliderType::CUBOID, glm::vec3(24.5, 3, 10), glm::vec3(4, 0.2, 4), 0, 1, orientation);

    // ___________________________________________________________
    // ------------------------ catapult -------------------------
    // support
    createObject("crate", ColliderType::CUBOID, glm::vec3(34.5, 1.5, 20), glm::vec3(0.5, 3, 2), 1, 1);

    // plank
    int mass = 50;
    glm::vec3 size = glm::vec3(14, 0.2, 0.5);
    createObject("crate", ColliderType::CUBOID, glm::vec3(34.5, 3.1, 20), size, mass, 0, {}, 999);
    dynamicObjects.back().canMoveLinearly = false;

    // projectile
    createObject("crate", ColliderType::CUBOID, glm::vec3(41.25, 3.45, 20), glm::vec3(0.5), 1, 0, {}, 999);
    // projectile2
    //createObject("crate", glm::vec3(27.75, 3.45, 20), glm::vec3(0.5), 5, 0, {}, 999);
    // counterweight
    createObject("crate", ColliderType::CUBOID, glm::vec3(28.25, 20, 20), glm::vec3(5.2), 10000, 0, {}, 999);

    // ____________________________________________________________
    // ----------------------- box stacks -------------------------
    int amountObjects = 5;
    for (int i = 0; i < amountObjects; i++)
        createObject("crate", ColliderType::CUBOID, glm::vec3(24.5, 1 + (1.2 * i), 24.5), glm::vec3(1), 1, 0, {}, 0.5);
    for (int i = 0; i < amountObjects; i++)
        createObject("crate", ColliderType::CUBOID, glm::vec3(25.5, 1 + (1.2 * i), 24.5), glm::vec3(1), 1, 0, {}, 0.5);
    for (int i = 0; i < amountObjects; i++)
        createObject("crate", ColliderType::CUBOID, glm::vec3(24.5, 1 + (1.2 * i), 25.5), glm::vec3(1), 1, 0, {}, 0.5);
    for (int i = 0; i < amountObjects; i++)
        createObject("crate", ColliderType::CUBOID, glm::vec3(25.5, 1 + (1.2 * i), 25.5), glm::vec3(1), 1, 0, {}, 0.5);


    // ____________________________________________________________
    // ----------------------- brick wall -------------------------
    int wallHeight = 15;
    int wallWidth = 20;
    float brickWidth = 1.0f;
    float brickLength = 1.0f;
    float brickHeight = 0.5f;
    float brickDistance = 0.2f;

    int brickWeight = 10;
    int brickDecrease = 1;

    if (brickWeight < wallHeight) {
        brickWeight = wallHeight;
    }
    int brickWeightStart = brickWeight;
    // col
    for (int col = 0; col < wallHeight / 2; col++) {
        // row 0, 2, 4, 6...
        for (int row = 0; row < wallWidth; row++) {
            float x = 54.5f;
            float y = brickHeight / 2 + col * brickHeight * 2;
            float z = 8 + row * brickLength + brickDistance * row;
            glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
            createObject("plain", ColliderType::CUBOID, glm::vec3(x, y, z), glm::vec3(brickWidth, brickHeight, brickLength), brickWeight, 0, glm::quat(1, 0, 0, 0), 0.5f, 1, randomColor);
        }
        brickWeight -= brickDecrease;
        // row 1, 3, 5, 7...
        for (int row = 0; row < wallWidth - 1; row++) {
            float x = 54.5f;
            float y = brickHeight + brickHeight / 2 + col * brickHeight * 2;
            float z = 8.6f + row * brickLength + brickDistance * row;
            glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
            createObject("plain", ColliderType::CUBOID, glm::vec3(x, y, z), glm::vec3(brickWidth, brickHeight, brickLength), brickWeight, 0, glm::quat(1, 0, 0, 0), 0.5f, 1, randomColor);
        }
        brickWeight -= brickDecrease;
    }

    brickWeight = brickWeightStart;
    if (brickWeight < wallHeight) {
        brickWeight = wallHeight;
    }
    // col
    for (int col = 0; col < wallHeight / 2; col++) {
        // row 0, 2, 4, 6...
        for (int row = 0; row < wallWidth; row++) {
            float x = 55 + row * brickLength + brickDistance * row;
            float y = brickHeight + brickHeight / 2 + col * brickHeight * 2;
            float z = 7;
            glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
            createObject("plain", ColliderType::CUBOID, glm::vec3(x, y, z), glm::vec3(brickWidth, brickHeight, brickLength), brickWeight, 0, glm::quat(1, 0, 0, 0), 0.5f, 1, randomColor);
        }
        brickWeight -= brickDecrease;
        // row 1, 3, 5, 7...
        for (int row = 0; row < wallWidth - 1; row++) {
            float x = 55.6f + row * brickLength + brickDistance * row;
            float y = brickHeight / 2 + col * brickHeight * 2;
            float z = 7;
            glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
            createObject("plain", ColliderType::CUBOID, glm::vec3(x, y, z), glm::vec3(brickWidth, brickHeight, brickLength), brickWeight, 0, glm::quat(1, 0, 0, 0), 0.5f, 1, randomColor);
        }
        brickWeight -= brickDecrease;
    }

    // _____________________________________________________________
    // ----------------------- pyramids ----------------------------
    int numPyramidsX = 1;
    int numPyramidsZ = 1;

    int pyramidHeight = 12;
    int pyramidWidth = 15;

    float stoneWidth = 0.5f;
    float stoneLength = 0.5f;
    float stoneHeight = 3.0f;
    float stoneDistance = 0.0f;

    int stoneWeight = 1;

    // multiple pyramids
    for (int i = 0; i < numPyramidsX; i++) {
        for (int j = 0; j < numPyramidsZ; j++) {
            // single pyramid
            int pyramidWidthCounter = pyramidWidth;
            for (int y = 0; y < pyramidHeight; y++) {
                for (int x = 0; x < pyramidWidthCounter; x++) {
                    for (int z = 0; z < pyramidWidthCounter; z++) {
                        float xPos = i * 15 + 8 + x * (stoneWidth + stoneDistance) + y * (stoneWidth / 2 + stoneDistance);
                        float yPos = stoneHeight / 2 + y * stoneHeight;
                        float zPos = j * 15 + 34.5 + z * (stoneWidth + stoneDistance) + y * (stoneWidth / 2 + stoneDistance);

                        //glm::vec3 color = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
                        glm::vec3 color = glm::vec3(246, 215, 176);

                        createObject("plain", ColliderType::CUBOID, glm::vec3(xPos, yPos, zPos), glm::vec3(stoneWidth, stoneHeight, stoneLength), stoneWeight, 0, glm::quat(1, 0, 0, 0), 0.75f, 1, color);
                    }
                }
                pyramidWidthCounter -= 1;
            }
        }
    }
}