#include "pch.h"
#include "scene_builder.h"

void SceneBuilder::testTerrainScene() {

    createBlockPyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(210.0f, 120.0f, 225.0f), 12, 10, 1.0f, 1.0f, 1.0f, 0.0f, 1, true);
    createBlockPyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(250.0f, 120.0f, 155.0f), 12, 10, 1.0f, 1.0f, 1.0f, 0.0f, 1, true);

    createSpherePyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(100.0f, 120.0f, 205.0f), 12, 10, 0.5f, 0.0f, 0.5f, true);
    createSpherePyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(100.0f, 120.0f, 105.0f), 12, 10, 0.5f, 0.0f, 0.5f, true);

    //for (int x = 0; x < 2; x++)
    //    for (int z = 0; z < 2; z++) {
    //        glm::vec3 pos = glm::vec3(150 + x * 25, 100, 150 + z * 25);
    //        createSpherePyramid("plain", glm::vec3(-1, -1, -1), pos, 10, 8, 0.5f, 0.01f, 0.5f, false);
    //    }

    //for (int x = 0; x < 2; x++)
    //for (int z = 0; z < 2; z++) {
    //    glm::vec3 pos = glm::vec3(150 + x * 85, 100 + x*60 + z*30, 150 + z * 85);
    //    createBlockPyramid("plain", glm::vec3(-1, -1, -1), pos, 12, 10, 1.0f, 1.0f, 1.0f, 0.0f, 1, false);
    //}

    generateFlatTerrain(
        /*offset*/glm::vec3(0.0f, 0.0f, 0.0f),
        /*gridX=*/144,
        /*gridZ=*/144,
        /*cellSize=*/3.f,
        /*maxHeight=*/120.0f
    );
}

void SceneBuilder::testFloorScene() {
    int floorWidth = 1;
    int floorHeight = 1;

    for (int i = 0; i < floorWidth; i++) {
        for (int j = 0; j < floorHeight; j++) {
            glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0f, 0.5f, 0.0f));
            createObject("uvmap", ColliderType::CUBOID, glm::vec3(25 + i * 50, -0.5, 25 + j * 50), glm::vec3(50, 1, 50), 0, 1, orientation);
        }
    }

    // staplar
    glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0f, 0.5f, 0.0f)); 

    std::vector<glm::vec3> randomColors = {
        glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255)),
        glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255)),
    };

    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++)
            {
                glm::vec3 pos = glm::vec3(20 + j * 5, 5 + i * 11, 20 + k * 5);
                createObject("plain", ColliderType::CUBOID, pos, glm::vec3(1, 10, 1), 10, 0, orientation, 1, false, randomColors[0]);
            }
        }

        glm::vec3 pos = glm::vec3(22.5, 10.5 + i * 11, 22.5); 
        createObject("plain", ColliderType::CUBOID, pos, glm::vec3(6, 1, 6), 10, 0, orientation, 1, false, randomColors[1]); 
    }

    //createBlockPyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(20.0f, 0.0f, 15.0f), 12, 10, 1.0f, 1.0f, 1.0f, 0.0f, 1, false);
    //createSpherePyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(20.0f, 0.5f, 30.0f), 8, 6, 0.5f, 0.0f, 0.5f, false);

    //for (int i = 0; i < 10; i++) {
    //    createObject("plain", ColliderType::SPHERE, glm::vec3(5, 5+i*2,5), glm::vec3(0.5), 0.5f, 0, orientation);
    //}

    //// box stack
    //for (int i = 0; i < 10; i++) {
    //    createObject("plain", ColliderType::CUBOID, glm::vec3(5.5, 0.5f+i,5.5), glm::vec3(1), 1, 0, orientation, 1);
    //}
}

void SceneBuilder::tumblerScene() {
    createTumbler();

    createSpherePyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(-15.0f, 25.0f, 15.0f), 12, 10, 0.5f, 0.0f, 0.5f, true);
    createSpherePyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(-15.0f, 25.0f, 15.0f), 12, 10, 0.5f, 0.0f, 0.5f, true);

    createSpherePyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(15.0f, 25.0f, -15.0f), 12, 10, 0.5f, 0.0f, 0.5f, true);
    createSpherePyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(15.0f, 25.0f, -15.0f), 12, 10, 0.5f, 0.0f, 0.5f, true);
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
    // ------------------------ Halos ----------------------------
    createHalo(50.0f, 5.0f, 90.0f, glm::vec3(0, 1, 0), 0.05f, glm::vec3(125, 0, 125), 72, glm::vec3(0, 255, 0));
    createHalo(50.0f, 5.0f, 100.0f, glm::vec3(1, 0, 0), 0.05f, glm::vec3(125, 0, 125), 72, glm::vec3(255, 0, 0));
    createHalo(50.0f, 5.0f, 110.0f, glm::vec3(0, 0, 1), 0.05f, glm::vec3(125, 0, 125), 72, glm::vec3(0, 0, 255));


    // ___________________________________________________________
    // ------------------------ bridge-----------------------------
    float wWidth = 5.0f;
    float wHeight = 0.5f;
    float wLength = 20.0f;
    float wDistance = 0.0f;
    float halfDepth = wWidth * 0.5f;

    glm::vec3 lastPos = glm::vec3(35, 75, 180);
    float lastAngle = -90.0f;
    glm::quat lastOrient = glm::angleAxis(glm::radians(lastAngle), glm::vec3(0, 0, 1));

    for (int i = 0; i < 36; ++i) {
        glm::quat newOrient = glm::angleAxis(glm::radians(lastAngle), glm::vec3(0, 0, 1));
        glm::vec3 frontTip1 = lastPos + lastOrient * glm::vec3(halfDepth, 0, 0);
        glm::vec3 newPos = frontTip1 + newOrient * glm::vec3(halfDepth, 0, 0);

        createObject("plain", ColliderType::CUBOID, newPos, glm::vec3(wWidth, wHeight, wLength), 0, 1, newOrient);
        lastAngle += 5.0f;
        lastOrient = newOrient;
        lastPos = newPos;
    }

    // --- falling pyramid ---
    // textureName, color, pos, pHeight, pWidth, sWidth, sLength, sHeight, sDistance, sWeight
    //createBlockPyramid("plain", glm::vec3(-1,-1,-1), glm::vec3(140.0f, 85.0f, 175.0f), 10, 8, 1.0f, 1.0f, 1.0f, 0.0f, 1);
    createSpherePyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(140.0f, 85.0f, 175.0f), 10, 8, 0.5f, 0.0f, 0.5f, true);

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
    // textureName, color, pos, pHeight, pWidth, sWidth, sLength, sHeight, sDistance, sWeight
    //createBlockPyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(95.0f, 145.0f, 80.0f), 10, 8, 1.0f, 1.0f, 1.0f, 0.0f, 1);
    createSpherePyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(95.0f, 145.0f, 80.0f), 10, 8, 0.5f, 0.0f, 0.5f, true);

    // ___________________________________________________________
    // ------------------------ ramp -----------------------------
    glm::quat orientation1 = glm::angleAxis(glm::radians(-20.0f), glm::vec3(0.0f, 0.0f, -1.0f));
    createObject("plain", ColliderType::CUBOID, glm::vec3(80, 0, 120), glm::vec3(30, 0.5, 30), 0, 1, orientation1);
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
    createObject("crate", ColliderType::CUBOID, glm::vec3(28.25, 20, 20), glm::vec3(5.2), 1000, 0, {}, 999);

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
    int wallHeight = 20;
    int wallWidth = 20;
    float brickWidth = 1.0f;
    float brickLength = 1.0f;
    float brickHeight = 1.0f;
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
    // ----------------------- BIG pyramid -------------------------
    // 
    // textureName, color, pos, pHeight, pWidth, sWidth, sLength, sHeight, sDistance, sWeight, asleep
    createBlockPyramid("plain", glm::vec3(246, 215, 176), glm::vec3(8, 0, 34.5f), 15, 12, 0.5f, 0.5f, 3, 0, 0.75f, true);  

    // terrain mesh pyramid
    //createBlockPyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(-100.0f, 30.0f, 45.0f), 10, 8, 1.0f, 1.0f, 1.0f, 0.0f, 1, true); 
    //createSpherePyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(-120.0f, 30.0f, 45.0f), 10, 8, 0.5f, 0.0f, 0.5f, true);

    // sphere
    createObject("plain", ColliderType::SPHERE, glm::vec3(10,10,10), glm::vec3(0.5f), 0.5f, 0, glm::quat(1, 0, 0, 0), 0.5f, 0, glm::vec3(255,255,255));
}