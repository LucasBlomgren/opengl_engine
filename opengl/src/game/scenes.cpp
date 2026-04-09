#include "pch.h"
#include "scene_builder.h"

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

void SceneBuilder::emptyFloorScene() {

    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            GameObjectDesc obj;
            // change pos with i and j
            glm::vec3 position = { i * 15.0f, 150, j * 15.0f };
            glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
            glm::vec3 scale = { 5.0f, 5.0f, 5.0f };
            obj.rootTransformHandle = world.createTransform(position, orientation, scale);
            obj.bodyType = BodyType::Dynamic;
            obj.mass = 2.0f;

            SubPartDesc base;
            glm::vec3 positionBase = { 0,0,0 };
            glm::quat orientationBase = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
            glm::vec3 scaleBase{ 1.0f };
            base.localTransformHandle = world.createTransform(positionBase, orientationBase, scaleBase);
            base.meshName = "cube";
            base.textureName = "crate";
            base.shaderName = "default";
            base.colliderType = ColliderType::CUBOID;

            SubPartDesc mid;
            glm::vec3 positionMid = { 0,1,0 };
            glm::quat orientationMid = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
            glm::vec3 scaleMid{ 0.5f };
            mid.localTransformHandle = world.createTransform(positionMid, orientationMid, scaleMid);
            mid.meshName = "sphere";
            mid.textureName = "uvmap";
            mid.shaderName = "default";
            mid.color = { 255, 255, 255 };
            mid.colliderType = ColliderType::SPHERE;

            SubPartDesc top;
            glm::vec3 positionTop = { 0,1.75f,0 };
            glm::quat orientationTop = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
            glm::vec3 scaleTop{ 0.5f };
            top.localTransformHandle = world.createTransform(positionTop, orientationTop, scaleTop);
            top.meshName = "cube";
            top.textureName = "plain";
            top.shaderName = "default";
            // random color
            glm::vec3 randomColor = { static_cast<float>(rand() % 256) / 255.0f, static_cast<float>(rand() % 256) / 255.0f, static_cast<float>(rand() % 256) / 255.0f };
            top.color = randomColor;
            top.colliderType = ColliderType::CUBOID;

            obj.parts.push_back(base);

            GameObjectHandle h = world.createGameObject(obj);
        }
    }



    GameObjectDesc obj2;
    glm::vec3 position2 = { 0,0,0 };
    glm::quat orientation2 = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
    glm::vec3 scale2 = { 100.0f, 10.0f, 100.0f };
    obj2.rootTransformHandle = world.createTransform(position2, orientation2, scale2);
    obj2.bodyType = BodyType::Static;
    obj2.mass = 2.0f;

    SubPartDesc base2;
    glm::vec3 positionBase2 = { 0,-20,0 };
    glm::quat orientationBase2 = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
    glm::vec3 scaleBase2 = { 10.0f, 10.0f, 10.0f };
    base2.localTransformHandle = world.createTransform(positionBase2, orientationBase2, scaleBase2);
    base2.meshName = "cube";
    base2.textureName = "crate";
    base2.colliderType = ColliderType::CUBOID;
    obj2.parts.push_back(base2);
    GameObjectHandle h2 = world.createGameObject(obj2);

    generateFlatTerrain(
        /*offset*/glm::vec3(-50.0, 0.0, -50.0),
        /*gridX=*/144,
        /*gridZ=*/144,
        /*cellSize=*/3.0,
        /*maxHeight=*/30.0
    );

    //int floorWidth = 1;
    //int floorHeight = 1;
    //const float baseX = 0.0f;
    //const float baseZ = 0.0f;
    //for (int i = 0; i < floorWidth; i++) {
    //    for (int j = 0; j < floorHeight; j++) {
    //        glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
    //        world.createGameObject("uvmap", "cube", ColliderType::CUBOID, BodyType::Static, glm::vec3(baseX + i * 50, -0.5, baseZ + j * 50), glm::vec3(50.0, 1.0, 50.0), false, orientation, 0, false, {}, false);
    //    }
    //}

    ////// roof
    ////glm::vec3 pos = glm::vec3(0, 20, 0);
    ////glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
    ////world.createGameObject("uvmap", "cube", ColliderType::CUBOID, pos, glm::vec3(50.0, 1.0, 50.0), 0, 1, orientation, 0, 0, {}, true);

    ////createHalo(
    ////    /*width*/          40.0f,
    ////    /*height*/         3.0f,
    ////    /*length*/         2.0f,
    ////    /*baseRotation*/   glm::vec3(0, 0, 1),
    ////    /*rotDir*/         glm::vec3(0, 1, 0),
    ////    /*rotSpeed*/       0.0f,
    ////    /*pos*/            glm::vec3(125, 0, 125),
    ////    /*segments*/       72,
    ////    /*color*/          glm::vec3(255, 215, 0), // 0–255 färg
    ////    /*createsShadows*/ true,
    ////    /*seeThrough*/     true
    ////);

    ////world.createGameObject("plain", "cube", ColliderType::CUBOID, glm::vec3(0,10,0), glm::vec3(50.0, 30.0, 5.0), 0, 1, {});

    ////for (int i = 0; i < 100; i++) {
    ////    glm::vec3 pos = glm::vec3(-20 + i * 0.40f, 0.25f, -10.0f);
    ////    world.createGameObject("crate", "cube", ColliderType::CUBOID, pos, glm::vec3(0.05f, 0.5f, 0.2f), 1, 0);
    ////}

    //    // -------------------- double brick wall ---------------------
    //int wallHeight = 20;
    //int wallWidth = 20;
    //float brickWidth = 1.0f;
    //float brickLength = 1.0f;
    //float brickHeight = 0.5f;
    //float brickDistance = 0.2f;

    //int brickWeight = 10;
    //int brickDecrease = 1;

    //createBrickWall(glm::vec3(-5, 0, -10), 0, wallWidth, wallHeight, glm::vec3(brickWidth, brickHeight, brickLength), brickDistance, brickWeight, brickDecrease, glm::vec2(0, 255), true);
    //createBrickWall(glm::vec3(-5, 0, -9), 1, wallWidth, wallHeight, glm::vec3(brickWidth, brickHeight, brickLength), brickDistance, brickWeight, brickDecrease, glm::vec2(0, 255), true);
}

void SceneBuilder::testFloorScene() {
    //int floorWidth = 5;
    //int floorHeight = 5;
    //const float baseX = -30.0f;
    //const float baseZ = -30.0f;
    //for (int i = 0; i < floorWidth; i++) {
    //    for (int j = 0; j < floorHeight; j++) {
    //        glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
    //        world.createGameObject("uvmap", "cube", ColliderType::CUBOID, BodyType::Static, glm::vec3(baseX + i * 50, -0.5, baseZ + j * 50), glm::vec3(50.0, 1.0, 50.0), false, orientation, 0, false, {}, false);
    //    }
    //}

    ////// stack of boxes
    ////int amountObjects = 7;
    ////for (int i = 0; i < amountObjects; i++)
    ////    world.createGameObject("crate", "cube", ColliderType::CUBOID, glm::vec3(24.5, 0.5 + (1.0 * i), 34.5), glm::vec3(1.0), 1, 0, glm::quat(1, 0, 0, 0), 2.5);
    ////for (int i = 0; i < amountObjects; i++)
    ////    world.createGameObject("crate", "cube", ColliderType::CUBOID, glm::vec3(25.5, 0.5 + (1.0 * i), 34.5), glm::vec3(1.0), 1, 0, glm::quat(1, 0, 0, 0), 2.5);
    ////for (int i = 0; i < amountObjects; i++)
    ////    world.createGameObject("crate", "cube", ColliderType::CUBOID, glm::vec3(24.5, 0.5 + (1.0 * i), 35.5), glm::vec3(1.0), 1, 0, glm::quat(1, 0, 0, 0), 2.5);
    ////for (int i = 0; i < amountObjects; i++)
    ////    world.createGameObject("crate", "cube", ColliderType::CUBOID, glm::vec3(25.5, 0.5 + (1.0 * i), 35.5), glm::vec3(1.0), 1, 0, glm::quat(1, 0, 0, 0), 2.5);

    ////world.createGameObject("crate", "cube", ColliderType::CUBOID, glm::vec3(22.5, 6.0, 35.0), glm::vec3(1.5), 1, 0, glm::quat(1, 0, 0, 0), 2.5);
    ////GameObject& box = dynamicObjects.back();
    ////box.linearVelocity = glm::vec3(16.7, 0.0, 0.0);

    ////// 2D pyramid of boxes
    ////for (int col = 0; col < 50; col++) {
    ////    for (int row = 50; row - col > 0; row--) {

    ////        float x = 54.5f;
    ////        float y = 0.5f + col;
    ////        float z = 0.5f + row - (col + 0.5f) / 2 + 0.0f * row;
    ////        glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));

    ////        world.createGameObject("plain", "cube", ColliderType::CUBOID, glm::vec3(x, y, z), glm::vec3(1.0), 1, 0, glm::quat(1, 0, 0, 0), 2.5f, 0, randomColor);
    ////    }
    ////}

    //// circular tower of boxes
    //for (int k = 0; k < 7; k++) {
    //    for (int l = 0; l < 7; l++) {
    //        float angleRad;
    //        glm::vec3 axis = glm::normalize(glm::vec3(0.0, 1.0, 0.0));
    //        glm::vec3 center = glm::vec3(-35.0 + k * 25, 0.5, -35.0 + l * 25);
    //        glm::vec3 startPos = center + glm::vec3(0.0, 0.0, 5.5);
    //        glm::quat oldOrientation = glm::quat(1.0, 0.0, 0.0, 0.0);

    //        int height = 10;
    //        for (int i = 0; i < height; i++) {
    //            angleRad = glm::radians(static_cast<float>(i) * 15.0);
    //            for (int j = 0; j < 12; j++) {

    //                glm::quat q = glm::angleAxis(angleRad, glm::normalize(axis));

    //                glm::vec3 offset = startPos - center;   // radien från center till startPos
    //                glm::vec3 rotated = q * offset;         // radien vriden runt axeln
    //                glm::vec3 newCenter = center + rotated; // placera den vridna radien med bas i center

    //                glm::quat orientation = glm::angleAxis(angleRad, glm::normalize(axis));
    //                newCenter.y += i;

    //                glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
    //                world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(newCenter), glm::vec3(2.5, 1.0, 1.0), 100, orientation, 1.5, true, randomColor, false);

    //                angleRad += glm::radians(30.0f);
    //            }
    //        }
    //    }
    //}

    ////// big cube of boxes
    ////int w = 20;
    ////int h = 20;
    ////int d = 10;
    ////int cubeSize = 1;
    ////int spacing = 0.25f;
    ////for (int i = 0; i < w; i++) {
    ////    for (int j = 0; j < d; j++) {
    ////        for (int k = 0; k < h; k++) {

    ////            float x = -15.0f + i * (cubeSize + spacing);
    ////            float y = 0.5f + j * (cubeSize + spacing);
    ////            float z = -15.0f + k * (cubeSize + spacing);
    ////            world.createGameObject("crate", "cube", ColliderType::CUBOID, glm::vec3(x, y, z), glm::vec3(cubeSize), 1, 0, glm::quat(1, 0, 0, 0), 1.0, 1);
    ////        }
    ////    }
    ////}

    ////world.createGameObject("plain", "cube", ColliderType::CUBOID, glm::vec3(-30, 3, -30), glm::vec3(4), 1000, 0, glm::quat(1, 0, 0, 0), 3.5f, 0);

    ////createBlockPyramid("plain", glm::vec3(246.0, 215.0, 176.0), glm::vec3(8.0, 0.0, 74.5), 15, 12, 1.0f, 1.0f, 5.0f, 0, 1.0f, true);
}

//-------------------------
//       Terrain Scene
//-------------------------
void SceneBuilder::terrainScene() {

    ////world.createGameObject("plain", ColliderType::TEAPOT, glm::vec3(40, 120, 40), glm::vec3(1), 100, 0, glm::quat(1,0,0,0), 3.5f, 0);

    //createBlockPyramid("plain", glm::vec3(-1.0), glm::vec3(210.0, 120.0, 225.0), 12, 10, 1.0, 1.0, 1.0, 0.0, 1, true);
    //createBlockPyramid("plain", glm::vec3(-1.0), glm::vec3(250.0, 120.0, 155.0), 12, 10, 1.0, 1.0, 1.0, 0.0, 1, true);

    //createSpherePyramid("plain", glm::vec3(-1.0), glm::vec3(100.0, 120.0, 205.0), 12, 10, 0.5, 0.0, 0.5, true);
    //createSpherePyramid("plain", glm::vec3(-1.0), glm::vec3(100.0, 120.0, 105.0), 12, 10, 0.5, 0.0, 0.5, true);

    //generateFlatTerrain(
    //    /*offset*/glm::vec3(0.0, 0.0, 0.0),
    //    /*gridX=*/144,
    //    /*gridZ=*/144,
    //    /*cellSize=*/3.0,
    //    /*maxHeight=*/120.0
    //);
}

void SceneBuilder::containerScene() {

    //int floorWidth = 1;
    //int floorHeight = 1;

    //const float baseX = 0.0f;
    //const float baseZ = -30.0f;

    //float yOffset = 0.0f;

    //// bottom floor
    //for (int i = 0; i < floorWidth; i++) {
    //    for (int j = 0; j < floorHeight; j++) {
    //        glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0f, 0.5f, 0.0f));
    //        world.createGameObject("uvmap", "cube", ColliderType::CUBOID, BodyType::Static, glm::vec3(baseX + i * 50, yOffset, baseZ + j * 50), glm::vec3(50.0, 1.0, 50.0), 0, orientation, 99, false, {}, false);
    //    }
    //}

    //// ___________________________________________________________
    //// ------------------ walls around floor grid ----------------
    //const float tileSize = 50.0f;
    //const float halfTile = tileSize * 0.5f;
    //const int   w = floorWidth;
    //const int   h = floorHeight;

    //const float wallH = 25.0f;                  // höjd
    //const float thick = 5.0f;                   // tjocklek

    //// world-bounds för golvet
    //const float xMin = baseX - halfTile;
    //const float xMax = baseX + (w - 1) * tileSize + halfTile;
    //const float zMin = baseZ - halfTile;
    //const float zMax = baseZ + (h - 1) * tileSize + halfTile;

    //const float lenX = xMax - xMin + thick * 2;
    //const float lenZ = zMax - zMin + thick * 2;

    //const float y = yOffset + wallH * 0.5f;                // center i Y
    //glm::quat wallOri = glm::quat(1, 0, 0, 0);

    //// syd (zMin)
    //world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static,
    //    glm::vec3((xMin + xMax) * 0.5f, y, zMin - thick * 0.5f),
    //    glm::vec3(lenX, wallH, thick), 0, wallOri, 0, 0, glm::vec3(190, 255, 255), false);

    //    // nord (zMax)
    //    world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static,
    //        glm::vec3((xMin + xMax) * 0.5f, y, zMax + thick * 0.5f),
    //        glm::vec3(lenX, wallH, thick), 0, wallOri, 0, 0, glm::vec3(190, 255, 255), false);

    //        // väst (xMin)
    //        world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static,
    //            glm::vec3(xMin - thick * 0.5f, y, (zMin + zMax) * 0.5f),
    //            glm::vec3(thick, wallH, lenZ), 0, wallOri, 0, 0, glm::vec3(190, 255, 255), false);

    //            // öst (xMax)
    //            world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static,
    //                glm::vec3(xMax + thick * 0.5f, y, (zMin + zMax) * 0.5f),
    //                glm::vec3(thick, wallH, lenZ), 0, wallOri, 0, 0, glm::vec3(190, 255, 255), false);

    //                //// top floor
    //                //for (int i = 0; i < floorWidth; i++) {
    //                //    for (int j = 0; j < floorHeight; j++) {
    //                //        glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0f, 0.5f, 0.0f));
    //                //        world.createGameObject("uvmap", ColliderType::CUBOID, glm::vec3(baseX + i * 50, wallH, baseZ + j * 50), glm::vec3(50, 1, 50), 0, 1, orientation);
    //                //        GameObject& floorTile = dynamicObjects.back();
    //                //        floorTile.seeThrough = true;
    //                //    }
    //                //}

    //world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Kinematic,
    //    glm::vec3(xMax + thick * 0.5f, y+100, (zMin + zMax) * 0.5f),
    //    glm::vec3(thick, wallH, lenZ), 0, wallOri, 0, 0, glm::vec3(190, 255, 255), false);
}


void SceneBuilder::castleScene() {
    //int floorWidth = 4;
    //int floorHeight = 4;

    //const float baseX = -30.0f;
    //const float baseZ = -30.0f;
    //for (int i = 0; i < floorWidth; i++) {
    //    for (int j = 0; j < floorHeight; j++) {
    //        glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
    //        world.createGameObject("uvmap", "cube", ColliderType::CUBOID, BodyType::Static, glm::vec3(baseX + i * 50, -0.5, baseZ + j * 50), glm::vec3(50.0, 1.0, 50.0), false, orientation, 0, false, {}, false);
    //    }
    //}

    //createBrickWall(glm::vec3(25, 0, 44), 0, 20, 21, glm::vec3(1, 1, 1), 0.0f, 1, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(25, 0, 65), 0, 20, 21, glm::vec3(1, 1, 1), 0.0f, 1, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(25, 0, 45), 1, 20, 20, glm::vec3(1, 1, 1), 0.0f, 1, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(45, 0, 45), 1, 20, 20, glm::vec3(1, 1, 1), 0.0f, 1, 0, glm::vec2(95, 110), false);

    //// castle walls
    //for (int i = 0; i < 4; i++) {
    //    // -x, +z
    //    // side of gate walls
    //    createBrickWall(glm::vec3(20, 0, 39 - i), 0, 10, 13, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //    createBrickWall(glm::vec3(38, 0, 39 - i), 0, 10, 13, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //    // over gate wall
    //    createBrickWall(glm::vec3(33, 6, 39 - i), 0, 4, 5, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //    // +x, +z
    //    createBrickWall(glm::vec3(20, 0, 70 + i), 0, 10, 31, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //    // -x, -z
    //    createBrickWall(glm::vec3(19 - i, 0, 40), 1, 10, 30, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //    // +x, -z
    //    createBrickWall(glm::vec3(51 + i, 0, 40), 1, 10, 30, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //}

    //// tower -x, -z
    //createBrickWall(glm::vec3(13, 0, 33), 0, 15, 6, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(13, 0, 39), 0, 15, 6, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(13, 0, 34), 1, 15, 5, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(19, 0, 33), 1, 15, 7, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //// tower +x, -z
    //createBrickWall(glm::vec3(51, 0, 33), 0, 15, 6, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(51, 0, 39), 0, 15, 6, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(51, 0, 34), 1, 15, 5, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(57, 0, 33), 1, 15, 7, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //// tower -x, +z
    //createBrickWall(glm::vec3(13, 0, 70), 0, 15, 6, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(13, 0, 76), 0, 15, 6, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(13, 0, 71), 1, 15, 5, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(19, 0, 70), 1, 15, 7, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //// tower +x, +z
    //createBrickWall(glm::vec3(51, 0, 70), 0, 15, 6, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(51, 0, 76), 0, 15, 6, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(51, 0, 71), 1, 15, 5, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(57, 0, 70), 1, 15, 7, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);



    //for (int i = 0; i < 1; i++) {
    //    GameObjectHandle handle = world.createGameObject("plain", "sphere", ColliderType::SPHERE, BodyType::Dynamic, glm::vec3(55 - i * 2, 40 + i * 2, -90 - i * 10), glm::vec3(0.75f), 150, glm::quat(1, 0, 0, 0), 1, true, {}, false);
    //    GameObject* heavyBox = world.getGameObject(handle);
    //    RigidBody* rb = world.getRigidBody(heavyBox->rigidBodyHandle);
    //    rb->linearVelocity = glm::vec3(0, 0, 50);
    //}
}

//-----------------------------
//    Shape Pile Scene
//-----------------------------
void SceneBuilder::shapePileScene() {
    //int floorWidth = 4;
    //int floorHeight = 4;
    //const float baseX = -30.0f;
    //const float baseZ = -30.0f;
    //for (int i = 0; i < floorWidth; i++) {
    //    for (int j = 0; j < floorHeight; j++) {
    //        glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
    //        world.createGameObject("uvmap", "cube", ColliderType::CUBOID, BodyType::Static, glm::vec3(baseX + i * 50, -0.5, baseZ + j * 50), glm::vec3(50.0, 1.0, 50.0), false, orientation, 0, false, {}, false);
    //    }
    //}

    //int cubeSize = 16;
    //bool sphereLayer = false;
    //glm::vec3 base = glm::vec3(0, 0.5f, 0);
    //for (int i = 0; i < cubeSize; i++) {

    //    if (i % 2 == 0) sphereLayer = true;
    //    else sphereLayer = false;

    //    glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
    //    for (int j = 0; j < cubeSize; j++) {
    //        for (int k = 0; k < cubeSize; k++) {

    //            ColliderType type;
    //            glm::vec3 size;
    //            std::string meshType;
    //            if (sphereLayer) {
    //                type = ColliderType::SPHERE;
    //                size = glm::vec3(0.5f);
    //                meshType = "sphere";
    //            }
    //            else {
    //                type = ColliderType::CUBOID;
    //                size = glm::vec3(1);
    //                meshType = "cube";
    //            }

    //            world.createGameObject("plain", meshType, type, BodyType::Dynamic, glm::vec3(base.x + j, base.y + i, base.z + k), size, 1, glm::quat(1, 0, 0, 0), 1.0f, false, randomColor, false);
    //        }
    //    }
    //}
}

//-----------------------------
//    Tall Structure Scene
//-----------------------------
void SceneBuilder::tallStructureScene() {
    //int floorWidth = 4;
    //int floorHeight = 4;

    //const float baseX = -30.0f;
    //const float baseZ = -30.0f;
    //for (int i = 0; i < floorWidth; i++) {
    //    for (int j = 0; j < floorHeight; j++) {
    //        glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
    //        world.createGameObject("uvmap", "cube", ColliderType::CUBOID, BodyType::Static, glm::vec3(baseX + i * 50, -0.5, baseZ + j * 50), glm::vec3(50.0, 1.0, 50.0), false, orientation, 0, false, {}, false);
    //    }
    //}

    //world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(0.0, -10.0, 0.0), glm::vec3(135.0, 10.0, 135.0), 100000, glm::quat(1, 0, 0, 0), 2.0f, false, {}, false);

    //GameObjectHandle handle = world.createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(-10.0, 5.0, -10.0), glm::vec3(7.2), 1000, glm::quat(1, 0, 0, 0), 2.0f, false, {}, false);
    //GameObject* heavyBox = world.getGameObject(handle);
    //RigidBody* rb = world.getRigidBody(heavyBox->rigidBodyHandle);
    //rb->linearVelocity = glm::vec3(120, 180, 150);

    //// ----- staplar ----- 
    //std::vector<glm::vec3> randomcolors = {
    //glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255)),
    //glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255)),
    //};

    ////std::vector<glm::vec3> randomcolors = {
    ////    glm::vec3(67, 97, 167),
    ////    glm::vec3(244, 244, 107)
    ////};
    //for (int g = 1; g < 6; g++) {
    //    for (int h = 1; h < 6; h++) {

    //        for (int i = 0; i < 17; i++) {
    //            for (int j = 0; j < 2; j++) {
    //                for (int k = 0; k < 2; k++)
    //                {
    //                    glm::vec3 pos(
    //                        (g * 6) + j * 5,
    //                        5 + i * 11,
    //                        (h * 6) + k * 5
    //                    );

    //                    world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Dynamic, pos, glm::vec3(1, 10, 1), 10, glm::quat(1, 0, 0, 0), 1, true, randomcolors[0], false);
    //                }
    //            }

    //            glm::vec3 pos(
    //                (g * 6) + 2.5,
    //                10.5 + i * 11,
    //                (h * 6) + 2.5
    //            );
    //            world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Dynamic, pos, glm::vec3(6, 1, 6), 10, glm::quat(1, 0, 0, 0), 1, true, randomcolors[1], false);
    //        }
    //    }
    //}
}

//---------------------------
//       Tumbler Scene
//---------------------------
void SceneBuilder::tumblerScene() {
    ////createHalo(500.0f, 1.0f, 5.0f, glm::vec3(0, 1, 0), glm::vec3(0, 0, 1), 90.f, glm::vec3(125, 0, 125), 72, glm::vec3(255, 255, 255), true, false);

    //createBlockPyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(120.0f, -30.0f, 165.0f), 12, 10, 1.0f, 1.0f, 1.0f, 0.0f, 1, false);
    ////createBlockPyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(120.0f, -50.0f, 105.0f), 12, 10, 1.0f, 1.0f, 1.0f, 0.0f, 1, false);
    ////createBlockPyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(120.0f, -30.0f, 215.0f), 12, 10, 1.0f, 1.0f, 1.0f, 0.0f, 1, false);
    ////createBlockPyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(120.0f, -50.0f, 55.0f), 12, 10, 1.0f, 1.0f, 1.0f, 0.0f, 1, false);
}

//---------------------------
//         Main Scene
//---------------------------
void SceneBuilder::mainScene() {
    //// ___________________________________________________________
    //// ------------------------ floor tiles ----------------------
    //int floorWidth = 8;
    //int floorHeight = 8;

    //const float baseX = -50.0f;
    //const float baseZ = -50.0f;
    //for (int i = 0; i < floorWidth; i++) {
    //    for (int j = 0; j < floorHeight; j++) {
    //        glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
    //        world.createGameObject("uvmap", "cube", ColliderType::CUBOID, BodyType::Static, glm::vec3(baseX + i * 50, -0.5, baseZ + j * 50), glm::vec3(50.0, 1.0, 50.0), false, orientation, 0, false, {}, false);
    //    }
    //}

    //// ___________________________________________________________
    //// ------------------ walls around floor grid ----------------
    //const float tileSize = 50.0f;
    //const float halfTile = tileSize * 0.5f;
    //const int   w = floorWidth;
    //const int   h = floorHeight;

    //const float wallH = 50.0f;                  // höjd
    //const float thick = 20.0f;                  // tjocklek

    //// world-bounds för golvet
    //const float xMin = baseX - halfTile;
    //const float xMax = baseX + (w - 1) * tileSize + halfTile;
    //const float zMin = baseZ - halfTile;
    //const float zMax = baseZ + (h - 1) * tileSize + halfTile;

    //const float lenX = xMax - xMin + thick * 2;
    //const float lenZ = zMax - zMin + thick * 2;

    //const float y = wallH * 0.5f;               // center i Y
    //glm::quat wallOri = glm::quat(1, 0, 0, 0);

    //// syd (zMin)
    //world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static,
    //    glm::vec3((xMin + xMax) * 0.5f, y, zMin - thick * 0.5f),
    //    glm::vec3(lenX, wallH, thick), 0, wallOri, 0, false, glm::vec3(190, 255, 255), false);

    //// nord (zMax)
    //world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static,
    //    glm::vec3((xMin + xMax) * 0.5f, y, zMax + thick * 0.5f),
    //    glm::vec3(lenX, wallH, thick), 0, wallOri, 0, false, glm::vec3(190, 255, 255), false);

    //// väst (xMin)
    //world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static,
    //    glm::vec3(xMin - thick * 0.5f, y, (zMin + zMax) * 0.5f),
    //    glm::vec3(thick, wallH, lenZ), 0, wallOri, 0, false, glm::vec3(190, 255, 255), false);

    //// öst (xMax)
    //world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static,
    //    glm::vec3(xMax + thick * 0.5f, y, (zMin + zMax) * 0.5f),
    //    glm::vec3(thick, wallH, lenZ), 0, wallOri, 0, false, glm::vec3(190, 255, 255), false);


    //// ___________________________________________________________
    //// ------------------------ bridge----------------------------
    //float wWidth = 5.0f;
    //float wHeight = 0.5f;
    //float wLength = 20.0f;
    //float wDistance = 0.0f;
    //float halfDepth = wWidth * 0.5f;

    //glm::vec3 lastPos = glm::vec3(35, 75, 180);
    //float lastAngle = -90.0f;
    //glm::quat lastOrient = glm::angleAxis(glm::radians(lastAngle), glm::vec3(0, 0, 1));

    //for (int i = 0; i < 37; ++i) {
    //    glm::quat newOrient = glm::angleAxis(glm::radians(lastAngle), glm::vec3(0, 0, 1));
    //    glm::vec3 frontTip1 = lastPos + lastOrient * glm::vec3(halfDepth, 0, 0);
    //    glm::vec3 newPos = frontTip1 + newOrient * glm::vec3(halfDepth, 0, 0);

    //    world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static, newPos, glm::vec3(wWidth, wHeight, wLength), 0, newOrient, 0, false, glm::vec3(255.0f), false);
    //    lastAngle += 5.0f;
    //    lastOrient = newOrient;
    //    lastPos = newPos;
    //}

    //// --- falling pyramid ---
    //// textureName, color, pos, pHeight, pWidth, sWidth, sLength, sHeight, sDistance, sWeight
    ////createBlockPyramid("plain", glm::vec3(-1,-1,-1), glm::vec3(140.0f, 85.0f, 175.0f), 10, 8, 1.0f, 1.0f, 1.0f, 0.0f, 1);
    //createSpherePyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(140.0f, 85.0f, 175.0f), 10, 8, 0.5f, 0.0f, 0.5f, true);

    //// ___________________________________________________________
    //// ------------------------ sloped platforms -----------------
    //float slopeLeftX = 100.0f;
    //float slopeLeftY = 25.0f;
    //float slopeLeftZ = 60.0f;

    //float slopeRightX = 100.0f;
    //float slopeRightY = 45.0f;
    //float slopeRightZ = 80.0f;

    //float slopeWidth = 25.0f;
    //float slopeHeight = 1.0f;
    //float slopeLength = 30.0f;

    //float railWidth = slopeHeight;
    //float railHeight = 5.0f;
    //float railLength = slopeLength;

    //float distHeight = 40.0f;
    //float angle = 40.0f;

    //// left platforms
    //for (int i = 0; i < 3; i++) {
    //    glm::quat orientation = glm::angleAxis(glm::radians(angle), glm::vec3(1.0f, 0.0f, 0.0f));

    //    // main platform 
    //    glm::vec3 pos{ slopeLeftX, slopeLeftY + i * distHeight, slopeLeftZ };
    //    glm::vec3 size{ slopeWidth, slopeHeight, slopeLength };
    //    world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static, pos, size, 0, orientation, 0, false, glm::vec3(255.0f), false);
    //    // guard rails

    //    pos = glm::vec3(slopeLeftX + (railWidth / 2) + (slopeWidth / 2), slopeLeftY + slopeHeight + i * distHeight, slopeLeftZ + slopeHeight);
    //    size = glm::vec3(railWidth, slopeHeight * railHeight, railLength);
    //    world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static, pos, size, 0, orientation, 0, false, glm::vec3(255.0f), false);

    //    pos = glm::vec3(slopeLeftX - (railWidth / 2) - (slopeWidth / 2), slopeLeftY + slopeHeight + i * distHeight, slopeLeftZ + slopeHeight);
    //    size = glm::vec3(railWidth, slopeHeight * railHeight, railLength);
    //    world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static, pos, size, 0, orientation, 0, false, glm::vec3(255.0f), false);
    //}
    //// right platforms
    //for (int i = 0; i < 3; i++) {
    //    glm::quat orientation = glm::angleAxis(glm::radians(-angle), glm::vec3(1.0f, 0.0f, 0.0f));
    //    // main platform
    //    glm::vec3 pos{ slopeRightX, slopeRightY + i * distHeight, slopeRightZ };
    //    glm::vec3 size{ slopeWidth, slopeHeight, slopeLength };
    //    world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static, pos, size, 0, orientation, 0, false, glm::vec3(255.0f), false);

    //    // guard rails
    //    pos = glm::vec3(slopeRightX + (railWidth / 2) + (slopeWidth / 2), slopeRightY + slopeHeight + i * distHeight, slopeRightZ - slopeHeight);
    //    size = glm::vec3(railWidth, slopeHeight * railHeight, railLength);
    //    world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static, pos, size, 0, orientation, 0, false, glm::vec3(255.0f), false);

    //    pos = glm::vec3(slopeRightX - (railWidth / 2) - (slopeWidth / 2), slopeRightY + slopeHeight + i * distHeight, slopeRightZ - slopeHeight);
    //    size = glm::vec3(railWidth, slopeHeight * railHeight, railLength);
    //    world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static, pos, size, 0, orientation, 0, false, glm::vec3(255.0f), false);

    //}
    //// falling pyramid
    //// textureName, color, pos, pHeight, pWidth, sWidth, sLength, sHeight, sDistance, sWeight
    ////createBlockPyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(95.0f, 145.0f, 80.0f), 10, 8, 1.0f, 1.0f, 1.0f, 0.0f, 1, false);
    //createSpherePyramid("plain", glm::vec3(-1, -1, -1), glm::vec3(95.0f, 145.0f, 80.0f), 10, 8, 0.5f, 0.0f, 0.5f, true);

    //// ___________________________________________________________
    //// ------------------------ ramp -----------------------------
    //glm::quat orientation1 = glm::angleAxis(glm::radians(-20.0f), glm::vec3(0.0f, 0.0f, -1.0f));
    //GameObjectHandle handle = world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Static, glm::vec3(80, 0, 120), glm::vec3(30, 0.5, 30), 0, orientation1, 0, false, glm::vec3(255.0f), false);
    //GameObject* obj = world.getGameObject(handle);
    //obj->textureId = 999;

    //// ___________________________________________________________
    //// ------------------------ slanted platform -----------------
    //glm::quat orientation = glm::angleAxis(glm::radians(25.0f), glm::vec3(1.0f, 0.5f, 0.0f));
    //world.createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Static, glm::vec3(24.5, 3, 10), glm::vec3(4, 0.2, 4), 0, orientation, 0, false, {}, false);

    //// ___________________________________________________________
    //// ------------------------ catapult -------------------------
    //// support
    //world.createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Static, glm::vec3(34.5, 1.5, 20), glm::vec3(0.5, 3, 2), 1, {}, 0, false, {}, false);

    //// plank
    //int mass = 10;
    //glm::vec3 size = glm::vec3(14, 0.2, 0.5);
    //handle = world.createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(34.5, 3.1, 20), size, mass, glm::quat(1, 0, 0, 0), 999, false, {}, false);
    //obj = world.getGameObject(handle);
    //RigidBody* rb = world.getRigidBody(obj->rigidBodyHandle);
    //rb->canMoveLinearly = false;

    //// projectile
    //world.createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(41.25, 3.45, 20), glm::vec3(0.5f), 1, glm::quat(1, 0, 0, 0), 1, false, {}, false);
    //// projectile2
    ////world.createGameObject("crate", glm::vec3(27.75, 3.45, 20), glm::vec3(0.5), 5, 0, glm::quat(1,0,0,0), 999);
    //// counterweight
    //world.createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(28.25, 20, 20), glm::vec3(5.2f), 1000, glm::quat(1, 0, 0, 0), 1, false, {}, false);

    //// ____________________________________________________________
    //// ----------------------- box stacks -------------------------
    //int amountObjects = 7;
    //for (int i = 0; i < amountObjects; i++)
    //    world.createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(24.5, 1 + (1.2 * i), 34.5), glm::vec3(1), 1, glm::quat(1, 0, 0, 0), 2.5, false, {}, false);
    //    for (int i = 0; i < amountObjects; i++)
    //        world.createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(25.5, 1 + (1.2 * i), 34.5), glm::vec3(1), 1, glm::quat(1, 0, 0, 0), 2.5, false, {}, false);
    //        for (int i = 0; i < amountObjects; i++)
    //            world.createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(24.5, 1 + (1.2 * i), 35.5), glm::vec3(1), 1, glm::quat(1, 0, 0, 0), 2.5, false, {}, false);
    //            for (int i = 0; i < amountObjects; i++)
    //                world.createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(25.5, 1 + (1.2 * i), 35.5), glm::vec3(1), 1, glm::quat(1, 0, 0, 0), 2.5, false, {}, false);


    //// -------------------- double brick wall ---------------------
    //int wallHeight = 20;
    //int wallWidth = 20;
    //float brickWidth = 1.0f;
    //float brickLength = 1.0f;
    //float brickHeight = 0.5f;
    //float brickDistance = 0.2f;

    //int brickWeight = 10;
    //int brickDecrease = 1;

    //createBrickWall(glm::vec3(75, 0, 7), 0, wallWidth, wallHeight, glm::vec3(brickWidth, brickHeight, brickLength), brickDistance, brickWeight, brickDecrease, glm::vec2(0, 255), true);
    //createBrickWall(glm::vec3(74, 0, 8), 1, wallWidth, wallHeight, glm::vec3(brickWidth, brickHeight, brickLength), brickDistance, brickWeight, brickDecrease, glm::vec2(0, 255), true);

    //// ----------------------- BIG pyramid -------------------------
    //// textureName, color, pos, pHeight, pWidth, sWidth, sLength, sHeight, sDistance, sWeight, asleep
    //createBlockPyramid("plain", glm::vec3(246, 215, 176), glm::vec3(8, 0, 74.5f), 15, 12, 1.0f, 1.0f, 5.0f, 0, 1.0f, true);

    //// ----------------------- brick wall2 -------------------------
    //createBrickWall(glm::vec3(254.5, 0, -1), 1, 10, 100, glm::vec3(1.0, 1.0, 1.0), 0.0, 1, 0, glm::vec2(0, 255), true);

    //world.createGameObject("crate", "cube", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(254.5, 9.6, 4), glm::vec3(5.2), 100000, glm::quat(1, 0, 0, 0), 1, false, {}, false);

    //world.createGameObject("uvmap", "sphere", ColliderType::SPHERE, BodyType::Dynamic, glm::vec3(224.5, 9.0, 30), glm::vec3(4.0), 100000, glm::quat(1, 0, 0, 0), 1, false, {}, false);

    //// 2d pyramid of colored blocks
    //for (int col = 10, test = 0; col > 0; col--, test++) {
    //    for (int row = 0; row < col; row++) {

    //        float x = 104.5f;
    //        float y = test + 0.5f;
    //        float z = 100.5f + row - (col + 0.5f) / 2;
    //        glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));

    //        world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(x, y, z), glm::vec3(1), 1, glm::quat(1, 0, 0, 0), 3.0f, false, randomColor, false);
    //    }
    //}

    //world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(1.25), glm::vec3(2.5), 10, glm::quat(1, 0, 0, 0), 3.5f, false, {}, false);

    //world.createGameObject("plain", "teapot", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(-20, 0, 0), glm::vec3(1), 10, glm::quat(1, 0, 0, 0), 3.5f, false, glm::vec3(255, 255, 255), false);
    //world.createGameObject("crate", "teapot", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(-20, 0, 10), glm::vec3(1), 10, glm::quat(1, 0, 0, 0), 3.5f, false, glm::vec3(255, 0, 255), false);
    //world.createGameObject("uvmap", "teapot", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(-20, 0, 20), glm::vec3(1), 10, glm::quat(1, 0, 0, 0), 3.5f, false, glm::vec3(255, 0, 255), false);

    //world.createGameObject("plain", "pylon", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(-30, 2.0f, 0), glm::vec3(1), 10, glm::quat(1, 0, 0, 0), 3.5f, false, glm::vec3(255, 255, 255), false);
    //world.createGameObject("crate", "pylon", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(-30, 2.0f, 10), glm::vec3(1), 10, glm::quat(1, 0, 0, 0), 3.5f, false, glm::vec3(255, 0, 255), false);
    //world.createGameObject("uvmap", "pylon", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(-30, 2.0f, 20), glm::vec3(1), 10, glm::quat(1, 0, 0, 0), 3.5f, false, glm::vec3(255, 0, 255), false);

    //// grey brick walls
    //createBrickWall(glm::vec3(-10, 0, 194), 0, 50, 21, glm::vec3(1, 1, 1), 0.0f, 1, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(-10, 0, 215), 0, 50, 21, glm::vec3(1, 1, 1), 0.0f, 1, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(-10, 0, 195), 1, 50, 20, glm::vec3(1, 1, 1), 0.0f, 1, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(10, 0, 195), 1, 50, 20, glm::vec3(1, 1, 1), 0.0f, 1, 0, glm::vec2(95, 110), false);


    //orientation = glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    //world.createGameObject("plain", "girl", ColliderType::CUBOID, BodyType::Static, glm::vec3(-20, 0, 0), glm::vec3(0.0135), 1, orientation, 3.5f, false, glm::vec3(255.0f), false);

    //world.createGameObject("plain", "tank", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(-20, 0, 0), glm::vec3(1), 1, orientation, 3.5f, false, glm::vec3(255.0f), false);
}