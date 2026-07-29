#include "pch.h"
#include "scene_builder.h"

void SceneBuilder::createGridFloor(glm::vec3& offset, glm::vec3& cellSize, int gridWidth, int gridLength) {
    for (int i = 0; i < gridWidth; i++)
    for (int j = 0; j < gridLength; j++) {
        GameObjectDesc floorTile;
        glm::vec3 position = offset + glm::vec3(i * cellSize.x, 0.0f, j * cellSize.z);
        glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(0.0, 0.0, 0.0));
        floorTile.rootTransformHandle = world.createTransform(position, orientation, cellSize);
        floorTile.bodyType = physics::BodyType::Static;

        SubPartDesc part;
        part.localTransformHandle = world.createTransform();
        part.textureName = "uvmap";
        floorTile.parts.push_back(part);
        
        world.createGameObject(floorTile);
    }
}



void SceneBuilder::testFloorScene(int amount) {
    glm::vec3 offset = { -30.0f, -0.5f, -30.0f };
    glm::vec3 cellSize = { 50, 1, 50 };
    createGridFloor(offset, cellSize, 10, 8);

    // circular tower of boxes
    for (int k = 0; k < 5; k++) {
        for (int l = 0; l < 5; l++) {
            float angleRad;
            glm::vec3 axis = glm::normalize(glm::vec3(0.0, 1.0, 0.0));
            glm::vec3 center = glm::vec3(-35.0 + k * 25, 0.5, -35.0 + l * 25);
            glm::vec3 startPos = center + glm::vec3(0.0, 0.0, 5.5);
            glm::quat oldOrientation = glm::quat(1.0, 0.0, 0.0, 0.0);

            int height = 9;
            for (int i = 0; i < height; i++) {
                angleRad = glm::radians(static_cast<float>(i) * 15.0);

                for (int j = 0; j < 12; j++) {

                    glm::quat q = glm::angleAxis(angleRad, glm::normalize(axis));

                    glm::vec3 offset = startPos - center;   // radien från center till startPos
                    glm::vec3 rotated = q * offset;         // radien vriden runt axeln
                    glm::vec3 newCenter = center + rotated; // placera den vridna radien med bas i center

                    glm::quat orientation = glm::angleAxis(angleRad, glm::normalize(axis));
                    newCenter.y += i;

                    glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));

                    GameObjectDesc box;
                    box.name = "Box";
                    box.rootTransformHandle = world.createTransform(newCenter, orientation, glm::vec3(2.5, 1.0, 1.0));
                    box.asleep = true;
                    box.mass = 1.0f;
                    SubPartDesc part;
                    part.localTransformHandle = world.createTransform();
                    part.textureName = "plain";
                    part.color = randomColor / 255.0f;
                    box.parts.push_back(part);
                    world.createGameObject(box);

                    angleRad += glm::radians(30.0f);
                }
            }
        }
    }
//
//    glm::vec3 offsetpos = glm::vec3(150, 0.25, 150);
//
//    for (int j = 0; j < 500; j++) {   // y
//        for (int k = 0; k < 2; k++) {   // z
//            GameObjectDesc plank;
//            plank.name = "Plank";
//
//            float xPos = 0.0f;
//            float zPos = 0.0f;
//
//            glm::quat rotationShortSide =
//                glm::angleAxis(
//                    glm::radians(90.0f),
//                    glm::vec3(1.0f, 0.0f, 0.0f)
//                );
//
//            glm::quat orientation = rotationShortSide;
//
//            if (j % 2 == 0) {
//                glm::quat rotationY =
//                    glm::angleAxis(
//                        glm::radians(90.0f),
//                        glm::vec3(0.0f, 1.0f, 0.0f)
//                    );
//
//                // Först runt kortsidan X, sedan runt world-Y.
//                orientation = rotationY * rotationShortSide;
//
//                xPos = k * 4.0f - 2.0f;
//                zPos = 2.0f;
//            }
//            else {
//                zPos = k * 4.0f;
//            }
//
//            plank.rootTransformHandle = world.createTransform(
//                glm::vec3(
//                    offsetpos.x + xPos,
//                    offsetpos.y + j/2.0f,
//                    offsetpos.z + zPos),
//                orientation,
//                glm::vec3(5.0f, 1.0f, 0.5f)
//            );
//
//            plank.allowSleep = true;
//            plank.sleepCounterThreshold = 2.5f;
//            plank.mass = 50.0f;
//
//            SubPartDesc part;
//            part.name = "MainPart";
//            part.textureName = "crate";
//            part.localTransformHandle = world.createTransform();
//
//            plank.parts.push_back(part);
//
//            world.createGameObject(plank);
//        }
//    }
//
//
//    // single stack of boxes and a single box with velocity
//    {
//        for (int i = 0; i < 2; i++)   // x
//        for (int j = 0; j < 5; j++)   // y
//        for (int k = 0; k < 2; k++) { // z
//            GameObjectDesc box;
//            box.name = "Box";
//            box.rootTransformHandle = world.createTransform(glm::vec3(0 + i * 1.0f, 1.0f + j * 1.5f, 0 + k * 1.0f), glm::quat(), glm::vec3(1.0f,1.0f,1.0f));
//            box.allowSleep = true;
//            box.sleepCounterThreshold = 2.5f;
//            box.mass = 1.0f;
//
//            SubPartDesc part;
//            part.name = "MainPart";
//            part.textureName = "crate";
//            part.localTransformHandle = world.createTransform();
//
//            box.parts.push_back(part);
//
//            world.createGameObject(box);
//        }
//    }
//
//    //GameObjectDesc box;
//    //box.name = "Box";
//    //box.rootTransformHandle = world.createTransform(glm::vec3(-5, 4.5, 0.5), glm::quat(), glm::vec3(1.5f));
//    //box.mass = 1.0f;
//    //SubPartDesc boxPart;
//    //boxPart.localTransformHandle = world.createTransform();
//    //boxPart.textureName = "checker_gray";
//    //boxPart.meshName = "cube";
//    //box.parts.push_back(boxPart);
//
//    //GameObjectHandle ha = world.createGameObject(box);
//    //physicsEngine.setLinearVelocity(...);
//
//    //// create chairs in a grid pattern
//    //for (int i = 0; i < 10; i++)
//    //    for (int j = 0; j < 15; j++)
//    //        for (int k = 0; k < 10; k++)
//    //        {
//    //            GameObjectDesc chair;
//    //            chair.name = "Chair";
//    //            glm::vec3 position = { i * 5.0f, 25 + j * 5.0f, k * 5.0f };
//    //            glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
//    //            glm::vec3 scale{ 2.0f };
//    //            chair.rootTransformHandle = world.createTransform(position, orientation, scale);
//    //            chair.bodyType = physics::BodyType::Dynamic;
//    //            chair.mass = 2.0f;
//
//    //            glm::vec3 color = glm::vec3(
//    //                static_cast<float>(rand()) / RAND_MAX,
//    //                static_cast<float>(rand()) / RAND_MAX,
//    //                static_cast<float>(rand()) / RAND_MAX
//    //            );
//
//    //            // seat
//    //            SubPartDesc seat;
//    //            seat.name = "Seat";
//    //            glm::vec3 positionSeat = { 0,0,0 };
//    //            glm::quat orientationSeat = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
//    //            glm::vec3 scaleSeat{ 1.0f, 0.2f, 1.0f };
//    //            seat.localTransformHandle = world.createTransform(positionSeat, orientationSeat, scaleSeat);
//    //            seat.meshName = "cube";
//    //            seat.textureName = "crate";
//    //            seat.shaderName = "default";
//    //            seat.color = color;
//    //            seat.colliderType = physics::ColliderType::CUBOID;
//    //            chair.parts.push_back(seat);
//
//    //            // backrest
//    //            SubPartDesc backrest;
//    //            backrest.name = "Backrest";
//    //            glm::vec3 positionBackrest = { -0.4f, 0.6f, 0.0f };
//    //            glm::quat orientationBackrest = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
//    //            glm::vec3 scaleBackrest{ 0.2f, 1.0f, 1.0f };
//    //            backrest.localTransformHandle = world.createTransform(positionBackrest, orientationBackrest, scaleBackrest);
//    //            backrest.meshName = "cube";
//    //            backrest.textureName = "crate";
//    //            backrest.shaderName = "default";
//    //            backrest.color = color;
//    //            backrest.colliderType = physics::ColliderType::CUBOID;
//    //            chair.parts.push_back(backrest);
//
//    //            // legs
//    //            std::array<glm::vec3, 4> legPositions{
//    //                glm::vec3(-0.4f, -0.6f, -0.4f),
//    //                glm::vec3(0.4f, -0.6f, -0.4f),
//    //                glm::vec3(-0.4f, -0.6f, 0.4f),
//    //                glm::vec3(0.4f, -0.6f, 0.4f)
//    //            };
//    //            for (int i = 0; i < 4; i++) {
//    //                SubPartDesc leg;
//    //                leg.name = "Leg" + std::to_string(i);
//    //                glm::vec3 positionLeg = legPositions[i];
//    //                glm::quat orientationLeg = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
//    //                glm::vec3 scaleLeg{ 0.2f, 1.0f, 0.2f };
//    //                leg.localTransformHandle = world.createTransform(positionLeg, orientationLeg, scaleLeg);
//    //                leg.meshName = "cube";
//    //                leg.textureName = "crate";
//    //                leg.shaderName = "default";
//    //                leg.color = color;
//    //                leg.colliderType = physics::ColliderType::CUBOID;
//    //                chair.parts.push_back(leg);
//    //            }
//
//    //            GameObjectHandle h = world.createGameObject(chair);
//    //        }
//
//
//    //// create objects in a grid pattern
//    //for (int i = 0; i < 10; i++)
//    //    for (int j = 0; j < 7; j++)
//    //        for (int k = 0; k < 10; k++)
//    //        {
//    //            GameObjectDesc object;
//    //            object.name = "Object";
//    //            glm::vec3 position = { i * 10.0f, 25 + j * 15.0f, k * 10.0f };
//    //            glm::quat orientation = glm::angleAxis(glm::radians(240.0f), glm::vec3(1.0, 0.0, 0.0));
//    //            glm::vec3 scale{ 2.0f };
//    //            object.rootTransformHandle = world.createTransform(position, orientation, scale);
//    //            object.bodyType = physics::BodyType::Dynamic;
//    //            object.mass = 2.0f;
//
//    //            glm::vec3 color = glm::vec3(
//    //                static_cast<float>(rand()) / RAND_MAX,
//    //                static_cast<float>(rand()) / RAND_MAX,
//    //                static_cast<float>(rand()) / RAND_MAX
//    //            );
//
//    //            SubPartDesc part1;
//    //            part1.name = "part1";
//    //            glm::vec3 positionPart1 = { 0,0,0 };
//    //            glm::quat orientationPart1 = glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0, 0.0, 0.0));
//    //            glm::vec3 scalePart1{ 1.0f, 5.0f, 1.0f };
//    //            part1.localTransformHandle = world.createTransform(positionPart1, orientationPart1, scalePart1);
//    //            part1.meshName = "cube";
//    //            part1.textureName = "crate";
//    //            part1.shaderName = "default";
//    //            part1.color = color;
//    //            part1.colliderType = physics::ColliderType::CUBOID;
//    //            object.parts.push_back(part1);
//
//    //            SubPartDesc part2;
//    //            part2.name = "part2";
//    //            glm::vec3 positionPart2 = { 0.0f, 3.0f, 3.0f };
//    //            glm::quat orientationPart2 = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
//    //            glm::vec3 scalePart2{ 1.0f, 5.0f, 1.0f };
//    //            part2.localTransformHandle = world.createTransform(positionPart2, orientationPart2, scalePart2);
//    //            part2.meshName = "cube";
//    //            part2.textureName = "crate";
//    //            part2.shaderName = "default";
//    //            part2.color = color;
//    //            part2.colliderType = physics::ColliderType::CUBOID;
//    //            object.parts.push_back(part2);
//
//    //            GameObjectHandle h = world.createGameObject(object);
//    //        }
//
//    //// 2D pyramid of boxes
//    //{
//    //    for (int col = 0; col < 10; col++) {
//    //        for (int row = 10; row - col > 0; row--) {
//
//    //            float x = 54.5f;
//    //            float y = 1 + col;
//    //            float z = 0.5f + row - (col + 0.5f) / 2 + 0.0f * row;
//    //            glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
//
//    //            GameObjectDesc box;
//    //            box.name = "Box";
//    //            box.rootTransformHandle = world.createTransform(glm::vec3(x, y, z), glm::quat(), glm::vec3(1.0f));
//    //            box.mass = 1.0f;
//
//    //            SubPartDesc part;
//    //            part.name = "MainPart";
//    //            part.localTransformHandle = world.createTransform();
//    //            part.textureName = "plain";
//    //            part.color = randomColor / 255.0f;
//    //            box.parts.push_back(part);
//
//    //            world.createGameObject(box);
//    //        }
//    //    }
//    //}
//
//    //// 2D pyramid of boxes
//    //{
//    //    for (int i = 0; i < 5; i++)
//    //    for (int col = 0; col < 20; col++) {
//    //        for (int row = 20; row - col > 0; row--) {
//
//    //            float x = 54.5f + i * 10.0f;
//    //            float y = 0.5f + col;
//    //            float z = 150.0f + 0.5f + row - (col + 0.5f) / 2 + 0.0f * row;
//    //            glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
//
//    //            GameObjectDesc box;
//    //            box.name = "Box";
//    //            box.rootTransformHandle = world.createTransform(glm::vec3(x, y, z), glm::quat(), glm::vec3(1.0f));
//    //            box.mass = 1.0f;
//    //            box.asleep = true;
//
//    //            SubPartDesc part;
//    //            part.name = "MainPart";
//    //            part.localTransformHandle = world.createTransform();
//    //            part.textureName = "checker_magenta";
//    //            part.color = randomColor / 255.0f;
//    //            box.parts.push_back(part);
//
//    //            world.createGameObject(box);
//    //        }
//    //    }
//    //}
//
//    //GameObjectDesc sphere;
//    //sphere.name = "Sphere";
//    //sphere.rootTransformHandle = world.createTransform(glm::vec3(0, 5.5, 160.5), glm::quat(), glm::vec3(5.0f));
//    //sphere.mass = 10000.0f;
//    //SubPartDesc part;
//    //part.localTransformHandle = world.createTransform();
//    //part.textureName = "checker_gray";
//    //part.meshName = "sphere";
//    //part.colliderType = physics::ColliderType::SPHERE;
//    //sphere.parts.push_back(part);
//
//    //GameObjectHandle h = world.createGameObject (sphere);
//    //physicsEngine.setLinearVelocity(...);
//    //physicsEngine.setAngularVelocity(...);
//
//
//    //createBlockPyramid("plain", glm::vec3(246, 215, 176), glm::vec3(8, 0.0f, 74.5f), 15, 12, 1.0f, 1.0f, 5.0f, 0, 1.0f, true);
//
//
//    //for (int i = 0; i < 1; i++)
//    //for (int j = 0; j < 1; j++)
//    //createBlockPyramid("plain", glm::vec3(-1.0), glm::vec3(30.0 * i, 0.5, 30.0 * j), 10, 8, 1.0, 1.0, 1.0, 0.0, 1, true);
//
//
//
//
    // rotating cylinder made from box subparts + inner shelves/baffles
    glm::vec3 axis = glm::normalize(glm::vec3(0.0f, 0.0f, 1.0f));

    glm::vec3 center = glm::vec3(
        -35.0f + 25.0f,
        100.5f,
        -35.0f + 25.0f
    );

    // ------------------------------------------------------
    // Cylinder settings
    // ------------------------------------------------------
    int wallCount = 12;
    int shelfCount = 4;

    float radius = 50.0f;
    float cylinderDepth = 100.0f;

    float angleRad = glm::radians(15.0f);
    float angleStep = glm::radians(360.0f / static_cast<float>(wallCount));

    // ------------------------------------------------------
    // Auto-sized wall dimensions
    // ------------------------------------------------------
    float wallThickness = 1.0f;

    // Tangential width needed for one wall segment at this radius.
    // tan() is better here than sin() because the wall segment is flat/tangent.
    float wallOverlap = 1.10f;
    float wallWidth = 2.0f * radius * std::tan(angleStep * 0.5f) * wallOverlap;

    // ------------------------------------------------------
    // Auto-sized shelf / baffle dimensions
    // ------------------------------------------------------
    float shelfDepthRatio = 0.15f;   // shelf sticks inward 25% of radius
    float shelfWidthRatio = 0.15f;   // shelf tangential width relative to wall segment width

    float shelfDepth = radius * shelfDepthRatio;
    float shelfWidth = wallWidth * shelfWidthRatio;
    float shelfZ = cylinderDepth;

    // Keep shelf inside the wall.
    // Shelf's local +X is radial, so scale.x = shelfDepth.
    float shelfCenterRadius =
    radius
    - wallThickness * 0.5f
    - shelfDepth * 0.5f;

    // ------------------------------------------------------
    // One root GameObject at the shared cylinder center
    // ------------------------------------------------------
    GameObjectDesc cylinder;
    cylinder.name = "RotatingCylinder";

    cylinder.rootTransformHandle = world.createTransform(
        center,
        glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        glm::vec3(1.0f)
    );

    cylinder.bodyType = physics::BodyType::Kinematic;
    cylinder.asleep = false;
    cylinder.mass = 1.0f;

    // ------------------------------------------------------
    // Add wall parts around the root center
    // ------------------------------------------------------
    for (int j = 0; j < wallCount; j++) {
        glm::quat localRotation = glm::angleAxis(angleRad, axis);

        // Local position relative to cylinder root center
        glm::vec3 localOffset = localRotation * glm::vec3(radius, 0.0f, 0.0f);

        //glm::vec3 randomColor = glm::vec3(
        //    randomRange(0, 255),
        //    randomRange(0, 255),
        //    randomRange(0, 255)
        //);

        glm::vec3 randomColor = glm::vec3(200, 200, 200);

        SubPartDesc part;

        part.localTransformHandle = world.createTransform(
            localOffset,
            localRotation,
            glm::vec3(wallThickness, wallWidth, cylinderDepth)
        );

        part.textureName = "plain";
        part.color = randomColor / 255.0f;

        cylinder.parts.push_back(part);

        angleRad += angleStep;
    }

    // ------------------------------------------------------
    // Add inner shelves / baffles
    // ------------------------------------------------------
    for (int i = 0; i < shelfCount; i++) {
        float shelfAngle =
            glm::radians(360.0f * static_cast<float>(i) / static_cast<float>(shelfCount));

        glm::quat shelfRotation = glm::angleAxis(shelfAngle, axis);

        glm::vec3 shelfOffset =
            shelfRotation * glm::vec3(shelfCenterRadius+2.0f, 0.0f, 0.0f);

        SubPartDesc shelf;

        shelf.localTransformHandle = world.createTransform(
            shelfOffset,
            shelfRotation,
            glm::vec3(shelfDepth, shelfWidth, shelfZ)
        );

        shelf.textureName = "plain";
        shelf.color = glm::vec3(0.9f, 0.9f, 0.2f);

        cylinder.parts.push_back(shelf);
    }

    // ------------------------------------------------------
    // Create one GameObject and spin the whole thing
    // ------------------------------------------------------
    GameObjectHandle h = world.createGameObject(cylinder);

    if (GameObject* object = world.getGameObject(h)) {
        physicsEngine.setAngularVelocity(
            object->rigidBodyHandle,
            glm::vec3(0.0f, 0.0f, -0.3f)
        );
    }




    GameObjectDesc tumblerFrontWall;
    tumblerFrontWall.name = "TumblerFrontWall";
    tumblerFrontWall.rootTransformHandle = world.createTransform(
        center - glm::vec3(0.0f, 0.0f, shelfZ * 0.5f + wallThickness * 0.5f),
        glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        glm::vec3(radius * 2.0f, radius * 2.0f, wallThickness)
    );
    tumblerFrontWall.bodyType = physics::BodyType::Static;
    SubPartDesc frontWallPart;
    frontWallPart.seeThrough = true;
    frontWallPart.localTransformHandle = world.createTransform();
    frontWallPart.textureName = "plain";
    frontWallPart.color = glm::vec3(0.8f, 0.8f, 0.8f);
    tumblerFrontWall.parts.push_back(frontWallPart);
    world.createGameObject(tumblerFrontWall);

    GameObjectDesc tumblerBackWall;
    tumblerBackWall.name = "TumblerBackWall";
    tumblerBackWall.rootTransformHandle = world.createTransform(
        center + glm::vec3(0.0f, 0.0f, shelfZ * 0.5f + wallThickness * 0.5f),
        glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        glm::vec3(radius * 2.0f, radius * 2.0f, wallThickness)
    );
    tumblerBackWall.bodyType = physics::BodyType::Static;
    SubPartDesc backWallPart;
    backWallPart.localTransformHandle = world.createTransform();
    backWallPart.textureName = "plain";
    backWallPart.color = glm::vec3(0.8f, 0.8f, 0.8f);
    tumblerBackWall.parts.push_back(backWallPart);
    world.createGameObject(tumblerBackWall);
}

void SceneBuilder::terrainScene() {
    glm::vec3 offset = { -50, -110, -50 };
    glm::vec3 cellSize = { 50, 1, 50 };
    //createGridFloor(offset, cellSize, 10, 10);


    //{
    //    GameObjectDesc forkLift;
    //    glm::vec3 position = { 0, 0, 0 };
    //    glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
    //    glm::vec3 scale{ 3.0f };
    //    forkLift.rootTransformHandle = world.createTransform(position, orientation, scale);
    //    forkLift.bodyType = physics::BodyType::Dynamic;
    //    forkLift.mass = 10.0f;

    //    // body
    //    SubPartDesc body;
    //    glm::vec3 positionBody = { 0,0,0 };
    //    glm::quat orientationBody = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
    //    glm::vec3 scaleBody{ 1.0f, 1.5f, 1.5f };
    //    body.localTransformHandle = world.createTransform(positionBody, orientationBody, scaleBody);
    //    body.textureName = "uvmap";
    //    forkLift.parts.push_back(body);

    //    // forks
    //    glm::vec3 scaleFork{ 0.2f, 0.1f, 1.25f };
    //    glm::quat orientationFork = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
    //    float posForkX = positionBody.x;
    //    float posForkY = positionBody.y + (-scaleBody.y / 2) + (scaleFork.y / 2);
    //    float posForkZ = positionBody.z + (scaleBody.z / 2) + (scaleFork.z / 2);

    //    SubPartDesc fork1;
    //    float posForkX1 = posForkX - (scaleBody.x / 2) + (scaleFork.x / 2) + (scaleBody.x / 8);
    //    glm::vec3 positionFork1 = { posForkX1, posForkY, posForkZ};
    //    fork1.localTransformHandle = world.createTransform(positionFork1, orientationFork, scaleFork);
    //    fork1.textureName = "uvmap";
    //    forkLift.parts.push_back(fork1);

    //    SubPartDesc fork2;
    //    float posForkX2 = posForkX + (scaleBody.x / 2) - (scaleFork.x / 2) - (scaleBody.x / 8);
    //    glm::vec3 positionFork2 = { posForkX2, posForkY, posForkZ };
    //    fork2.localTransformHandle = world.createTransform(positionFork2, orientationFork, scaleFork);
    //    fork2.textureName = "uvmap";
    //    forkLift.parts.push_back(fork2);

    //    world.createGameObject(forkLift);
    //}



    //// create spheres in a grid pattern
    //{
    //    for (int i = 0; i < 60; i++)
    //    for (int j = 0; j < 1; j++)
    //    for (int k = 0; k < 60; k++)
    //    {
    //        GameObjectDesc sphere;
    //        sphere.name = "Sphere";
    //        glm::vec3 position = { i * 5.0f, 25 + j * 5.0f, k * 5.0f };
    //        sphere.rootTransformHandle = world.createTransform(position);
    //        SubPartDesc part;
    //        part.name = "MainPart";
    //        part.localTransformHandle = world.createTransform();
    //        part.colliderType = physics::ColliderType::SPHERE;
    //        part.meshName = "sphere";
    //        part.textureName = "plain";
    //        sphere.parts.push_back(part);
    //        world.createGameObject(sphere);
    //    }
    //}

       
    // create chairs in a grid pattern
    for (int i = 0; i < 5; i++) 
    for (int j = 0; j < 18; j++)
    for (int k = 0; k < 5; k++)
    {
        GameObjectDesc chair;
        chair.name = "Chair";
        glm::vec3 position = { i * 5.0f, 75 + j * 5.0f, k * 5.0f };
        glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
        glm::vec3 scale{ 2.0f };
        chair.rootTransformHandle = world.createTransform(position, orientation, scale);
        chair.bodyType = physics::BodyType::Dynamic;
        chair.mass = 2.0f;

        glm::vec3 color = glm::vec3(
            static_cast<float>(rand()) / RAND_MAX,
            static_cast<float>(rand()) / RAND_MAX,
            static_cast<float>(rand()) / RAND_MAX
        );

        // seat
        SubPartDesc seat;
        seat.name = "Seat";
        glm::vec3 positionSeat = { 0,0,0 };
        glm::quat orientationSeat = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
        glm::vec3 scaleSeat{ 1.0f, 0.2f, 1.0f };
        seat.localTransformHandle = world.createTransform(positionSeat, orientationSeat, scaleSeat);
        seat.meshName = "cube";
        seat.textureName = "checker_magenta";
        seat.shaderName = "default";
        seat.color = color;
        seat.colliderType = physics::ColliderType::CUBOID;
        chair.parts.push_back(seat);

        // backrest
        SubPartDesc backrest;
        backrest.name = "Backrest";
        glm::vec3 positionBackrest = { -0.4f, 0.6f, 0.0f };
        glm::quat orientationBackrest = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
        glm::vec3 scaleBackrest{ 0.2f, 1.0f, 1.0f };
        backrest.localTransformHandle = world.createTransform(positionBackrest, orientationBackrest, scaleBackrest);
        backrest.meshName = "cube";
        backrest.textureName = "checker_magenta";
        backrest.shaderName = "default";
        backrest.color = color;
        backrest.colliderType = physics::ColliderType::CUBOID;
        chair.parts.push_back(backrest);

        // legs
        std::array<glm::vec3, 4> legPositions{
            glm::vec3(-0.4f, -0.6f, -0.4f),
            glm::vec3(0.4f, -0.6f, -0.4f),
            glm::vec3(-0.4f, -0.6f, 0.4f),
            glm::vec3(0.4f, -0.6f, 0.4f)
        };
        for (int i = 0; i < 4; i++) {
            SubPartDesc leg;
            leg.name = "Leg" + std::to_string(i);
            glm::vec3 positionLeg = legPositions[i];
            glm::quat orientationLeg = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
            glm::vec3 scaleLeg{ 0.2f, 1.0f, 0.2f };
            leg.localTransformHandle = world.createTransform(positionLeg, orientationLeg, scaleLeg);
            leg.meshName = "cube";
            leg.textureName = "checker_magenta";
            leg.shaderName = "default";
            leg.color = color;
            leg.colliderType = physics::ColliderType::CUBOID;
            chair.parts.push_back(leg);
        }

        GameObjectHandle h = world.createGameObject(chair);
    }


    //// cube of cubes
    //for (int i = 0; i < 100; i++)
    //for (int j = 0; j < 100; j++)
    //for (int k = 0; k < 100; k++)
    //{
    //    GameObjectDesc cube;
    //    //glm::vec3 position = { i * 2.0f, 25 + j * 2.0f, k * 2.0f };
    //    glm::vec3 position = { i * 10.0f, j * 10.0f, k * 10.0f };
    //    glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
    //    glm::vec3 scale{ 2.0f };
    //    cube.rootTransformHandle = world.createTransform(position, orientation, scale);
    //    cube.bodyType = physics::BodyType::Dynamic;
    //    cube.mass = 2.0f;
    //    cube.asleep = true;

    //    SubPartDesc cubePart;
    //    glm::vec3 positionPart = { 0,0,0 };
    //    glm::quat orientationPart = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
    //    glm::vec3 scalePart{ 1.0f };
    //    cubePart.localTransformHandle = world.createTransform(positionPart, orientationPart, scalePart);
    //    cubePart.meshName = "cube";
    //    cubePart.textureName = "crate";
    //    cubePart.colliderType = physics::ColliderType::CUBOID;
    //    cube.parts.push_back(cubePart);

    //    GameObjectHandle h = world.createGameObject(cube);
    //}



    generateFlatTerrain(
        /*offset*/glm::vec3(-50.0, -90.0, -50.0),
        /*gridX=*/344,
        /*gridZ=*/344,
        /*cellSize=*/3.0,
        /*maxHeight=*/240.0
    );


    ////// stack of boxes
    ////int amountObjects = 7;
    ////for (int i = 0; i < amountObjects; i++)
    ////    world.createGameObject("crate", "cube", physics::ColliderType::CUBOID, glm::vec3(24.5, 0.5 + (1.0 * i), 34.5), glm::vec3(1.0), 1, 0, glm::quat(1, 0, 0, 0), 2.5);
    ////for (int i = 0; i < amountObjects; i++)
    ////    world.createGameObject("crate", "cube", physics::ColliderType::CUBOID, glm::vec3(25.5, 0.5 + (1.0 * i), 34.5), glm::vec3(1.0), 1, 0, glm::quat(1, 0, 0, 0), 2.5);
    ////for (int i = 0; i < amountObjects; i++)
    ////    world.createGameObject("crate", "cube", physics::ColliderType::CUBOID, glm::vec3(24.5, 0.5 + (1.0 * i), 35.5), glm::vec3(1.0), 1, 0, glm::quat(1, 0, 0, 0), 2.5);
    ////for (int i = 0; i < amountObjects; i++)
    ////    world.createGameObject("crate", "cube", physics::ColliderType::CUBOID, glm::vec3(25.5, 0.5 + (1.0 * i), 35.5), glm::vec3(1.0), 1, 0, glm::quat(1, 0, 0, 0), 2.5);

    ////world.createGameObject("crate", "cube", physics::ColliderType::CUBOID, glm::vec3(22.5, 6.0, 35.0), glm::vec3(1.5), 1, 0, glm::quat(1, 0, 0, 0), 2.5);
    ////GameObject& box = dynamicObjects.back();
    ////box.linearVelocity = glm::vec3(16.7, 0.0, 0.0);

    ////// 2D pyramid of boxes
    ////for (int col = 0; col < 50; col++) {
    ////    for (int row = 50; row - col > 0; row--) {

    ////        float x = 54.5f;
    ////        float y = 0.5f + col;
    ////        float z = 0.5f + row - (col + 0.5f) / 2 + 0.0f * row;
    ////        glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));

    ////        world.createGameObject("plain", "cube", physics::ColliderType::CUBOID, glm::vec3(x, y, z), glm::vec3(1.0), 1, 0, glm::quat(1, 0, 0, 0), 2.5f, 0, randomColor);
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
    //                world.createGameObject("plain", "cube", physics::ColliderType::CUBOID, physics::BodyType::Dynamic, glm::vec3(newCenter), glm::vec3(2.5, 1.0, 1.0), 100, orientation, 1.5, true, randomColor, false);

    //                angleRad += glm::radians(30.0f);
    //            }
    //        }
    //    }
    //}
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
    //        world.createGameObject("uvmap", "cube", physics::ColliderType::CUBOID, physics::BodyType::Static, glm::vec3(baseX + i * 50, yOffset, baseZ + j * 50), glm::vec3(50.0, 1.0, 50.0), 0, orientation, 99, false, {}, false);
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
    //world.createGameObject("plain", "cube", physics::ColliderType::CUBOID, physics::BodyType::Static,
    //    glm::vec3((xMin + xMax) * 0.5f, y, zMin - thick * 0.5f),
    //    glm::vec3(lenX, wallH, thick), 0, wallOri, 0, 0, glm::vec3(190, 255, 255), false);

    //    // nord (zMax)
    //    world.createGameObject("plain", "cube", physics::ColliderType::CUBOID, physics::BodyType::Static,
    //        glm::vec3((xMin + xMax) * 0.5f, y, zMax + thick * 0.5f),
    //        glm::vec3(lenX, wallH, thick), 0, wallOri, 0, 0, glm::vec3(190, 255, 255), false);

    //        // väst (xMin)
    //        world.createGameObject("plain", "cube", physics::ColliderType::CUBOID, physics::BodyType::Static,
    //            glm::vec3(xMin - thick * 0.5f, y, (zMin + zMax) * 0.5f),
    //            glm::vec3(thick, wallH, lenZ), 0, wallOri, 0, 0, glm::vec3(190, 255, 255), false);

    //            // öst (xMax)
    //            world.createGameObject("plain", "cube", physics::ColliderType::CUBOID, physics::BodyType::Static,
    //                glm::vec3(xMax + thick * 0.5f, y, (zMin + zMax) * 0.5f),
    //                glm::vec3(thick, wallH, lenZ), 0, wallOri, 0, 0, glm::vec3(190, 255, 255), false);

    //                //// top floor
    //                //for (int i = 0; i < floorWidth; i++) {
    //                //    for (int j = 0; j < floorHeight; j++) {
    //                //        glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0f, 0.5f, 0.0f));
    //                //        world.createGameObject("uvmap", physics::ColliderType::CUBOID, glm::vec3(baseX + i * 50, wallH, baseZ + j * 50), glm::vec3(50, 1, 50), 0, 1, orientation);
    //                //        GameObject& floorTile = dynamicObjects.back();
    //                //        floorTile.seeThrough = true;
    //                //    }
    //                //}

    //world.createGameObject("plain", "cube", physics::ColliderType::CUBOID, physics::BodyType::Kinematic,
    //    glm::vec3(xMax + thick * 0.5f, y+100, (zMin + zMax) * 0.5f),
    //    glm::vec3(thick, wallH, lenZ), 0, wallOri, 0, 0, glm::vec3(190, 255, 255), false);
}


void SceneBuilder::castleScene() {
    glm::vec3 offset = { -30.0f, -0.5f, -30.0f };
    glm::vec3 cellSize = { 50, 1, 50 };
    createGridFloor(offset, cellSize, 4, 4);

    //createBrickWall(glm::vec3(25, 0, 44), 0, 20, 21, glm::vec3(1, 1, 1), 0.0f, 1, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(25, 0, 65), 0, 20, 21, glm::vec3(1, 1, 1), 0.0f, 1, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(25, 0, 45), 1, 20, 20, glm::vec3(1, 1, 1), 0.0f, 1, 0, glm::vec2(95, 110), false);
    //createBrickWall(glm::vec3(45, 0, 45), 1, 20, 20, glm::vec3(1, 1, 1), 0.0f, 1, 0, glm::vec2(95, 110), false);

    // castle walls
    for (int i = 0; i < 4; i++) {
        //// -x, +z
        //// side of gate walls
        //createBrickWall(glm::vec3(20, 0, 39 - i), 0, 10, 13, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
        //createBrickWall(glm::vec3(38, 0, 39 - i), 0, 10, 13, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
        //// over gate wall
        //createBrickWall(glm::vec3(33, 6, 39 - i), 0, 4, 5, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
        // +x, +z
        createBrickWall(glm::vec3(20, 0, 70 + i), 0, 10, 31, glm::vec3(1, 1, 1), 0.0f, 2500, 0, glm::vec2(95, 110), false);
        //// -x, -z
        //createBrickWall(glm::vec3(19 - i, 0, 40), 1, 10, 30, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
        //// +x, -z
        //createBrickWall(glm::vec3(51 + i, 0, 40), 1, 10, 30, glm::vec3(1, 1, 1), 0.0f, 100, 0, glm::vec2(95, 110), false);
    }

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
    //        world.createGameObject("uvmap", "cube", physics::ColliderType::CUBOID, physics::BodyType::Static, glm::vec3(baseX + i * 50, -0.5, baseZ + j * 50), glm::vec3(50.0, 1.0, 50.0), false, orientation, 0, false, {}, false);
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

    //            physics::ColliderType type;
    //            glm::vec3 size;
    //            std::string meshType;
    //            if (sphereLayer) {
    //                type = physics::ColliderType::SPHERE;
    //                size = glm::vec3(0.5f);
    //                meshType = "sphere";
    //            }
    //            else {
    //                type = physics::ColliderType::CUBOID;
    //                size = glm::vec3(1);
    //                meshType = "cube";
    //            }

    //            world.createGameObject("plain", meshType, type, physics::BodyType::Dynamic, glm::vec3(base.x + j, base.y + i, base.z + k), size, 1, glm::quat(1, 0, 0, 0), 1.0f, false, randomColor, false);
    //        }
    //    }
    //}
}

//-----------------------------
//    Tall Structure Scene
//-----------------------------
void SceneBuilder::tallStructureScene() {
    glm::vec3 offset = glm::vec3(-30.0f, 0.0f, -30.0f);
    glm::vec3 cellSize = { 50, 1, 50 };
    createGridFloor(offset, cellSize, 4, 4);

    // big box with high initial velocity
    GameObjectDesc bigBoxDesc;
    bigBoxDesc.name = "bigBox";
    bigBoxDesc.rootTransformHandle = world.createTransform(glm::vec3(-10.0, 5.0, -10.0), glm::quat(1, 0, 0, 0), glm::vec3(7.2));
    bigBoxDesc.mass = 1000;
    SubPartDesc bigBoxPart;
    bigBoxPart.localTransformHandle = world.createTransform();
    bigBoxDesc.parts.push_back(bigBoxPart);
    GameObjectHandle bigBoxHandle = world.createGameObject(bigBoxDesc);
    if (GameObject* object = world.getGameObject(bigBoxHandle)) {
        physicsEngine.setLinearVelocity(
            object->rigidBodyHandle,
            glm::vec3(120, 180, 150)
        );
    }


    // ----- staplar ----- 
    std::vector<glm::vec3> randomcolors = {
    glm::vec3(randomRange(0, 255)/255.0f, randomRange(0, 255)/255.0f, randomRange(0, 255)/255.0f),
    glm::vec3(randomRange(0, 255)/255.0f, randomRange(0, 255)/255.0f, randomRange(0, 255)/255.0f),
    };

    //std::vector<glm::vec3> randomcolors = {
    //    glm::vec3(67, 97, 167),
    //    glm::vec3(244, 244, 107)
    //};
    for (int g = 1; g < 6; g++) {
        for (int h = 1; h < 6; h++) {
            for (int i = 0; i < 17; i++) {

                for (int j = 0; j < 2; j++) {
                    for (int k = 0; k < 2; k++)
                    {
                        glm::vec3 pos(
                            (g * 6) + j * 5,
                            5 + i * 11,
                            (h * 6) + k * 5
                        );

                        GameObjectDesc columnDesc;
                        columnDesc.name = "column";
                        columnDesc.rootTransformHandle = world.createTransform(pos, glm::quat(1, 0, 0, 0), glm::vec3(1, 10, 1));
                        columnDesc.mass = 10;
                        columnDesc.asleep = true;
                        SubPartDesc columnPart;
                        columnPart.name = "columnPart";
                        columnPart.localTransformHandle = world.createTransform();
                        columnPart.textureName = "plain";
                        columnPart.color = randomcolors[0];
                        columnDesc.parts.push_back(columnPart);
                        world.createGameObject(columnDesc);
                    
                    }
                }   

                glm::vec3 pos(
                    (g * 6) + 2.5,
                    10.5 + i * 11,
                    (h * 6) + 2.5
                );

                GameObjectDesc floorDesc;
                floorDesc.name = "floor";
                floorDesc.rootTransformHandle = world.createTransform(pos, glm::quat(1, 0, 0, 0), glm::vec3(6, 1, 6));
                floorDesc.mass = 10;
                floorDesc.asleep = true;
                SubPartDesc floorPart;
                floorPart.name = "floorPart";
                floorPart.localTransformHandle = world.createTransform();
                floorPart.textureName = "plain";
                floorPart.color = randomcolors[1];
                floorDesc.parts.push_back(floorPart);
                world.createGameObject(floorDesc);
            }
        }
    }
}

//---------------------------
//         Main Scene
//---------------------------
void SceneBuilder::sandBox() {
    const glm::quat identity = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    auto createSinglePartObject = [&](
        const std::string& name,
        const std::string& textureName,
        const std::string& meshName,
        physics::ColliderType colliderType,
        physics::BodyType bodyType,
        const glm::vec3& position,
        const glm::vec3& scale,
        float mass,
        const glm::quat& orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        float sleepCounterThreshold = 0.0f,
        bool asleep = false,
        const glm::vec3& color255 = glm::vec3(-1.0f),
        bool seeThrough = false
        ) -> GameObjectHandle {
            GameObjectDesc object;
            object.name = name;
            object.rootTransformHandle = world.createTransform(position, orientation, scale);
            object.bodyType = bodyType;
            object.mass = mass;
            object.asleep = asleep;

            if (sleepCounterThreshold > 0.0f || asleep) {
                object.allowSleep = true;
                object.sleepCounterThreshold = sleepCounterThreshold;
            }

            SubPartDesc part;
            part.name = "MainPart";
            part.localTransformHandle = world.createTransform();
            part.textureName = textureName;
            part.meshName = meshName;
            part.colliderType = colliderType;
            part.seeThrough = seeThrough;

            if (color255.x >= 0.0f && color255.y >= 0.0f && color255.z >= 0.0f) {
                part.color = color255 / 255.0f;
            }

            object.parts.push_back(part);
            return world.createGameObject(object);
        };

    // ___________________________________________________________
    // ------------------------ floor tiles ----------------------
    int floorWidth = 8;
    int floorHeight = 8;

    const float baseX = -50.0f;
    const float baseZ = -50.0f;

    for (int i = 0; i < floorWidth; i++) {
        for (int j = 0; j < floorHeight; j++) {
            createSinglePartObject(
                "FloorTile",
                "uvmap",
                "cube",
                physics::ColliderType::CUBOID,
                physics::BodyType::Static,
                glm::vec3(baseX + i * 50.0f, -0.5f, baseZ + j * 50.0f),
                glm::vec3(50.0f, 1.0f, 50.0f),
                0.0f
            );
        }
    }

    // ___________________________________________________________
    // ------------------ walls around floor grid ----------------
    const float tileSize = 50.0f;
    const float halfTile = tileSize * 0.5f;
    const int w = floorWidth;
    const int h = floorHeight;

    const float wallH = 50.0f;
    const float thick = 20.0f;

    const float xMin = baseX - halfTile;
    const float xMax = baseX + (w - 1) * tileSize + halfTile;
    const float zMin = baseZ - halfTile;
    const float zMax = baseZ + (h - 1) * tileSize + halfTile;

    const float lenX = xMax - xMin + thick * 2.0f;
    const float lenZ = zMax - zMin + thick * 2.0f;

    const float y = wallH * 0.5f;
    const glm::vec3 wallColor = glm::vec3(190.0f, 255.0f, 255.0f);

    createSinglePartObject(
        "SouthWall",
        "plain",
        "cube",
        physics::ColliderType::CUBOID,
        physics::BodyType::Static,
        glm::vec3((xMin + xMax) * 0.5f, y, zMin - thick * 0.5f),
        glm::vec3(lenX, wallH, thick),
        0.0f,
        identity,
        0.0f,
        false,
        wallColor
    );

    createSinglePartObject(
        "NorthWall",
        "plain",
        "cube",
        physics::ColliderType::CUBOID,
        physics::BodyType::Static,
        glm::vec3((xMin + xMax) * 0.5f, y, zMax + thick * 0.5f),
        glm::vec3(lenX, wallH, thick),
        0.0f,
        identity,
        0.0f,
        false,
        wallColor
    );

    createSinglePartObject(
        "WestWall",
        "plain",
        "cube",
        physics::ColliderType::CUBOID,
        physics::BodyType::Static,
        glm::vec3(xMin - thick * 0.5f, y, (zMin + zMax) * 0.5f),
        glm::vec3(thick, wallH, lenZ),
        0.0f,
        identity,
        0.0f,
        false,
        wallColor
    );

    createSinglePartObject(
        "EastWall",
        "plain",
        "cube",
        physics::ColliderType::CUBOID,
        physics::BodyType::Static,
        glm::vec3(xMax + thick * 0.5f, y, (zMin + zMax) * 0.5f),
        glm::vec3(thick, wallH, lenZ),
        0.0f,
        identity,
        0.0f,
        false,
        wallColor
    );

    // ___________________________________________________________
    // ------------------------ bridge ---------------------------
    float wWidth = 5.0f;
    float wHeight = 0.5f;
    float wLength = 20.0f;
    float halfDepth = wWidth * 0.5f;

    glm::vec3 lastPos = glm::vec3(35.0f, 75.0f, 180.0f);
    float lastAngle = -90.0f;
    glm::quat lastOrient = glm::angleAxis(glm::radians(lastAngle), glm::vec3(0.0f, 0.0f, 1.0f));

    for (int i = 0; i < 37; ++i) {
        glm::quat newOrient = glm::angleAxis(glm::radians(lastAngle), glm::vec3(0.0f, 0.0f, 1.0f));
        glm::vec3 frontTip = lastPos + lastOrient * glm::vec3(halfDepth, 0.0f, 0.0f);
        glm::vec3 newPos = frontTip + newOrient * glm::vec3(halfDepth, 0.0f, 0.0f);

        createSinglePartObject(
            "BridgeSegment",
            "plain",
            "cube",
            physics::ColliderType::CUBOID,
            physics::BodyType::Static,
            newPos,
            glm::vec3(wWidth, wHeight, wLength),
            0.0f,
            newOrient,
            0.0f,
            false,
            glm::vec3(255.0f)
        );

        lastAngle += 5.0f;
        lastOrient = newOrient;
        lastPos = newPos;
    }

    // Falling pyramid
    createSpherePyramid(
        "plain",
        glm::vec3(-1.0f, -1.0f, -1.0f),
        glm::vec3(140.0f, 85.0f, 175.0f),
        10,
        8,
        0.5f,
        0.0f,
        0.5f,
        true
    );

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

    for (int i = 0; i < 3; i++) {
        glm::quat orientation = glm::angleAxis(glm::radians(angle), glm::vec3(1.0f, 0.0f, 0.0f));

        createSinglePartObject(
            "LeftSlope",
            "plain",
            "cube",
            physics::ColliderType::CUBOID,
            physics::BodyType::Static,
            glm::vec3(slopeLeftX, slopeLeftY + i * distHeight, slopeLeftZ),
            glm::vec3(slopeWidth, slopeHeight, slopeLength),
            0.0f,
            orientation,
            0.0f,
            false,
            glm::vec3(255.0f)
        );

        createSinglePartObject(
            "LeftSlopeRailA",
            "plain",
            "cube",
            physics::ColliderType::CUBOID,
            physics::BodyType::Static,
            glm::vec3(slopeLeftX + railWidth * 0.5f + slopeWidth * 0.5f, slopeLeftY + slopeHeight + i * distHeight, slopeLeftZ + slopeHeight),
            glm::vec3(railWidth, slopeHeight * railHeight, railLength),
            0.0f,
            orientation,
            0.0f,
            false,
            glm::vec3(255.0f)
        );

        createSinglePartObject(
            "LeftSlopeRailB",
            "plain",
            "cube",
            physics::ColliderType::CUBOID,
            physics::BodyType::Static,
            glm::vec3(slopeLeftX - railWidth * 0.5f - slopeWidth * 0.5f, slopeLeftY + slopeHeight + i * distHeight, slopeLeftZ + slopeHeight),
            glm::vec3(railWidth, slopeHeight * railHeight, railLength),
            0.0f,
            orientation,
            0.0f,
            false,
            glm::vec3(255.0f)
        );
    }

    for (int i = 0; i < 3; i++) {
        glm::quat orientation = glm::angleAxis(glm::radians(-angle), glm::vec3(1.0f, 0.0f, 0.0f));

        createSinglePartObject(
            "RightSlope",
            "plain",
            "cube",
            physics::ColliderType::CUBOID,
            physics::BodyType::Static,
            glm::vec3(slopeRightX, slopeRightY + i * distHeight, slopeRightZ),
            glm::vec3(slopeWidth, slopeHeight, slopeLength),
            0.0f,
            orientation,
            0.0f,
            false,
            glm::vec3(255.0f)
        );

        createSinglePartObject(
            "RightSlopeRailA",
            "plain",
            "cube",
            physics::ColliderType::CUBOID,
            physics::BodyType::Static,
            glm::vec3(slopeRightX + railWidth * 0.5f + slopeWidth * 0.5f, slopeRightY + slopeHeight + i * distHeight, slopeRightZ - slopeHeight),
            glm::vec3(railWidth, slopeHeight * railHeight, railLength),
            0.0f,
            orientation,
            0.0f,
            false,
            glm::vec3(255.0f)
        );

        createSinglePartObject(
            "RightSlopeRailB",
            "plain",
            "cube",
            physics::ColliderType::CUBOID,
            physics::BodyType::Static,
            glm::vec3(slopeRightX - railWidth * 0.5f - slopeWidth * 0.5f, slopeRightY + slopeHeight + i * distHeight, slopeRightZ - slopeHeight),
            glm::vec3(railWidth, slopeHeight * railHeight, railLength),
            0.0f,
            orientation,
            0.0f,
            false,
            glm::vec3(255.0f)
        );
    }

    createSpherePyramid(
        "plain",
        glm::vec3(-1.0f, -1.0f, -1.0f),
        glm::vec3(95.0f, 145.0f, 80.0f),
        10,
        8,
        0.5f,
        0.0f,
        0.5f,
        true
    );

    // ___________________________________________________________
    // ------------------------ ramp -----------------------------
    glm::quat rampOrientation = glm::angleAxis(glm::radians(-20.0f), glm::vec3(0.0f, 0.0f, -1.0f));

    GameObjectHandle rampHandle = createSinglePartObject(
        "Ramp",
        "plain",
        "cube",
        physics::ColliderType::CUBOID,
        physics::BodyType::Static,
        glm::vec3(80.0f, 0.0f, 120.0f),
        glm::vec3(30.0f, 0.5f, 30.0f),
        0.0f,
        rampOrientation,
        0.0f,
        false,
        glm::vec3(255.0f)
    );

    // Om GameObject::textureId fortfarande finns kvar och behövs:
    // if (GameObject* obj = world.getGameObject(rampHandle)) {
    //     obj->textureId = 999;
    // }

    // ___________________________________________________________
    // ------------------------ slanted platform -----------------
    glm::quat slantedOrientation = glm::angleAxis(glm::radians(25.0f), glm::vec3(1.0f, 0.5f, 0.0f));

    createSinglePartObject(
        "SlantedPlatform",
        "crate",
        "cube",
        physics::ColliderType::CUBOID,
        physics::BodyType::Static,
        glm::vec3(24.5f, 3.0f, 10.0f),
        glm::vec3(4.0f, 0.2f, 4.0f),
        0.0f,
        slantedOrientation
    );

    // ___________________________________________________________
    // ------------------------ catapult -------------------------
    createSinglePartObject(
        "CatapultSupport",
        "crate",
        "cube",
        physics::ColliderType::CUBOID,
        physics::BodyType::Static,
        glm::vec3(34.5f, 1.5f, 20.0f),
        glm::vec3(0.5f, 3.0f, 2.0f),
        1.0f
    );

    glm::vec3 plankSize = glm::vec3(14.0f, 0.2f, 0.5f);

    GameObjectHandle plankHandle = createSinglePartObject(
        "CatapultPlank",
        "crate",
        "cube",
        physics::ColliderType::CUBOID,
        physics::BodyType::Dynamic,
        glm::vec3(34.5f, 3.1f, 20.0f),
        plankSize,
        10.0f,
        identity,
        999.0f
    );

    if (GameObject* object = world.getGameObject(plankHandle)) {
        physicsEngine.setRigidBodyCanMoveLinearly(
            object->rigidBodyHandle,
            false
        );
    }

    createSinglePartObject(
        "CatapultProjectile",
        "crate",
        "cube",
        physics::ColliderType::CUBOID,
        physics::BodyType::Dynamic,
        glm::vec3(41.25f, 3.45f, 20.0f),
        glm::vec3(0.5f),
        1.0f,
        identity,
        1.0f
    );

    createSinglePartObject(
        "CatapultCounterweight",
        "crate",
        "cube",
        physics::ColliderType::CUBOID,
        physics::BodyType::Dynamic,
        glm::vec3(28.25f, 20.0f, 20.0f),
        glm::vec3(5.2f),
        1000.0f,
        identity,
        1.0f
    );

    // ____________________________________________________________
    // ----------------------- box stacks -------------------------
    int amountObjects = 7;

    for (int i = 0; i < amountObjects; i++) {
        createSinglePartObject(
            "StackBox",
            "crate",
            "cube",
            physics::ColliderType::CUBOID,
            physics::BodyType::Dynamic,
            glm::vec3(24.5f, 1.0f + 1.2f * i, 34.5f),
            glm::vec3(1.0f),
            1.0f,
            identity,
            2.5f
        );
    }

    for (int i = 0; i < amountObjects; i++) {
        createSinglePartObject(
            "StackBox",
            "crate",
            "cube",
            physics::ColliderType::CUBOID,
            physics::BodyType::Dynamic,
            glm::vec3(25.5f, 1.0f + 1.2f * i, 34.5f),
            glm::vec3(1.0f),
            1.0f,
            identity,
            2.5f
        );
    }

    for (int i = 0; i < amountObjects; i++) {
        createSinglePartObject(
            "StackBox",
            "crate",
            "cube",
            physics::ColliderType::CUBOID,
            physics::BodyType::Dynamic,
            glm::vec3(24.5f, 1.0f + 1.2f * i, 35.5f),
            glm::vec3(1.0f),
            1.0f,
            identity,
            2.5f
        );
    }

    for (int i = 0; i < amountObjects; i++) {
        createSinglePartObject(
            "StackBox",
            "crate",
            "cube",
            physics::ColliderType::CUBOID,
            physics::BodyType::Dynamic,
            glm::vec3(25.5f, 1.0f + 1.2f * i, 35.5f),
            glm::vec3(1.0f),
            1.0f,
            identity,
            2.5f
        );
    }

    // -------------------- double brick wall ---------------------
    int wallHeight = 20;
    int wallWidth = 20;
    float brickWidth = 1.0f;
    float brickLength = 1.0f;
    float brickHeight = 0.5f;
    float brickDistance = 0.2f;

    int brickWeight = 10;
    int brickDecrease = 1;

    createBrickWall(
        glm::vec3(75.0f, 0.0f, 7.0f),
        0,
        wallWidth,
        wallHeight,
        glm::vec3(brickWidth, brickHeight, brickLength),
        brickDistance,
        brickWeight,
        brickDecrease,
        glm::vec2(0.0f, 255.0f),
        true
    );

    createBrickWall(
        glm::vec3(74.0f, 0.0f, 8.0f),
        1,
        wallWidth,
        wallHeight,
        glm::vec3(brickWidth, brickHeight, brickLength),
        brickDistance,
        brickWeight,
        brickDecrease,
        glm::vec2(0.0f, 255.0f),
        true
    );

    // ----------------------- BIG pyramid -------------------------
    createBlockPyramid(
        "plain",
        glm::vec3(246.0f, 215.0f, 176.0f),
        glm::vec3(8.0f, 0.0f, 74.5f),
        15,
        12,
        1.0f,
        1.0f,
        5.0f,
        0.0f,
        1.0f,
        true
    );

    // ----------------------- brick wall2 -------------------------
    createBrickWall(
        glm::vec3(254.5f, 0.0f, -1.0f),
        1,
        10,
        100,
        glm::vec3(1.0f, 1.0f, 1.0f),
        0.0f,
        1,
        0,
        glm::vec2(0.0f, 255.0f),
        true
    );

    createSinglePartObject(
        "HeavyCube",
        "crate",
        "cube",
        physics::ColliderType::CUBOID,
        physics::BodyType::Dynamic,
        glm::vec3(254.5f, 9.6f, -30.0f),
        glm::vec3(5.2f),
        100000.0f,
        identity,
        1.0f
    );

    createSinglePartObject(
        "HeavySphere",
        "uvmap",
        "sphere",
        physics::ColliderType::SPHERE,
        physics::BodyType::Dynamic,
        glm::vec3(224.5f, 9.0f, 10.0f),
        glm::vec3(4.0f),
        100000.0f,
        identity,
        1.0f
    );

    // 2D pyramid of colored blocks
    for (int col = 10, test = 0; col > 0; col--, test++) {
        for (int row = 0; row < col; row++) {
            float x = 104.5f;
            float y = test + 0.5f;
            float z = 100.5f + row - (col + 0.5f) * 0.5f;

            glm::vec3 randomColor = glm::vec3(
                randomRange(0, 255),
                randomRange(0, 255),
                randomRange(0, 255)
            );

            createSinglePartObject(
                "PyramidBlock",
                "plain",
                "cube",
                physics::ColliderType::CUBOID,
                physics::BodyType::Dynamic,
                glm::vec3(x, y, z),
                glm::vec3(1.0f),
                1.0f,
                identity,
                3.0f,
                false,
                randomColor
            );
        }
    }

    createSinglePartObject(
        "TestCube",
        "plain",
        "cube",
        physics::ColliderType::CUBOID,
        physics::BodyType::Dynamic,
        glm::vec3(1.25f),
        glm::vec3(2.5f),
        10.0f,
        identity,
        3.5f
    );

    createSinglePartObject(
        "TeapotPlain",
        "plain",
        "teapot",
        physics::ColliderType::CUBOID,
        physics::BodyType::Dynamic,
        glm::vec3(-20.0f, 0.0f, 0.0f),
        glm::vec3(1.0f),
        10.0f,
        identity,
        3.5f,
        false,
        glm::vec3(255.0f, 255.0f, 255.0f)
    );

    createSinglePartObject(
        "TeapotCrate",
        "crate",
        "teapot",
        physics::ColliderType::CUBOID,
        physics::BodyType::Dynamic,
        glm::vec3(-20.0f, 0.0f, 10.0f),
        glm::vec3(1.0f),
        10.0f,
        identity,
        3.5f,
        false,
        glm::vec3(255.0f, 0.0f, 255.0f)
    );

    createSinglePartObject(
        "TeapotUvmap",
        "uvmap",
        "teapot",
        physics::ColliderType::CUBOID,
        physics::BodyType::Dynamic,
        glm::vec3(-20.0f, 0.0f, 20.0f),
        glm::vec3(1.0f),
        10.0f,
        identity,
        3.5f,
        false,
        glm::vec3(255.0f, 0.0f, 255.0f)
    );

    createSinglePartObject(
        "PylonPlain",
        "plain",
        "pylon",
        physics::ColliderType::CUBOID,
        physics::BodyType::Dynamic,
        glm::vec3(-30.0f, 2.0f, 0.0f),
        glm::vec3(1.0f),
        10.0f,
        identity,
        3.5f,
        false,
        glm::vec3(255.0f, 255.0f, 255.0f)
    );

    createSinglePartObject(
        "PylonCrate",
        "crate",
        "pylon",
        physics::ColliderType::CUBOID,
        physics::BodyType::Dynamic,
        glm::vec3(-30.0f, 2.0f, 10.0f),
        glm::vec3(1.0f),
        10.0f,
        identity,
        3.5f,
        false,
        glm::vec3(255.0f, 0.0f, 255.0f)
    );

    createSinglePartObject(
        "PylonUvmap",
        "uvmap",
        "pylon",
        physics::ColliderType::CUBOID,
        physics::BodyType::Dynamic,
        glm::vec3(-30.0f, 2.0f, 20.0f),
        glm::vec3(1.0f),
        10.0f,
        identity,
        3.5f,
        false,
        glm::vec3(255.0f, 0.0f, 255.0f)
    );


    glm::quat modelOrientation = glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    //createSinglePartObject(
    //    "Girl",
    //    "plain",
    //    "girl",
    //    physics::ColliderType::CUBOID,
    //    physics::BodyType::Static,
    //    glm::vec3(-20.0f, 0.0f, 0.0f),
    //    glm::vec3(0.0135f),
    //    1.0f,
    //    modelOrientation,
    //    3.5f,
    //    false,
    //    glm::vec3(255.0f)
    //);

    createSinglePartObject(
        "Tank",
        "plain",
        "tank",
        physics::ColliderType::CUBOID,
        physics::BodyType::Dynamic,
        glm::vec3(-20.0f, 0.0f, 0.0f),
        glm::vec3(1.0f),
        1.0f,
        modelOrientation,
        3.5f,
        false,
        glm::vec3(255.0f)
    );
}
