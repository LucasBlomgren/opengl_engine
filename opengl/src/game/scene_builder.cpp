#include "scene_builder.h"

std::vector<GameObject>& SceneBuilder::getGameObjectList() {
    return GameObjectList;
}

void SceneBuilder::setPointers(TextureManager* tm, LightManager* lm, std::mt19937& rng) {
    this->textureManager = tm;
    this->lightManager = lm;
    this->rng = &rng;
}

int SceneBuilder::randomRange(int start, int end) {
   if (start > end or start == end) {
      std::cerr << "Invalid range: " << start << " to " << end << std::endl;
      return -1; 
   }

   std::uniform_int_distribution<int> dist(start, end);
   return dist(*this->rng);
}

void SceneBuilder::toggleDayTime() {
    dayTime = !dayTime;

    if (dayTime) {
        // sun light
        lightManager->setDirectionalLight(glm::vec3(0.8f, -1.0f, 0.4f), glm::vec3(0.1f), glm::vec3(1.0), glm::vec3(0.5));
        lightManager->clearLights();
    }
    else {
        // red light
        Light light(glm::vec3(350, 160, 320), glm::vec3(5, 2, 5), glm::vec3(1.0, 0.0, 0.0), 60);
        lightManager->addLight(light);

        // green
        Light light2(glm::vec3(150, 220, 200), glm::vec3(20, 2, 20), glm::vec3(0.0, 1.0, 0.0), 75);
        lightManager->addLight(light2);

        // blue light
        Light light3(glm::vec3(1050, 220, 1000), glm::vec3(20, 2, 20), glm::vec3(0.0, 0.0, 1.0), 100);
        lightManager->addLight(light3);
        
        lightManager->clearDirectionalLight();
    }
}

void SceneBuilder::createScene(PhysicsEngine& physicsEngine)
{
    bool onlyFloor = 0;

    objectId = 0;
    GameObjectList.clear();
    GameObjectList.reserve(10000);
    physicsEngine.clearPhysicsData();

    // ________________________________________________________
    // ------------------------ light sources -----------------
    lightManager->clearLights();
    lightManager->clearDirectionalLight();

    if (this->dayTime) {
        // sun light
        lightManager->setDirectionalLight(glm::vec3(0.8f, -1.0f, 0.4f), glm::vec3(0.1f), glm::vec3(1.0), glm::vec3(0.5));
    }
    else {
        // red light
        Light light(glm::vec3(350, 160, 320), glm::vec3(5, 2, 5), glm::vec3(1.0, 0.0, 0.0), 60);
        lightManager->addLight(light);
        // green
        Light light2(glm::vec3(150, 220, 200), glm::vec3(20, 2, 20), glm::vec3(0.0, 1.0, 0.0), 75);
        lightManager->addLight(light2);
        // blue light
        Light light3(glm::vec3(1050, 220, 1000), glm::vec3(20, 2, 20), glm::vec3(0.0, 0.0, 1.0), 100);
        lightManager->addLight(light3);
    }

    // ___________________________________________________________
    // ------------------------ floor tiles ----------------------
    int floorWidth = 5; 
    int floorHeight = 5;

    if (onlyFloor) {
       floorWidth = 1;
       floorHeight = 1;
    }
    for (int i = 0; i < floorWidth; i++)
        for (int j = 0; j < floorHeight; j++) {
            glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0f, 0.5f, 0.0f));
            createObject("uvmap", glm::vec3(250 + i * 500, -5, 250 + j * 500), glm::vec3(500, 10, 500), 0, 1, orientation);
            
        }

    if (onlyFloor) {
        return;
    }

    // ___________________________________________________________
    // ------------------------ sloped platforms -----------------
    float slopeLeftX = 1000.0f;
    float slopeLeftY = 250.0f;
    float slopeLeftZ = 600.0f;

    float slopeRightX = 1000.0f;
    float slopeRightY = 450.0f;
    float slopeRightZ = 800.0f;

    float slopeWidth = 250.0f;
    float slopeHeight = 10.0f;
    float slopeLength = 300.0f;

    float railWidth = slopeHeight;
    float railHeight = 5.0f;
    float railLength = slopeLength;

    float distHeight = 400.0f;
    float angle = 40.0f;

    // left
    for (int i = 0; i < 3; i++) {
        glm::quat orientation = glm::angleAxis(glm::radians(angle), glm::vec3(1.0f, 0.0f, 0.0f));

        // main platform 
        glm::vec3 pos{ slopeLeftX, slopeLeftY + i * distHeight, slopeLeftZ };
        glm::vec3 size{ slopeWidth, slopeHeight, slopeLength };
        createObject("plain", pos, size, 0, 1, orientation);
        // guard rails

        pos = glm::vec3(slopeLeftX + (railWidth / 2) + (slopeWidth / 2), slopeLeftY + slopeHeight + i * distHeight, slopeLeftZ + slopeHeight);
        size = glm::vec3(railWidth, slopeHeight * railHeight, railLength);
        createObject("plain", pos, size, 0, 1, orientation);

        pos = glm::vec3(slopeLeftX - (railWidth / 2) - (slopeWidth / 2), slopeLeftY + slopeHeight + i * distHeight, slopeLeftZ + slopeHeight);
        size = glm::vec3(railWidth, slopeHeight * railHeight, railLength);
        createObject("plain", pos, size, 0, 1, orientation);
    }
    // right
    for (int i = 0; i < 3; i++) {
        glm::quat orientation = glm::angleAxis(glm::radians(-angle), glm::vec3(1.0f, 0.0f, 0.0f));
        // main platform
        glm::vec3 pos{ slopeRightX, slopeRightY + i * distHeight, slopeRightZ };
        glm::vec3 size{ slopeWidth, slopeHeight, slopeLength };
        createObject("plain", pos, size, 0, 1, orientation);

        // guard rails
        pos = glm::vec3(slopeRightX + (railWidth / 2) + (slopeWidth / 2), slopeRightY + slopeHeight + i * distHeight, slopeRightZ - slopeHeight);
        size = glm::vec3(railWidth, slopeHeight * railHeight, railLength);
        createObject("plain", pos, size, 0, 1, orientation);

        pos = glm::vec3(slopeRightX - (railWidth / 2) - (slopeWidth / 2), slopeRightY + slopeHeight + i * distHeight, slopeRightZ - slopeHeight);
        size = glm::vec3(railWidth, slopeHeight * railHeight, railLength);
        createObject("plain", pos, size, 0, 1, orientation);
    
    }
    // falling pyramid
    int pHeight = 8;
    int pWidth = 10;
    int sWidth = 10;
    int sLength = 10;
    int sHeight = 10; 
    int sDist = 0;
    int pWidthCounter = pWidth;
    for (int y = 0; y < pHeight; y++) {
        for (int x = 0; x < pWidthCounter; x++) {
            for (int z = 0; z < pWidthCounter; z++) {
                float xPos = 950 + x * (sWidth + sDist) + y * (sWidth / 2 + sDist);
                float yPos = 1450 + sHeight / 2 + y * sHeight;
                float zPos = 800 + z * (sWidth + sDist) + y * (sWidth / 2 + sDist);

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

                createObject("plain", glm::vec3(xPos, yPos, zPos), glm::vec3(xSize, ySize, zSize), 1, 0, glm::quat(1, 0, 0, 0), 2.0f, 1, color);
            }
        }
        pWidthCounter -= 1;
    }

    // ___________________________________________________________
    // ------------------------ ramp -----------------------------
    glm::quat orientation1 = glm::angleAxis(glm::radians(-20.0f), glm::vec3(0.0f, 0.0f, -1.0f));
    createObject("uvmap", glm::vec3(800,0,1500), glm::vec3(300, 5, 300), 0, 1, orientation1);
    GameObject& obj = GameObjectList.back();
    obj.textureID = 999;

    // ___________________________________________________________
    // ------------------------ slanted platform -----------------
    glm::quat orientation = glm::angleAxis(glm::radians(25.0f), glm::vec3(1.0f, 0.5f, 0.0f));
    createObject("crate", glm::vec3(245, 30, 100), glm::vec3(40, 2, 40), 0, 1, orientation);

    // ___________________________________________________________
    // ------------------------ catapult -------------------------
    // support
    createObject( "crate", glm::vec3(345, 15, 200), glm::vec3(5, 30, 20), 1, 1);

    // plank
    int mass = 50;
    glm::vec3 size = glm::vec3(140, 2, 5);
    createObject("crate", glm::vec3(345, 31, 200), size, mass, 0, {}, 999);
    GameObjectList.back().canMoveLinearly = false;

    // projectile
    createObject("crate", glm::vec3(412.5, 34.5, 200), glm::vec3(5, 5, 5), 1, 0, {}, 999);
    // projectile2
    //createObject("crate", glm::vec3(277.5, 34.5, 200), glm::vec3(5, 5, 5), 5, 0, {}, 999);
    // counterweight
    createObject("crate", glm::vec3(282.5, 200, 200), glm::vec3(12, 12, 12), 100, 0, {}, 999);

    // ____________________________________________________________
    // ----------------------- box stacks -------------------------
    int amountObjects = 5;
    for (int i = 0; i < amountObjects; i++)
       createObject("crate", glm::vec3(245, 10 + (12 * i), 245), glm::vec3(10, 10, 10), 1, 0, {}, 0.5);
    for (int i = 0; i < amountObjects; i++)
       createObject("crate", glm::vec3(255, 10 + (12 * i), 245), glm::vec3(10, 10, 10), 1, 0, {}, 0.5);
    for (int i = 0; i < amountObjects; i++)
       createObject("crate", glm::vec3(245, 10 + (12 * i), 255), glm::vec3(10, 10, 10), 1, 0, {}, 0.5);
    for (int i = 0; i < amountObjects; i++)
       createObject("crate", glm::vec3(255, 10 + (12 * i), 255), glm::vec3(10, 10, 10), 1, 0, {}, 0.5);


    // ____________________________________________________________
    // ----------------------- brick wall -------------------------
    int wallHeight = 15;
    int wallWidth = 20;
    int brickWidth = 10;
    int brickLength = 10;
    int brickHeight = 5;
    int brickDistance = 2;

    int brickWeight = 10;
    int brickDecrease = 1;

    if (brickWeight < wallHeight) {
       brickWeight = wallHeight;
    }

    // col
    for (int col = 0; col < wallHeight/2; col++) {
       // row 0, 2, 4, 6...
       for (int row = 0; row < wallWidth; row++) {
          float x = 545;
          float y = brickHeight / 2 + col * brickHeight * 2;
          float z = 80 + row * brickLength + brickDistance * row;
          glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
          createObject("plain", glm::vec3(x, y, z), glm::vec3(brickWidth, brickHeight, brickLength), brickWeight, 0, glm::quat(1, 0, 0, 0), 0.2f, 1, randomColor);
       }
       brickWeight--;
       // row 1, 3, 5, 7...
       for (int row = 0; row < wallWidth-1; row++) {
          float x = 545;
          float y = brickHeight + brickHeight / 2 + col * brickHeight * 2;
          float z = 86 + row * brickLength + brickDistance * row;
          glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
          createObject("plain", glm::vec3(x, y, z), glm::vec3(brickWidth, brickHeight, brickLength), brickWeight, 0, glm::quat(1, 0, 0, 0), 0.2f, 1, randomColor);
       }
       brickWeight--;
    }

    brickWeight = 10;
    if (brickWeight < wallHeight) {
       brickWeight = wallHeight;
    }
    // col
    for (int col = 0; col < wallHeight / 2; col++) {
       // row 0, 2, 4, 6...
       for (int row = 0; row < wallWidth; row++) {
          float x = 550 + row * brickLength + brickDistance * row;
          float y = brickHeight + brickHeight / 2 + col * brickHeight * 2;
          float z = 70;
          glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
          createObject("plain", glm::vec3(x, y, z), glm::vec3(brickWidth, brickHeight, brickLength), brickWeight, 0, glm::quat(1, 0, 0, 0), 0.2f, 1, randomColor);
       }
       brickWeight--;
       // row 1, 3, 5, 7...
       for (int row = 0; row < wallWidth-1; row++) {
          float x = 556 + row * brickLength + brickDistance * row;
          float y = brickHeight / 2 + col * brickHeight * 2;
          float z = 70;
          glm::vec3 randomColor = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
          createObject("plain", glm::vec3(x, y, z), glm::vec3(brickWidth, brickHeight, brickLength), brickWeight, 0, glm::quat(1, 0, 0, 0), 0.2f, 1, randomColor);
       }
       brickWeight--;
    }

    // _____________________________________________________________
    // ----------------------- pyramids ----------------------------
    int numPyramidsX = 2;
    int numPyramidsZ = 2;

    int pyramidHeight = 8;
    int pyramidWidth = 10;
    int stoneWidth = 5;
    int stoneLength = 5;
    int stoneHeight = 20;
    int stoneDistance = 0;

    int stoneWeight = 1;

    // multiple pyramids
    for (int i = 0; i < numPyramidsX; i++) {
        for (int j = 0; j < numPyramidsZ; j++) {
            // single pyramid
            int pyramidWidthCounter = pyramidWidth;
            for (int y = 0; y < pyramidHeight; y++) {
                for (int x = 0; x < pyramidWidthCounter; x++) {
                    for (int z = 0; z < pyramidWidthCounter; z++) {
                        float xPos = i * 150 + 80 + x * (stoneWidth + stoneDistance) + y * (stoneWidth / 2 + stoneDistance);
                        float yPos = stoneHeight / 2 + y * stoneHeight;
                        float zPos = j * 150 + 345 + z * (stoneWidth + stoneDistance) + y * (stoneWidth / 2 + stoneDistance);

                        //glm::vec3 color = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
                        glm::vec3 color = glm::vec3(246, 215, 176);

                        createObject("plain", glm::vec3(xPos, yPos, zPos), glm::vec3(stoneWidth, stoneHeight, stoneLength), stoneWeight, 0, glm::quat(1, 0, 0, 0), 0.2f, 1, color);
                    }
                }
                pyramidWidthCounter -= 1;
            }
        }
    }

    ////// sphere
    ////GameObject& sphere = createObject(physicsEngine, "crate", glm::vec3(500,500,500), glm::vec3(10), 1, 0);
    ////sphere.textureID = 999;
}

void SceneBuilder::objectRain(float& current_time, std::mt19937& rng) {
    constexpr float interval = 1.0f / 200.0f;
    if (current_time - lastTime < interval)
        return;

    lastTime = current_time;

    // position
    constexpr glm::vec3 spawnPoint = glm::vec3(1250, 650, 1250);
    float varianceRange = 1100.0f;
    float xVariance = randomRange(-varianceRange, varianceRange);
    float yVariance = randomRange(-25, 25);
    float zVariance = randomRange(-varianceRange, varianceRange);
    glm::vec3 color = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
    glm::vec3 spawnPos = spawnPoint + glm::vec3(xVariance, yVariance, zVariance);

    // orientation
    float randomAng = randomRange(0, 360);
    glm::vec3 randomAxis = glm::vec3(randomRange(-1, 1), randomRange(-1, 1), randomRange(-1, 1));
    glm::quat orientation = glm::angleAxis(glm::radians(randomAng), randomAxis);

    createObject("plain", spawnPos, glm::vec3(10.0f), 1, 0, orientation, 2.0f, 0, color);
}

GameObject& SceneBuilder::createObject
(
    const std::string& textureName,
    glm::vec3 pos,
    glm::vec3 size,
    float mass,
    bool isStatic,
    glm::quat orientation,
    float sleepCounterThreshold,
    bool asleep,
    glm::vec3 color
)
{
    unsigned int textureID;
    if (textureName == "plain") 
        textureID = 999;
    else 
        textureID = textureManager->getTexture(textureName);

    GameObject object(objectId, cubeVertices, indices, pos, size, mass, isStatic, textureID, orientation, sleepCounterThreshold, asleep, color);

    GameObjectList.emplace_back(object);
    objectId++;

    return GameObjectList.back();
}