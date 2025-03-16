#include <Servo.h>
#include <SPI.h>
#include <MFRC522.h>
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <QueueArray.h>

// Motor Control
#define MOTOR_L 5
#define MOTOR_R 6
Servo motorL, motorR;

// Ultrasonic Sensors
#define NUM_SENSORS 3
NewPing sonar[NUM_SENSORS] = {
  NewPing(7, 8, 200),   // Left
  NewPing(9, 10, 200),  // Front
  NewPing(11, 12, 200)  // Right
};

// RFID
#define RST_PIN 13
#define SS_PIN 4
MFRC522 rfid(SS_PIN, RST_PIN);

// Voice & Speaker
SoftwareSerial voiceModule(2, 3); // RX, TX

// Grid Navigation
struct Node {
  bool walls[4] = {false}; // N, E, S, W
  bool visited = false;
  bool tempBlocked = false;
  int parentX = -1, parentY = -1;
};
Node grid[4][4];

struct Location {
  String name;
  int x;
  int y;
};

Location locations[] = {
  {"entrance", 0, 0}, {"dairy", 1, 0}, {"bakery", 2, 0}, {"beverages", 3, 0},
  {"vegetables", 0, 1}, {"fruits", 1, 1}, {"meat", 2, 1}, {"fish", 3, 1},
  {"snacks", 0, 2}, {"cereals", 1, 2}, {"spices", 2, 2}, {"canned", 3, 2},
  {"checkout", 0, 3}, {"frozen", 1, 3}, {"deli", 2, 3}, {"toiletries", 3, 3}
};

// Robot State
int currentX = 0, currentY = 0, currentDir = 0; // 0-N, 1-E, 2-S, 3-W
QueueArray<int> bfsQueue;
unsigned long lastObstacleTime = 0;
const unsigned long retryInterval = 10000;
bool mappingCompleted = false;
bool locationsConfigured = false;

void setup() {
  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();
  voiceModule.begin(9600);
  
  motorL.attach(MOTOR_L);
  motorR.attach(MOTOR_R);
  
  configureLocations();
  mapMaze();
  speak("System ready. Please state your destination.");
  mappingCompleted = true;
}

void loop() {
  checkVoiceCommands();
  checkRFID();
  
  if (!bfsQueue.isEmpty() && millis() - lastObstacleTime > retryInterval) {
    executeNextMove();
  }
}

// Location Configuration
void configureLocations() {
  speak("Would you like to configure locations? Say yes or no.");
  
  if (getVoiceConfirmation()) {
    speak("Starting location configuration.");
    for (auto &loc : locations) {
      configureSingleLocation(loc);
    }
  } else {
    speak("Using default locations.");
  }
  locationsConfigured = true;
}

void configureSingleLocation(Location &loc) {
  while (true) {
    speak("Current coordinates for " + loc.name + " are X: " + String(loc.x) + " Y: " + String(loc.y));
    speak("Say keep or change.");

    String response = getVoiceResponse();
    if (response.indexOf("keep") != -1) return;

    int newX = getNumberInput("X");
    int newY = getNumberInput("Y");

    if (isValidCoordinate(newX) && isValidCoordinate(newY)) {
      loc.x = newX;
      loc.y = newY;
      speak(loc.name + " now at X: " + String(newX) + " Y: " + String(newY));
      return;
    }
    speak("Invalid coordinates. Try again.");
  }
}

// Voice Processing
void checkVoiceCommands() {
  if (voiceModule.available()) {
    String command = voiceModule.readStringUntil('\n');
    command.toLowerCase();
    processVoiceCommand(command);
  }
}

void processVoiceCommand(String command) {
  for (const auto &loc : locations) {
    if (command.indexOf(loc.name) != -1) {
      navigateToDestination(loc);
      return;
    }
  }
  speak("Destination not recognized.");
}

void navigateToDestination(Location loc) {
  if (!locationsConfigured || !mappingCompleted) {
    speak("System not ready. Please wait.");
    return;
  }
  
  speak("Calculating route to " + loc.name);
  calculateBFSPath(loc.x, loc.y);
}

// BFS Pathfinding
void calculateBFSPath(int targetX, int targetY) {
  resetGridForBFS();
  bfsQueue.push(currentX);
  bfsQueue.push(currentY);
  grid[currentY][currentX].visited = true;

  while (!bfsQueue.isEmpty()) {
    int x = bfsQueue.pop();
    int y = bfsQueue.pop();

    if (x == targetX && y == targetY) {
      reconstructPath(targetX, targetY);
      return;
    }

    exploreNeighbor(x, y, x+1, y); // East
    exploreNeighbor(x, y, x-1, y); // West
    exploreNeighbor(x, y, x, y+1); // South
    exploreNeighbor(x, y, x, y-1); // North
  }

  speak("Path not found. Try again.");
  bfsQueue.clear();
}

void exploreNeighbor(int x, int y, int nx, int ny) {
  if (nx >=0 && nx <4 && ny >=0 && ny <4 && !grid[ny][nx].visited && !grid[ny][nx].tempBlocked) {
    if (!hasWallBetween(x, y, nx, ny)) {
      grid[ny][nx].parentX = x;
      grid[ny][nx].parentY = y;
      grid[ny][nx].visited = true;
      bfsQueue.push(nx);
      bfsQueue.push(ny);
    }
  }
}

void reconstructPath(int targetX, int targetY) {
  int x = targetX, y = targetY;
  while (x != currentX || y != currentY) {
    int px = grid[y][x].parentX;
    int py = grid[y][x].parentY;
    addMoveToQueue(px, py, x, y);
    x = px;
    y = py;
  }
}

// Movement Control
void executeNextMove() {
  if (detectObstacle()) {
    handleObstacle();
    return;
  }

  if (bfsQueue.isEmpty()) return;
  
  int move = bfsQueue.pop();
  switch(move) {
    case 0: moveForward(); break;
    case 1: turnRight(); break;
    case 2: turnLeft(); break;
  }
}

void moveForward() {
  motorL.write(180);
  motorR.write(0);
  delay(1000); // Adjust based on movement speed
  stopMotors();
  updatePosition();
}

void turnRight() {
  motorL.write(180);
  motorR.write(180);
  delay(500); // Adjust for 90° turn
  stopMotors();
  currentDir = (currentDir + 1) % 4;
}

void turnLeft() {
  motorL.write(0);
  motorR.write(0);
  delay(500); // Adjust for 90° turn
  stopMotors();
  currentDir = (currentDir + 3) % 4;
}

// Sensor Handling
bool detectObstacle() {
  return sonar[1].ping_cm() < 20; // Front sensor
}

void handleObstacle() {
  speak("Obstacle detected. Recalculating.");
  grid[currentY][currentX].tempBlocked = true;
  lastObstacleTime = millis();
  calculateBFSPath(bfsQueue);
}

// Helper Functions
void addMoveToQueue(int px, int py, int x, int y) {
  if (x == px+1) bfsQueue.push(0); // Move East
  else if (x == px-1) bfsQueue.push(0); // Move West
  else if (y == py+1) bfsQueue.push(0); // Move South
  else if (y == py-1) bfsQueue.push(0); // Move North
}

void resetGridForBFS() {
  for (int y = 0; y < 4; y++) {
    for (int x = 0; x < 4; x++) {
      grid[y][x].visited = false;
      grid[y][x].parentX = -1;
      grid[y][x].parentY = -1;
    }
  }
}

bool hasWallBetween(int x1, int y1, int x2, int y2) {
  // Implement based on grid data
  return false;
}

void updatePosition() {
  switch(currentDir) {
    case 0: currentY--; break; // North
    case 1: currentX++; break; // East
    case 2: currentY++; break; // South
    case 3: currentX--; break; // West
  }
}

void stopMotors() {
  motorL.write(90);
  motorR.write(90);
}

// Voice Helpers
void speak(String msg) {
  voiceModule.println("SPK:" + msg);
  delay(300);
}

String getVoiceResponse() {
  unsigned long timeout = millis() + 10000;
  while (millis() < timeout) {
    if (voiceModule.available()) {
      String response = voiceModule.readStringUntil('\n');
      response.toLowerCase();
      return response;
    }
  }
  return "";
}

int getNumberInput(String coordinate) {
  speak("Enter " + coordinate + " coordinate (0-3)");
  unsigned long timeout = millis() + 10000;
  while (millis() < timeout) {
    if (voiceModule.available()) {
      String input = voiceModule.readStringUntil('\n');
      input.trim();
      if (input.length() == 1 && isDigit(input[0])) {
        return input.toInt();
      }
    }
  }
  return -1;
}

bool getVoiceConfirmation() {
  String response = getVoiceResponse();
  return response.indexOf("yes") != -1;
}

bool isValidCoordinate(int value) {
  return value >= 0 && value <= 3;
}

// RFID Handling
void checkRFID() {
  if (!rfid.PICC_IsNewCardPresent()) return;
  
  String item = readRFID();
  speak("Item: " + item + ". Confirm purchase?");
}

String readRFID() {
  // Implement RFID reading logic
  return "Test Item";
}

// Maze Mapping
void mapMaze() {
  while (!allVisited()) {
    updateWalls();
    grid[currentY][currentX].visited = true;
    
    if (canMoveLeft()) {
      turnLeft();
      moveForward();
    } else if (canMoveForward()) {
      moveForward();
    } else if (canMoveRight()) {
      turnRight();
      moveForward();
    } else {
      turnAround();
      moveForward();
    }
  }
}

bool allVisited() {
  for (int y = 0; y < 4; y++) {
    for (int x = 0; x < 4; x++) {
      if (!grid[y][x].visited) return false;
    }
  }
  return true;
}

void updateWalls() {
  // Implement sensor-based wall detection
}

bool canMoveLeft() {
  return sonar[0].ping_cm() > 20; // Left sensor
}

bool canMoveRight() {
  return sonar[2].ping_cm() > 20; // Right sensor
}

bool canMoveForward() {
  return sonar[1].ping_cm() > 20; // Front sensor
}

void turnAround() {
  turnLeft();
  turnLeft();
}
