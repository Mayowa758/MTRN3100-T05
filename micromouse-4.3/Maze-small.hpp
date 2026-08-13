#pragma once

#include <Arduino.h>
#include <U8g2lib.h>
#include "Lidar.hpp"
#include "Robot.hpp"

namespace mtrn3100 {

class Maze {

public:

// ---------------- SETTINGS ----------------

static const int SIZE = 9;
static const int TARGET_CELLS = 59;

const uint8_t WALL_THRESHOLD = 100;

// ---------------- DIRECTIONS ----------------

enum Direction {
NORTH = 0,
EAST = 1,
SOUTH = 2,
WEST = 3
};

// ---------------- POSITION ----------------

struct Position {
uint8_t row;
uint8_t col;
};

// ---------------- CONSTRUCTOR ----------------

Maze(Lidar& leftLidar, Lidar& rightLidar, Lidar& frontLidar, Direction startDirection, 
      uint8_t startRow, uint8_t startCol, uint8_t goalRow, uint8_t goalCol):
lidarL(leftLidar),
lidarR(rightLidar),
lidarF(frontLidar),
robotDirection(startDirection),
robotRow(startRow),
robotCol(startCol),
startRow(startRow),
startCol(startCol),
goalRow(goalRow),
goalCol(goalCol),
cellsVisited(0),
pathLength(0)
{
clearMap();
path[pathLength].row = robotRow;
path[pathLength].col = robotCol;
pathLength++;
}

// ---------------- INITIALISE MAP ----------------

void clearMap() {

cellsVisited = 0;
pathLength = 0;

for (int row = 0; row < SIZE; row++) {
for (int col = 0; col < SIZE; col++) {
walls[row][col] = 0;
knownWalls[row][col] = 0;
}
}

for (int i = 0; i < sizeof(visited); i++) {
visited[i] = 0;
}

/*
 * The outside boundary of the maze
 * is known to be a wall.
 */

for (int col = 0; col < SIZE; col++) {
setWall(0, col, NORTH, true);
setWall(SIZE - 1, col, SOUTH, true);
}

for (int row = 0; row < SIZE; row++) {
setWall(row, 0, WEST, true);
setWall(row, SIZE - 1, EAST, true);
}
}

// ---------------- MAP CURRENT CELL ----------------

void mapCurrentCell() {

uint16_t leftDistance = lidarL.readDistance();
uint16_t frontDistance = lidarF.readDistance();
uint16_t rightDistance = lidarR.readDistance();

bool leftWall = leftDistance < WALL_THRESHOLD;
bool frontWall = frontDistance < WALL_THRESHOLD;
bool rightWall = rightDistance < WALL_THRESHOLD;

Serial.print("Front: ");
Serial.print(frontDistance);
Serial.print(" Left: ");
Serial.print(leftDistance);
Serial.print(" Right: ");
Serial.println(rightDistance);

/*
 * Convert robot-relative measurements
 * into absolute maze directions.
 */

if (robotDirection == NORTH) {
setWall(robotRow, robotCol, WEST, leftWall);
setWall(robotRow, robotCol, NORTH, frontWall);
setWall(robotRow, robotCol, EAST, rightWall);
}

else if (robotDirection == EAST) {
setWall(robotRow, robotCol, NORTH, leftWall);
setWall(robotRow, robotCol, EAST, frontWall);
setWall(robotRow, robotCol, SOUTH, rightWall);
}

else if (robotDirection == SOUTH) {
setWall(robotRow, robotCol, EAST, leftWall);
setWall(robotRow, robotCol, SOUTH, frontWall);
setWall(robotRow, robotCol, WEST, rightWall);
}

else if (robotDirection == WEST) {
setWall(robotRow, robotCol, SOUTH, leftWall);
setWall(robotRow, robotCol, WEST, frontWall);
setWall(robotRow, robotCol, NORTH, rightWall);
}

visitCell(robotRow, robotCol);
}

// ---------------- WALL STORAGE ----------------

void setWall(int row, int col, Direction direction, bool hasWall) {

if (!validCell(row, col)) {
return;
}

uint8_t bit = directionBit(direction);

/*
 * Store whether this wall exists.
 */

if (hasWall) {
walls[row][col] |= bit;
}
else {
walls[row][col] &= ~bit;
}

/*
 * Store that this wall has been measured.
 */

knownWalls[row][col] |= bit;

/*
 * Update the neighbouring cell.
 */

int neighbourRow = row;
int neighbourCol = col;

Direction oppositeDirection;

if (direction == NORTH) {
neighbourRow--;
oppositeDirection = SOUTH;
}

else if (direction == EAST) {
neighbourCol++;
oppositeDirection = WEST;
}

else if (direction == SOUTH) {
neighbourRow++;
oppositeDirection = NORTH;
}

else {
neighbourCol--;
oppositeDirection = EAST;
}

if (validCell(neighbourRow, neighbourCol)) {

uint8_t oppositeBit = directionBit(oppositeDirection);

if (hasWall) {
walls[neighbourRow][neighbourCol] |= oppositeBit;
}
else {
walls[neighbourRow][neighbourCol] &= ~oppositeBit;
}

knownWalls[neighbourRow][neighbourCol] |= oppositeBit;
}
}

// ---------------- VISITED CELLS ----------------

void visitCell(int row, int col) {

if (!validCell(row, col)) {
return;
}

int index = row * SIZE + col;
int byteIndex = index / 8;
int bitIndex = index % 8;

uint8_t mask = 1 << bitIndex;

if (!(visited[byteIndex] & mask)) {
visited[byteIndex] |= mask;
cellsVisited++;

Serial.print("Visited: ");
Serial.print(row);
Serial.print(",");
Serial.print(col);
Serial.print(" Cells: ");
Serial.println(cellsVisited);
}
}

// ---------------- MOVE ROBOT POSITION ----------------

void moveToNextCell() {

if (robotDirection == NORTH) {
robotRow--;
}

else if (robotDirection == EAST) {
robotCol++;
}

else if (robotDirection == SOUTH) {
robotRow++;
}

else if (robotDirection == WEST) {
robotCol--;
}

if (validCell(robotRow, robotCol)) {
visitCell(robotRow, robotCol);
}
}

// ---------------- TURN ROBOT ----------------

void turnLeft() {

if (robotDirection == NORTH) {
robotDirection = WEST;
}

else if (robotDirection == WEST) {
robotDirection = SOUTH;
}

else if (robotDirection == SOUTH) {
robotDirection = EAST;
}

else {
robotDirection = NORTH;
}
}

void turnRight() {

if (robotDirection == NORTH) {
robotDirection = EAST;
}

else if (robotDirection == EAST) {
robotDirection = SOUTH;
}

else if (robotDirection == SOUTH) {
robotDirection = WEST;
}

else {
robotDirection = NORTH;
}
}

// ---------------- CHECK WALL ----------------

bool hasWall(int row, int col, Direction direction) const {

if (!validCell(row, col)) {
return true;
}

return (walls[row][col] & directionBit(direction)) != 0;
}

// ---------------- CHECK KNOWN WALL ----------------

bool isWallKnown(int row, int col, Direction direction) const {

if (!validCell(row, col)) {
return false;
}

return (knownWalls[row][col] & directionBit(direction)) != 0;
}

// ---------------- CHECK VISITED ----------------

bool isVisited(int row, int col) const {

if (!validCell(row, col)) {
return false;
}

int index = row * SIZE + col;
int byteIndex = index / 8;
int bitIndex = index % 8;

return (visited[byteIndex] & (1 << bitIndex)) != 0;
}

// ---------------- FIND NEXT CELL ----------------

bool canMoveTo(Direction direction) {

if (!isWallKnown(robotRow, robotCol, direction)) {
return false;
}

if (hasWall(robotRow, robotCol, direction)) {
return false;
}

int nextRow = robotRow;
int nextCol = robotCol;

if (direction == NORTH) {
nextRow--;
}

else if (direction == EAST) {
nextCol++;
}

else if (direction == SOUTH) {
nextRow++;
}

else {
nextCol--;
}

if (!validCell(nextRow, nextCol)) {
return false;
}

if (isVisited(nextRow, nextCol)) {
return false;
}

return true;
}

// ---------------- FIND UNVISITED DIRECTION ----------------

bool findUnvisitedDirection(Direction& direction) {

if (canMoveTo(NORTH)) {
direction = NORTH;
return true;
}

if (canMoveTo(EAST)) {
direction = EAST;
return true;
}

if (canMoveTo(SOUTH)) {
direction = SOUTH;
return true;
}

if (canMoveTo(WEST)) {
direction = WEST;
return true;
}

return false;
}

// ---------------- GET NEXT POSITION ----------------

Position getNextPosition(Direction direction) {

Position next;

next.row = robotRow;
next.col = robotCol;

if (direction == NORTH) {
next.row--;
}

else if (direction == EAST) {
next.col++;
}

else if (direction == SOUTH) {
next.row++;
}

else {
next.col--;
}

return next;
}

// ---------------- GET HEADING ----------------

float getHeading(Direction direction) const {

if (direction == EAST) {
return 0.0;
}

if (direction == NORTH) {
return 90.0;
}

if (direction == WEST) {
return 180.0;
}

return -90.0;
}

// ---------------- TURN TO DIRECTION ----------------

bool faceDirection(Robot& robot, Direction direction) {

float targetHeading = getHeading(direction);

bool success = robot.turnToHeading(targetHeading);

if (success) {
robotDirection = direction;
return true;
}

return false;
}

// ---------------- DFS MAPPING STEP ----------------

bool mappingStep(Robot& robot) {

  if (cellsVisited >= TARGET_CELLS) {
  robot.stopMotors();
  return false;
  }

  /*
  * Map the cell we are currently in.
  */

  mapCurrentCell();

  Direction nextDirection;

  /*
  * DFS:
  *
  * 1. Find an unvisited neighbouring cell.
  * 2. Move there.
  * 3. If no unvisited cell exists,
  *    backtrack to the previous cell.
  */

  if (findUnvisitedDirection(nextDirection)) {

  /*
  * Store current position in DFS path.
  */

  if (pathLength < SIZE * SIZE) {
  path[pathLength].row = robotRow;
  path[pathLength].col = robotCol;
  pathLength++;
  }

  /*
  * Turn towards the next cell.
  */

  if (!faceDirection(robot, nextDirection)) {
  robot.stopMotors();
  return false;
  }

  /*
  * Drive exactly one maze cell.
  */

  if (!robot.driveForwardOneCell()) {
  robot.stopMotors();
  return false;
  }

  /*
  * Update our internal maze position.
  */

  moveToNextCell();

  /*
  * Map the newly entered cell.
  */

  mapCurrentCell();

  return true;
  }

  /*
  * No unvisited neighbours.
  *
  * Backtrack to the previous cell.
  */

  if (pathLength <= 1) {
  robot.stopMotors();
  return false;
  }

  pathLength--;

  Position previous = path[pathLength - 1];

  Direction backDirection;

  if (previous.row < robotRow) {
  backDirection = NORTH;
  }

  else if (previous.row > robotRow) {
  backDirection = SOUTH;
  }

  else if (previous.col < robotCol) {
  backDirection = WEST;
  }

  else {
  backDirection = EAST;
  }

  Serial.println("Backtracking");

  if (!faceDirection(robot, backDirection)) {
  robot.stopMotors();
  return false;
  }

  if (!robot.driveForwardOneCell()) {
  robot.stopMotors();
  return false;
  }

  robotRow = previous.row;
  robotCol = previous.col;

  return true;
}

// ---------------- MAPPING COMPLETE ----------------

bool mappingComplete() const {
  return cellsVisited >= TARGET_CELLS;
}

// ---------------- ACCESSORS ----------------

int getRow() const {
return robotRow;
}

int getCol() const {
return robotCol;
}

Direction getDirection() const {
return robotDirection;
}

uint8_t getCellsVisited() const {
return cellsVisited;
}

float getMappingPercentage() const {
return 100.0f * cellsVisited / TARGET_CELLS;
}

// ---------------- DISPLAY MAP ----------------

void displayMap(U8G2_SSD1306_128X64_NONAME_1_HW_I2C& display) {

const int CELL_WIDTH = 6;
const int CELL_HEIGHT = 6;

display.firstPage();

do {

/*
 * ----------------
 * PERCENTAGE
 * ----------------
 */

display.setFont(u8g2_font_6x10_tf);

display.setCursor(70, 20);
display.print(getMappingPercentage(), 1);
display.print("%");

display.setCursor(70, 30);
display.print("mapped");

/*
 * ----------------
 * MAZE
 * ----------------
 */

for (int row = 0; row < SIZE; row++) {

for (int col = 0; col < SIZE; col++) {

int x = 1 + col * CELL_WIDTH;
int y = 5 + row * CELL_HEIGHT;

/*
 * ----------------
 * UNVISITED CELL
 * ----------------
 */

if (!isVisited(row, col)) {

display.drawBox(x + 1, y + 1, CELL_WIDTH - 1, CELL_HEIGHT - 1);
}

/*
 * ----------------
 * VISITED CELL
 * ----------------
 */

else {

if (hasWall(row, col, NORTH)) {
display.drawLine(x, y, x + CELL_WIDTH, y);
}

if (hasWall(row, col, EAST)) {
display.drawLine(x + CELL_WIDTH, y, x + CELL_WIDTH, y + CELL_HEIGHT);
}

if (hasWall(row, col, SOUTH)) {
display.drawLine(x, y + CELL_HEIGHT, x + CELL_WIDTH, y + CELL_HEIGHT);
}

if (hasWall(row, col, WEST)) {
display.drawLine(x, y, x, y + CELL_HEIGHT);
}
}

/*
 * ----------------
 * ROBOT
 * ----------------
 */

if (row == robotRow && col == robotCol) {

display.drawBox(x + 2, y + 2, 3, 3);
}
}
}

} while (display.nextPage());
}

// ---------------- BFS ----------------
void setGoal(uint8_t row, uint8_t col) {
    goalRow = row;
    goalCol = col;
}

bool atStart() const {
    return robotRow == startRow && robotCol == startCol;
}

bool atGoal() const {
    return robotRow == goalRow && robotCol == goalCol;
}

bool findShortestPath() {
  uint8_t queueRow[SIZE * SIZE];
  uint8_t queueCol[SIZE * SIZE];

  int previous[SIZE][SIZE];

  for (int row = 0; row < SIZE; row++) {
  for (int col = 0; col < SIZE; col++) {
  previous[row][col] = -1;
  }
  }

  int front = 0;
  int back = 0;

  queueRow[back] = startRow;
  queueCol[back] = startCol;
  back++;

  previous[startRow][startCol] = startRow * SIZE + startCol;

  while (front < back) {

  int row = queueRow[front];
  int col = queueCol[front];
  front++;

  if (row == goalRow && col == goalCol) {
  break;
  }

  Direction directions[4] = {
  NORTH,
  EAST,
  SOUTH,
  WEST
  };

  for (int i = 0; i < 4; i++) {

  Direction direction = directions[i];

  if (hasWall(row, col, direction)) {
  continue;
  }

  int nextRow = row;
  int nextCol = col;

  if (direction == NORTH) {
  nextRow--;
  }

  else if (direction == EAST) {
  nextCol++;
  }

  else if (direction == SOUTH) {
  nextRow++;
  }

  else {
  nextCol--;
  }

  if (!validCell(nextRow, nextCol)) {
  continue;
  }

  if (previous[nextRow][nextCol] != -1) {
  continue;
  }

  previous[nextRow][nextCol] = row * SIZE + col;

  queueRow[back] = nextRow;
  queueCol[back] = nextCol;
  back++;
  }
  }

  if (previous[goalRow][goalCol] == -1) {
  return false;
  }

  /*
  * Reconstruct path backwards.
  */

  pathLength = 0;

  int row = goalRow;
  int col = goalCol;

  while (!(row == startRow && col == startCol)) {

  if (pathLength >= SIZE * SIZE) {
  return false;
  }

  path[pathLength].row = row;
  path[pathLength].col = col;
  pathLength++;

  int previousIndex = previous[row][col];

  row = previousIndex / SIZE;
  col = previousIndex % SIZE;
  }

  path[pathLength].row = startRow;
  path[pathLength].col = startCol;
  pathLength++;

  /*
  * Reverse path so it goes:
  *
  * start -> goal
  */

  for (int i = 0; i < pathLength / 2; i++) {

  Position temp = path[i];

  path[i] = path[pathLength - 1 - i];

  path[pathLength - 1 - i] = temp;
  }

  return true;
}

bool driveShortestPath(Robot& robot) {
  if (pathLength < 2) {
      return false;
  }

  for (int i = 1; i < pathLength; i++) {

      Position current = path[i - 1];
      Position next = path[i];

      Direction direction;

      if (next.row < current.row) {
          direction = NORTH;
      }
      else if (next.row > current.row) {
          direction = SOUTH;
      }
      else if (next.col > current.col) {
          direction = EAST;
      }
      else {
          direction = WEST;
      }

      Serial.print("Moving to: ");
      Serial.print(next.row);
      Serial.print(",");
      Serial.println(next.col);

      if (!faceDirection(robot, direction)) {
          robot.stopMotors();
          return false;
      }

      if (!robot.driveForwardOneCell()) {
          robot.stopMotors();
          return false;
      }

      robotRow = next.row;
      robotCol = next.col;
      robotDirection = direction;
  }

  robot.stopMotors();

  return true;
}

// ---------------- PRIVATE ----------------

private:

uint8_t startRow;
uint8_t startCol;

uint8_t goalRow;
uint8_t goalCol;

// ---------------- SENSORS ----------------

Lidar& lidarL;
Lidar& lidarR;
Lidar& lidarF;

// ---------------- MAZE DATA ----------------

/*
 * Each cell uses one byte for walls.
 *
 * Bit 0 = NORTH
 * Bit 1 = EAST
 * Bit 2 = SOUTH
 * Bit 3 = WEST
 */

uint8_t walls[SIZE][SIZE];

/*
 * Each cell uses one byte for
 * whether each wall is known.
 */

uint8_t knownWalls[SIZE][SIZE];

/*
 * One bit per visited cell.
 *
 * 81 cells = 11 bytes.
 */

uint8_t visited[(SIZE * SIZE + 7) / 8];

// ---------------- ROBOT STATE ----------------

uint8_t robotRow;
uint8_t robotCol;

Direction robotDirection;

// ---------------- MAPPING ----------------

uint8_t cellsVisited;

// ---------------- DFS PATH ----------------

Position path[SIZE * SIZE];
uint8_t pathLength;

// ---------------- WALL BIT ----------------

uint8_t directionBit(Direction direction) const {
return (1 << direction);
}

// ---------------- VALIDATION ----------------

bool validCell(int row, int col) const {

return (
row >= 0 &&
row < SIZE &&
col >= 0 &&
col < SIZE
);
}

};

}