#pragma once

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Lidar.hpp"

namespace mtrn3100 {

class Maze {

public:
// ---------------- SETTINGS ----------------

static const int SIZE = 9;

// Change this after testing your actual maze
const uint16_t WALL_THRESHOLD = 80;


// ---------------- DIRECTIONS ----------------

enum Direction {
  NORTH = 0,
  EAST = 1,
  SOUTH = 2,
  WEST = 3
};


// ---------------- CONSTRUCTOR ----------------

Maze(Lidar& leftLidar, Lidar& rightLidar, Lidar& frontLidar, Direction startDirection):
  lidarL(leftLidar),
  lidarR(rightLidar),
  lidarF(frontLidar),
  robotDirection(startDirection)
{
  clearMap();
}


// ---------------- INITIALISE MAP ----------------

void clearMap() {

  cellsVisited = 0;

  robotRow = 0;
  robotCol = 0;

  // Clear wall data
  for (int row = 0; row < SIZE; row++) {
    for (int col = 0; col < SIZE; col++) {
      walls[row][col] = 0;
    }
  }

  // Clear visited data
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
  bool leftValid = lidarL.isLastReadValid();
  uint16_t frontDistance = lidarF.readDistance();
  bool frontValid = lidarF.isLastReadValid();
  uint16_t rightDistance = lidarR.readDistance();
  bool rightValid = lidarR.isLastReadValid();

  // Out-of-range readings are invalid. Do not reuse a stale close distance as
  // a wall: an invalid reading means no wall was detected.
  bool leftWall = leftValid && leftDistance < WALL_THRESHOLD;
  bool frontWall = frontValid && frontDistance < WALL_THRESHOLD;
  bool rightWall = rightValid && rightDistance < WALL_THRESHOLD;

  /*
    * Convert the robot-relative measurements
    * into absolute compass directions.
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

  // Mark this cell as visited
  visitCell(robotRow, robotCol);
}


// ---------------- WALL STORAGE ----------------

void setWall(int row, int col, Direction direction, bool hasWall) {

  // Make sure the cell is valid
  if (!validCell(row, col)) {
    return;
  }

  /*
    * Each cell uses one byte for wall information.
    *
    * Bit 0 = NORTH
    * Bit 1 = EAST
    * Bit 2 = SOUTH
    * Bit 3 = WEST
    */

  uint8_t wallBit = directionBit(direction);

  if (hasWall) {
    walls[row][col] |= wallBit;
  }

  else {
    walls[row][col] &= ~wallBit;
  }


  /*
    * Also update the neighbouring cell.
    *
    * For example:
    *
    * (2,2) EAST = wall
    *
    * means:
    *
    * (2,3) WEST = wall
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

    uint8_t oppositeWallBit = directionBit(oppositeDirection);

    if (hasWall) {
      walls[neighbourRow][neighbourCol] |= oppositeWallBit;
    }

    else {
      walls[neighbourRow][neighbourCol] &= ~oppositeWallBit;
    }
  }
}


// ---------------- VISITED CELLS ----------------

void visitCell(int row, int col) {

  if (!validCell(row, col)) {
    return;
  }

  /*
    * Each cell only needs one bit to store
    * whether it has been visited.
    *
    * 81 cells require 11 bytes.
    */

  int index = row * SIZE + col;
  int byteIndex = index / 8;
  int bitIndex = index % 8;

  uint8_t mask = 1 << bitIndex;

  if (!(visited[byteIndex] & mask)) {

    visited[byteIndex] |= mask;

    cellsVisited++;
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

  else if (robotDirection == EAST) {
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

  else if (robotDirection == WEST) {
    robotDirection = NORTH;
  }
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


float getMappingPercentage() const {
  return 100.0f * cellsVisited / (SIZE * SIZE);
}


bool hasWall(int row, int col, Direction direction) const {

  if (!validCell(row, col)) {
    return true;
  }

  return (walls[row][col] & directionBit(direction)) != 0;
}


bool isVisited(int row, int col) const {

  if (!validCell(row, col)) {
    return false;
  }

  int index = row * SIZE + col;
  int byteIndex = index / 8;
  int bitIndex = index % 8;

  return (visited[byteIndex] & (1 << bitIndex)) != 0;
}


// ---------------- DISPLAY MAP ----------------

void displayMap(Adafruit_SSD1306& display) {

  display.clearDisplay();

  const int CELL_WIDTH = 6;
  const int CELL_HEIGHT = 6;

  const int START_X = 1;
  const int START_Y = 1;


  // --------------------------------
  // DISPLAY MAPPING PERCENTAGE
  // --------------------------------

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.print(getMappingPercentage(), 1);
  display.println("% mapped");


  // --------------------------------
  // DISPLAY MAZE
  // --------------------------------

  // Go through every cell in the maze

  for (int row = 0; row < SIZE; row++) {

    for (int col = 0; col < SIZE; col++) {

      // Position of this cell on the OLED

      int x = START_X + col * CELL_WIDTH;
      int y = START_Y + row * CELL_HEIGHT;


      // --------------------------------
      // UNVISITED CELL
      // --------------------------------

      if (!isVisited(row, col)) {

        display.fillRect(x + 1, y + 1, CELL_WIDTH - 1, CELL_HEIGHT - 1, SSD1306_WHITE);
      }


      // --------------------------------
      // VISITED CELL
      // --------------------------------

      else {

        // North wall
        if (hasWall(row, col, NORTH)) {
          display.drawLine(x, y, x + CELL_WIDTH, y, SSD1306_WHITE);
        }

        // East wall
        if (hasWall(row, col, EAST)) {
          display.drawLine(x + CELL_WIDTH, y, x + CELL_WIDTH, y + CELL_HEIGHT, SSD1306_WHITE);
        }

        // South wall
        if (hasWall(row, col, SOUTH)) {
          display.drawLine(x, y + CELL_HEIGHT, x + CELL_WIDTH, y + CELL_HEIGHT, SSD1306_WHITE);
        }

        // West wall
        if (hasWall(row, col, WEST)) {
          display.drawLine(x, y, x, y + CELL_HEIGHT, SSD1306_WHITE);
        }
      }


      // --------------------------------
      // ROBOT POSITION
      // --------------------------------

      if (row == robotRow && col == robotCol) {

        // Small square inside the cell
        display.fillRect(x + 4, y + 2, 5, 3, SSD1306_WHITE);
      }
    }
  }


  // Send everything to the OLED

  display.display();
}


private:

// ---------------- SENSORS ----------------

Lidar& lidarL;
Lidar& lidarR;
Lidar& lidarF;


// ---------------- MAZE DATA ----------------

/*
  * WALL STORAGE
  *
  * Each cell uses one byte.
  *
  * Bit 0 = NORTH
  * Bit 1 = EAST
  * Bit 2 = SOUTH
  * Bit 3 = WEST
  *
  * Original storage:
  *
  * uint8_t walls[SIZE][SIZE][4];
  *
  * Memory = 324 bytes
  *
  * New storage:
  *
  * uint8_t walls[SIZE][SIZE];
  *
  * Memory = 81 bytes
  */

uint8_t walls[SIZE][SIZE];


/*
  * VISITED STORAGE
  *
  * One bit is used for each cell.
  *
  * 81 cells require:
  *
  * ceil(81 / 8) = 11 bytes
  */

uint8_t visited[(SIZE * SIZE + 7) / 8];


// ---------------- ROBOT STATE ----------------

int robotRow;
int robotCol;

Direction robotDirection;


// ---------------- MAPPING ----------------

int cellsVisited = 0;


// ---------------- WALL BIT HELPER ----------------

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

} // namespace mtrn3100
