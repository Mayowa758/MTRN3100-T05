#pragma once

#include <Arduino.h>
#include <U8g2lib.h>
#include "Lidar.hpp"
#include "Robot.hpp"

namespace mtrn3100 {

class Maze {

public:

    // ============================================================
    // SETTINGS
    // ============================================================

    static constexpr uint8_t SIZE = 9;
    // A 9 x 9 board has 81 positions, but the octagonal maze excludes the
    // three cut-off positions at each corner: 81 - 12 = 69 usable cells.
    static constexpr uint8_t TARGET_CELLS = 69;
    // A wall must be close enough to be the boundary of the current cell.
    // Far/ambiguous readings are deliberately treated as open space.
    static constexpr uint8_t WALL_THRESHOLD = 80;


    // ============================================================
    // DIRECTIONS
    // ============================================================

    enum Direction : uint8_t {
        NORTH = 0,
        EAST = 1,
        SOUTH = 2,
        WEST = 3
    };


    // ============================================================
    // POSITION
    // ============================================================

    struct Position {
        uint8_t row;
        uint8_t col;
    };


    // ============================================================
    // CONSTRUCTOR
    // ============================================================

    Maze(
        Lidar& leftLidar,
        Lidar& rightLidar,
        Lidar& frontLidar,
        Direction startDirection,
        uint8_t startRow,
        uint8_t startCol,
        uint8_t goalRow,
        uint8_t goalCol
    )
        : lidarL(leftLidar),
          lidarR(rightLidar),
          lidarF(frontLidar),
          robotDirection(startDirection),
          startDirection(startDirection),
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


    // ============================================================
    // INITIALISE MAP
    // ============================================================

    void clearMap() {

        cellsVisited = 0;
        pathLength = 0;

        for (uint8_t row = 0; row < SIZE; row++) {
            for (uint8_t col = 0; col < SIZE; col++) {
                walls[row][col] = 0;
                knownWalls[row][col] = 0;
            }
        }

        for (uint8_t i = 0; i < sizeof(visited); i++) {
            visited[i] = 0;
        }

        // Add permanent walls around the rectangular outer edge and the four
        // cut-off corners of the physical octagonal maze.
        for (uint8_t row = 0; row < SIZE; row++) {
            for (uint8_t col = 0; col < SIZE; col++) {
                if (!validCell(row, col)) {
                    continue;
                }

                for (uint8_t direction = NORTH; direction <= WEST; direction++) {
                    if (isMazeBoundary(row, col, (Direction)direction)) {
                        setWall(row, col, (Direction)direction, true);
                    }
                }
            }
        }
    }


    // ============================================================
    // MAP CURRENT CELL
    // ============================================================

    void mapCurrentCell() {

        uint16_t leftDistance = lidarL.readDistance();
        bool leftValid = lidarL.isLastReadValid();
        uint16_t frontDistance = lidarF.readDistance();
        bool frontValid = lidarF.isLastReadValid();
        uint16_t rightDistance = lidarR.readDistance();
        bool rightValid = lidarR.isLastReadValid();

        // An invalid/out-of-range reading must mean "no wall detected". The
        // Lidar class otherwise returns its last valid value, which may be a
        // stale close-wall measurement from an earlier cell.
        bool leftWall =
            leftValid &&
            leftDistance < WALL_THRESHOLD &&
            leftDistance != 0;

        bool frontWall =
            frontValid &&
            frontDistance < WALL_THRESHOLD &&
            frontDistance != 0;

        bool rightWall =
            rightValid &&
            rightDistance < WALL_THRESHOLD &&
            rightDistance != 0;

        /*
         * Convert robot-relative measurements
         * into absolute maze directions.
         */

        if (robotDirection == NORTH) {

            setWall(
                robotRow,
                robotCol,
                WEST,
                leftWall
            );

            setWall(
                robotRow,
                robotCol,
                NORTH,
                frontWall
            );

            setWall(
                robotRow,
                robotCol,
                EAST,
                rightWall
            );
        }

        else if (robotDirection == EAST) {

            setWall(
                robotRow,
                robotCol,
                NORTH,
                leftWall
            );

            setWall(
                robotRow,
                robotCol,
                EAST,
                frontWall
            );

            setWall(
                robotRow,
                robotCol,
                SOUTH,
                rightWall
            );
        }

        else if (robotDirection == SOUTH) {

            setWall(
                robotRow,
                robotCol,
                EAST,
                leftWall
            );

            setWall(
                robotRow,
                robotCol,
                SOUTH,
                frontWall
            );

            setWall(
                robotRow,
                robotCol,
                WEST,
                rightWall
            );
        }

        else {

            setWall(
                robotRow,
                robotCol,
                SOUTH,
                leftWall
            );

            setWall(
                robotRow,
                robotCol,
                WEST,
                frontWall
            );

            setWall(
                robotRow,
                robotCol,
                NORTH,
                rightWall
            );
        }

        visitCell(robotRow, robotCol);
    }


    // ============================================================
    // WALL STORAGE
    // ============================================================

    void setWall(
        int row,
        int col,
        Direction direction,
        bool hasWall
    ) {

        if (!validCell(row, col)) {
            return;
        }

        /*
         * Never allow the outer maze boundary
         * to be overwritten by a LiDAR reading.
         */

        if (row == 0 && direction == NORTH) {
            hasWall = true;
        }

        if (row == SIZE - 1 && direction == SOUTH) {
            hasWall = true;
        }

        if (col == 0 && direction == WEST) {
            hasWall = true;
        }

        if (col == SIZE - 1 && direction == EAST) {
            hasWall = true;
        }

        // The diagonal cut-offs at each corner are also permanent boundaries.
        if (isMazeBoundary(row, col, direction)) {
            hasWall = true;
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
         * Update neighbouring cell.
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

            uint8_t oppositeBit =
                directionBit(oppositeDirection);

            if (hasWall) {
                walls[neighbourRow][neighbourCol] |= oppositeBit;
            }
            else {
                walls[neighbourRow][neighbourCol] &= ~oppositeBit;
            }

            knownWalls[neighbourRow][neighbourCol] |=
                oppositeBit;
        }
    }


    // ============================================================
    // VISITED CELLS
    // ============================================================

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
        }
    }


    // ============================================================
    // MOVE ROBOT POSITION
    // ============================================================

    void moveToNextCell() {

        /*
         * Use int here because moving NORTH from row 0
         * temporarily produces -1.
         *
         * Do NOT use uint8_t for these temporary values.
         */

        int nextRow = robotRow;
        int nextCol = robotCol;

        if (robotDirection == NORTH) {
            nextRow--;
        }

        else if (robotDirection == EAST) {
            nextCol++;
        }

        else if (robotDirection == SOUTH) {
            nextRow++;
        }

        else if (robotDirection == WEST) {
            nextCol--;
        }

        if (validCell(nextRow, nextCol)) {

            robotRow = nextRow;
            robotCol = nextCol;

            visitCell(robotRow, robotCol);
        }
    }


    // ============================================================
    // TURN ROBOT
    // ============================================================

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


    // ============================================================
    // CHECK WALL
    // ============================================================

    bool hasWall(
        int row,
        int col,
        Direction direction
    ) const {

        if (!validCell(row, col)) {
            return true;
        }

        return (
            walls[row][col] &
            directionBit(direction)
        ) != 0;
    }


    // ============================================================
    // CHECK KNOWN WALL
    // ============================================================

    bool isWallKnown(
        int row,
        int col,
        Direction direction
    ) const {

        if (!validCell(row, col)) {
            return false;
        }

        return (
            knownWalls[row][col] &
            directionBit(direction)
        ) != 0;
    }


    // ============================================================
    // CHECK VISITED
    // ============================================================

    bool isVisited(int row, int col) const {

        if (!validCell(row, col)) {
            return false;
        }

        int index = row * SIZE + col;
        int byteIndex = index / 8;
        int bitIndex = index % 8;

        return (
            visited[byteIndex] &
            (1 << bitIndex)
        ) != 0;
    }


    // ============================================================
    // FIND NEXT CELL
    // ============================================================

    bool canMoveTo(Direction direction) {

        if (!isWallKnown(
                robotRow,
                robotCol,
                direction
            )) {

            return false;
        }

        if (hasWall(
                robotRow,
                robotCol,
                direction
            )) {

            return false;
        }

        /*
         * Keep temporary coordinates as int because
         * they may become -1 at the boundary.
         */

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


    // ============================================================
    // FIND UNVISITED DIRECTION
    // ============================================================

    bool findUnvisitedDirection(
        Direction& direction
    ) {

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


    // ============================================================
    // GET NEXT POSITION
    // ============================================================

    Position getNextPosition(Direction direction) {

        Position next;

        next.row = robotRow;
        next.col = robotCol;

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

        if (validCell(nextRow, nextCol)) {
            next.row = nextRow;
            next.col = nextCol;
        }

        return next;
    }


    // ============================================================
    // TURN TO DIRECTION
    // ============================================================

    bool faceDirection(
        Robot& robot,
        Direction direction
    ) {

        int difference =
            direction - robotDirection;

        if (difference < 0) {
            difference += 4;
        }

        if (difference == 0) {
            return true;
        }

        if (difference == 1) {

            if (robot.turnRight90()) {

                robotDirection = direction;
                return true;
            }
        }

        else if (difference == 2) {

            if (robot.turnRight90()) {

                if (robot.turnRight90()) {

                    robotDirection = direction;
                    return true;
                }
            }
        }

        else if (difference == 3) {

            if (robot.turnLeft90()) {

                robotDirection = direction;
                return true;
            }
        }

        return false;
    }


    // ============================================================
    // DFS MAPPING STEP
    // ============================================================

    bool mappingStep(Robot& robot) {

        if (cellsVisited >= TARGET_CELLS) {

            robot.stopMotors();
            return false;
        }

        /*
         * Map current cell.
         */

        mapCurrentCell();

        Direction nextDirection;

        /*
         * DFS:
         *
         * 1. Find unvisited neighbour.
         * 2. Turn towards it.
         * 3. Drive one cell.
         * 4. Add new cell to DFS path.
         */

        if (findUnvisitedDirection(nextDirection)) {

            if (!faceDirection(
                    robot,
                    nextDirection
                )) {

                robot.stopMotors();
                return false;
            }

            if (!robot.driveForwardOneCell()) {

                robot.stopMotors();
                return false;
            }

            // A front-lidar stop is not a completed cell movement. Stay in
            // the current map cell and let the next mapping step choose a
            // different known-open direction.
            if (!robot.didLastForwardCompleteCell()) {
                mapCurrentCell();
                return true;
            }

            /*
             * Update robot position.
             */

            moveToNextCell();

            /*
             * Add newly entered cell to DFS path.
             */

            if (pathLength < SIZE * SIZE) {

                path[pathLength].row =
                    robotRow;

                path[pathLength].col =
                    robotCol;

                pathLength++;
            }

            /*
             * Map new cell.
             */

            mapCurrentCell();

            return true;
        }


        // --------------------------------------------------------
        // BACKTRACK
        // --------------------------------------------------------

        if (pathLength <= 1) {

            robot.stopMotors();
            return false;
        }

        /*
         * Remove current cell from DFS path.
         */

        pathLength--;

        Position previous =
            path[pathLength - 1];

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

        if (!faceDirection(
                robot,
                backDirection
            )) {

            robot.stopMotors();
            return false;
        }

        if (!robot.driveForwardOneCell()) {

            robot.stopMotors();
            return false;
        }

        // Do not claim that the robot returned to its previous cell unless
        // the encoder-based one-cell movement actually completed.
        if (!robot.didLastForwardCompleteCell()) {
            robot.stopMotors();
            return false;
        }

        /*
         * Update robot position.
         */

        robotRow = previous.row;
        robotCol = previous.col;

        return true;
    }


    // ============================================================
    // MAPPING COMPLETE
    // ============================================================

    bool mappingComplete() const {

        return cellsVisited >= TARGET_CELLS;
    }


    // ============================================================
    // ACCESSORS
    // ============================================================

    uint8_t getRow() const {
        return robotRow;
    }

    uint8_t getCol() const {
        return robotCol;
    }

    Direction getDirection() const {
        return robotDirection;
    }

    uint8_t getCellsVisited() const {
        return cellsVisited;
    }

    float getMappingPercentage() const {

        return (
            100.0f *
            cellsVisited /
            TARGET_CELLS
        );
    }


    // ============================================================
    // DISPLAY MAP
    // ============================================================

    void displayMap(
        U8G2_SSD1306_128X64_NONAME_1_HW_I2C& display
    ) {

        constexpr uint8_t CELL_WIDTH = 6;
        constexpr uint8_t CELL_HEIGHT = 6;

        display.firstPage();

        do {

            display.setFont(
                u8g2_font_6x10_tf
            );

            display.setCursor(70, 20);
            display.print(
                getMappingPercentage(),
                1
            );
            display.print("%");

            display.setCursor(70, 30);
            display.print("mapped");


            // ----------------------------------------------------
            // DRAW MAZE
            // ----------------------------------------------------

            for (uint8_t row = 0; row < SIZE; row++) {

                for (uint8_t col = 0; col < SIZE; col++) {

                    // The three cells in each physical corner are outside the
                    // octagonal maze, so do not draw or visit them.
                    if (!validCell(row, col)) {
                        continue;
                    }

                    uint8_t x =
                        1 + col * CELL_WIDTH;

                    uint8_t y =
                        5 + row * CELL_HEIGHT;


                    // ------------------------------------------------
                    // UNVISITED CELL
                    // ------------------------------------------------

                    if (!isVisited(row, col)) {

                        display.drawBox(
                            x + 1,
                            y + 1,
                            CELL_WIDTH - 1,
                            CELL_HEIGHT - 1
                        );
                    }


                    // ------------------------------------------------
                    // VISITED CELL
                    // ------------------------------------------------

                    else {

                        if (hasWall(
                                row,
                                col,
                                NORTH
                            )) {

                            display.drawLine(
                                x,
                                y,
                                x + CELL_WIDTH,
                                y
                            );
                        }

                        if (hasWall(
                                row,
                                col,
                                EAST
                            )) {

                            display.drawLine(
                                x + CELL_WIDTH,
                                y,
                                x + CELL_WIDTH,
                                y + CELL_HEIGHT
                            );
                        }

                        if (hasWall(
                                row,
                                col,
                                SOUTH
                            )) {

                            display.drawLine(
                                x,
                                y + CELL_HEIGHT,
                                x + CELL_WIDTH,
                                y + CELL_HEIGHT
                            );
                        }

                        if (hasWall(
                                row,
                                col,
                                WEST
                            )) {

                            display.drawLine(
                                x,
                                y,
                                x,
                                y + CELL_HEIGHT
                            );
                        }
                    }


                    // ------------------------------------------------
                    // ROBOT
                    // ------------------------------------------------

                    if (
                        row == robotRow &&
                        col == robotCol
                    ) {

                        display.drawBox(
                            x + 2,
                            y + 2,
                            3,
                            3
                        );
                    }
                }
            }

        } while (display.nextPage());
    }


    // ============================================================
    // BFS SETTINGS
    // ============================================================

    void setGoal(
        uint8_t row,
        uint8_t col
    ) {

        goalRow = row;
        goalCol = col;
    }


    bool atStart() const {

        return (
            robotRow == startRow &&
            robotCol == startCol
        );
    }


    bool atGoal() const {

        return (
            robotRow == goalRow &&
            robotCol == goalCol
        );
    }


    // ============================================================
    // BFS SHORTEST PATH
    // ============================================================

    bool findShortestPath() {

        /*
         * 255 means "no predecessor".
         *
         * There are only 81 cells, so uint8_t
         * is enough to store a cell index.
         */

        uint8_t previous[SIZE * SIZE];

        for (
            uint8_t i = 0;
            i < SIZE * SIZE;
            i++
        ) {

            previous[i] = 255;
        }


        /*
         * Reuse path[] as the BFS queue.
         *
         * This avoids allocating:
         *
         * queueRow[81]
         * queueCol[81]
         *
         * and saves about 162 bytes of stack.
         */

        uint8_t front = 0;
        uint8_t back = 0;


        path[back].row = startRow;
        path[back].col = startCol;

        back++;


        uint8_t startIndex =
            startRow * SIZE + startCol;

        previous[startIndex] =
            startIndex;


        // --------------------------------------------------------
        // BFS
        // --------------------------------------------------------

        while (front < back) {

            uint8_t row =
                path[front].row;

            uint8_t col =
                path[front].col;

            front++;


            if (
                row == goalRow &&
                col == goalCol
            ) {

                break;
            }


            const Direction directions[4] = {
                NORTH,
                EAST,
                SOUTH,
                WEST
            };


            for (
                uint8_t i = 0;
                i < 4;
                i++
            ) {

                Direction direction =
                    directions[i];


                if (hasWall(
                        row,
                        col,
                        direction
                    )) {

                    continue;
                }


                /*
                 * Keep these as int because
                 * NORTH from row 0 gives -1.
                 */

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


                if (!validCell(
                        nextRow,
                        nextCol
                    )) {

                    continue;
                }


                uint8_t nextIndex =
                    nextRow * SIZE + nextCol;


                /*
                 * Already visited by BFS.
                 */

                if (
                    previous[nextIndex] != 255
                ) {

                    continue;
                }


                /*
                 * Store predecessor.
                 */

                previous[nextIndex] =
                    row * SIZE + col;


                /*
                 * Add cell to BFS queue.
                 */

                path[back].row =
                    nextRow;

                path[back].col =
                    nextCol;

                back++;
            }
        }


        // --------------------------------------------------------
        // CHECK WHETHER GOAL WAS FOUND
        // --------------------------------------------------------

        uint8_t goalIndex =
            goalRow * SIZE + goalCol;


        if (
            previous[goalIndex] == 255
        ) {

            return false;
        }


        // --------------------------------------------------------
        // RECONSTRUCT PATH
        // --------------------------------------------------------

        pathLength = 0;

        int row = goalRow;
        int col = goalCol;


        while (
            !(row == startRow &&
              col == startCol)
        ) {

            if (
                pathLength >=
                SIZE * SIZE
            ) {

                return false;
            }


            path[pathLength].row =
                row;

            path[pathLength].col =
                col;

            pathLength++;


            uint8_t currentIndex =
                row * SIZE + col;


            uint8_t previousIndex =
                previous[currentIndex];


            row =
                previousIndex / SIZE;

            col =
                previousIndex % SIZE;
        }


        /*
         * Add start cell.
         */

        path[pathLength].row =
            startRow;

        path[pathLength].col =
            startCol;

        pathLength++;


        // --------------------------------------------------------
        // REVERSE PATH
        // --------------------------------------------------------

        for (
            uint8_t i = 0;
            i < pathLength / 2;
            i++
        ) {

            Position temp =
                path[i];


            path[i] =
                path[pathLength - 1 - i];


            path[pathLength - 1 - i] =
                temp;
        }


        return true;
    }


    // ============================================================
    // DRIVE SHORTEST PATH
    // ============================================================

    bool driveShortestPath(
        Robot& robot
    ) {

        if (pathLength < 2) {
            return false;
        }


        for (
            uint8_t i = 1;
            i < pathLength;
            i++
        ) {

            Position current =
                path[i - 1];

            Position next =
                path[i];


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


            if (!faceDirection(
                    robot,
                    direction
                )) {

                robot.stopMotors();
                return false;
            }


            if (!robot.driveForwardOneCell()) {

                robot.stopMotors();
                return false;
            }

            // The planned-path state is updated only after a full cell move.
            if (!robot.didLastForwardCompleteCell()) {
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


private:

    // ============================================================
    // START / GOAL
    // ============================================================

    uint8_t startRow;
    uint8_t startCol;

    uint8_t goalRow;
    uint8_t goalCol;


    // ============================================================
    // SENSORS
    // ============================================================

    Lidar& lidarL;
    Lidar& lidarR;
    Lidar& lidarF;


    // ============================================================
    // MAZE DATA
    // ============================================================

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

    uint8_t visited[
        (SIZE * SIZE + 7) / 8
    ];


    // ============================================================
    // ROBOT STATE
    // ============================================================

    uint8_t robotRow;
    uint8_t robotCol;

    Direction robotDirection;
    Direction startDirection;


    // ============================================================
    // MAPPING
    // ============================================================

    uint8_t cellsVisited;


    // ============================================================
    // DFS / BFS PATH
    // ============================================================

    /*
     * During DFS:
     *
     * path[] = DFS stack
     *
     * During BFS:
     *
     * path[] = BFS queue, then reconstructed
     *          shortest path.
     */

    Position path[
        SIZE * SIZE
    ];

    uint8_t pathLength;


    // ============================================================
    // WALL BIT
    // ============================================================

    uint8_t directionBit(
        Direction direction
    ) const {

        return (1 << direction);
    }


    // ============================================================
    // VALIDATION
    // ============================================================

    bool validCell(
        int row,
        int col
    ) const {
        if (row < 0 || row >= SIZE || col < 0 || col >= SIZE) {
            return false;
        }

        // Three excluded cells at each corner of the 9 x 9 bounding square.
        const bool topLeft = (row == 0 && col <= 1) || (row == 1 && col == 0);
        const bool topRight = (row == 0 && col >= SIZE - 2) ||
                              (row == 1 && col == SIZE - 1);
        const bool bottomLeft = (row == SIZE - 1 && col <= 1) ||
                                (row == SIZE - 2 && col == 0);
        const bool bottomRight = (row == SIZE - 1 && col >= SIZE - 2) ||
                                 (row == SIZE - 2 && col == SIZE - 1);

        return !(topLeft || topRight || bottomLeft || bottomRight);
    }

    bool isMazeBoundary(
        int row,
        int col,
        Direction direction
    ) const {
        int neighbourRow = row;
        int neighbourCol = col;

        if (direction == NORTH) neighbourRow--;
        else if (direction == EAST) neighbourCol++;
        else if (direction == SOUTH) neighbourRow++;
        else neighbourCol--;

        return !validCell(neighbourRow, neighbourCol);
    }
};

}
