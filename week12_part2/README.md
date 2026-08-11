# Week 12 Part 2 Test Procedure

## First 3 Marks

1. Open and run `plan_route.py` in VS Code.
2. Select the overhead maze image.
3. Select the four corners of the 5×5 obstacle area in this order:
   - top-left;
   - top-right;
   - bottom-right;
   - bottom-left.
4. Select the green entrance cell and the red exit cell.
5. Press Enter to generate the obstacle-course route.
6. Check the result image. Make sure the route does not cross a wall or the cylinder clearance area.
7. Open `arduino/week12_part2/week12_part2.ino` in the Arduino IDE.
8. Upload the sketch to the robot.
9. Place the robot in the centre of the green entrance cell, facing into the 5×5 obstacle area.


## Final 2 Marks

1. Open and run `plan_full_route.py` in VS Code.
2. Enter the full-maze start row, start column, starting direction and goal row and column supplied for the test.
3. Click `Generate full route`.
4. Check the complete result image. Make sure the orange, magenta and blue route sections do not cross any wall or the cylinder clearance area.
5. Upload `arduino/week12_part2/week12_part2.ino` again. The sketch must be uploaded again because the route has changed.
6. Place the robot in the centre of the supplied full-maze start cell.
7. Point the robot in the entered starting direction:
   - `N`: towards the top of the image;
   - `E`: towards the right of the image;
   - `S`: towards the bottom of the image;
   - `W`: towards the left of the image.


