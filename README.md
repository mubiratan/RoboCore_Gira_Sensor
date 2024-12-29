# Anti-collision-check-sides

It was written for ESP32 hardware compatible with the Arduino architecture.

The program was written in C++ using the PlatformIO plugin in VSCode and the "RoboCore_Vespa" library.

1) Robot starts moving forward;
2) If it finds an obstacle, it stops, turns the sensor to the left and right, if it finds obstacles it records them in the stack;
3) If there are obstacles on both sides, the robot should move backward for a defined time and check again for obstacles;
4) If it finds only one obstacle, it moves to the opposite side, e.g., if it finds one on the right, it turns 90 degrees and moves to the left.
5) If both sides have no obstacles, it randomly chooses which side to follow.