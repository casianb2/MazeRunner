# MazeRunner

This is a prototype/work-in-progress. The embedded code is a mess, not enough comments, hard to follow variable names, etc. 
It will most likely stay like this... a mess, because I am lazy.

It is a student project. A robot that solves a maze. When the robot detects a black square in the maze it means it solved the maze.

It uses an IR sensor (TCRT5000), 2 stepper motors (each with a4988 stepper driver), 3 ultrasonic proximity sensors (HC-SR04), 
bluetooth (HC-05 or HC-06), laptop battery, 2 arduinos, one which uses (i2c SD card module) to play music from SD card (we played 
the mario bros tune when the robot was seeking the exit, and when it found it, it would play the end level tune), the other arduino 
controls the rest of the functions and logic.

The robot follows the maze's left side rule for now.
