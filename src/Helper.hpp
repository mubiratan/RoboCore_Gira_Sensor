#ifndef HELPER_HPP
#define HELPER_HPP

#include <stack>
#include "RoboCore_Vespa.h"

// Sensor pin
const uint_fast8_t PINO_TRIGGER = 25;
const uint_fast8_t PINO_ECHO = 26;

// Variables that stores the distance from the obstacle
const uint_fast8_t OBSTACLE_DISTANCE = 25;
const uint_fast8_t OBSTACLE_DISTANCE_FAR_AWAY = 40;

// Variables that store the rotation and straight speed of the robot's motors
const uint_fast8_t SPEED = 80;

// Variables that stores the waiting time between sensor readings
const uint_fast8_t WAIT = 100;
const uint_fast8_t WAIT_TURN = 200;

// Variables that stores the time that the deviation movements will last
const unsigned int WAIT_MOTION = 1000;
const unsigned int ROTATION_90 = WAIT_MOTION;
constexpr unsigned int ROTATION_180 = WAIT_MOTION * 2;

// Stack Status
const uint_fast8_t FRONT_OBSTACLE = 0, RIGHT_OBSTACLE = 1, LEFT_OBSTACLE = 2;

static uint_fast8_t ANGLES[] = {0, 30, 150};

// LED
const int LED_VESPA = 15;
const uint32_t TIME_UPDATE_VBAT = 60000; // [ms]

//Servo
const uint16_t SERVO_MAX = 2200;
const uint16_t SERVO_MIN = 700;
const uint_fast8_t WAIT_TURN_SENSOR = 300;
const uint_fast8_t SMALLER_TURN_SENSOR_WAIT = 150;

// Functions
void walkBackwards(VespaMotors &, const uint_fast8_t espera);
void spinsRobot(VespaMotors &motores, const uint_fast8_t &, const uint_fast8_t &, const uint_fast8_t &);
void checksObstacles(VespaServo &, std::stack<uint_fast8_t> &);
int ultrasonicSensor();
void removeStack(std::stack<uint_fast8_t> &pilha);

#endif