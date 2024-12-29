#include <Arduino.h>
#include <stack>
#include "AntiColisao.hpp"
#include "RoboCore_Vespa.h"
#include "Helper.hpp"

VespaServo servo;
VespaMotors engines;

enum RobotState {
    MOVING_FORWARD,
    CHECKING_OBSTACLE,
    AVOIDING_OBSTACLE
};

std::stack<uint_fast8_t> move_stack{};

unsigned long previousMillis = 0;
const long interval = WAIT;
bool sensor = true;

uint_fast8_t sensor_angle[] {90, 80, 70, 60, 90, 100, 110, 120, 130};
uint_fast8_t count = 0;
constexpr uint_fast8_t size_sensor_angle = sizeof(sensor_angle) / sizeof(sensor_angle[0]);

RobotState currentState = MOVING_FORWARD;

void setup_anti_collision() {
    pinMode(PINO_TRIGGER, OUTPUT);
    pinMode(PINO_ECHO, INPUT);
    digitalWrite(PINO_TRIGGER, LOW);
    servo.attach(VESPA_SERVO_S4, SERVO_MIN, SERVO_MAX);
}

void loop_anti_collision() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        switch (currentState) {
            case MOVING_FORWARD:
                engines.forward(SPEED);
                if (ultrasonicSensor() <= OBSTACLE_DISTANCE) {
                    engines.stop();
                    currentState = CHECKING_OBSTACLE;
                }
                break;

            case CHECKING_OBSTACLE:
                servo.write(sensor_angle[count]);
                if (ultrasonicSensor() <= OBSTACLE_DISTANCE) {
                    engines.stop();
                    checksObstacles(servo, move_stack);
                    currentState = AVOIDING_OBSTACLE;
                }
                ++count;
                if (count == size_sensor_angle) count = 0;
                break;

            case AVOIDING_OBSTACLE:
                if (move_stack.size() >= 2) {
                    removeStack(move_stack);
                    walkBackwards(engines, WAIT_MOTION);
                    checksObstacles(servo, move_stack);
                } else if (move_stack.empty()) {
                    if (millis() % 2 == 0) {
                        spinsRobot(engines, SPEED, -SPEED, ROTATION_90);
                    } else {
                        spinsRobot(engines, -SPEED, SPEED, ROTATION_90);
                    }
                    engines.forward(SPEED);
                    currentState = MOVING_FORWARD;
                } else if (move_stack.top() == LEFT_OBSTACLE) {
                    removeStack(move_stack);
                    spinsRobot(engines, SPEED, -SPEED, ROTATION_90);
                } else {
                    removeStack(move_stack);
                    spinsRobot(engines, -SPEED, SPEED, ROTATION_90);
                }
                break;
        }
    }
}