#include <stack>
#include "Helper.hpp"

// Walk backwards
void walkBackwards(VespaMotors &motors, const uint_fast8_t waitMovement)
{
    motors.stop();
    delay(WAIT);
    motors.backward(SPEED);
    delay(waitMovement);
    motors.stop();
}

// Spin the robot to the sides
void spinsRobot(VespaMotors &motors, const uint_fast8_t &speed, const uint_fast8_t &speed2, const uint_fast8_t &waitingTime)
{
    delay(WAIT);
    motors.turn(speed, speed2);
    delay(waitingTime);
    motors.stop();
    delay(WAIT);
}

// Spin the robot to the sides and check for obstacles
void checksObstacles(VespaServo &servo, std::stack<uint_fast8_t> &stack)
{
    removeStack(stack);
    uint_fast8_t RIGHT_COUNTER = 0;
    uint_fast8_t LEFT_COUNTER = 0;

    for(int x = 0; x < sizeof(ANGLES) / sizeof(ANGLES[0]); x++)
    {
        // Spin Sensor
        servo.write(ANGLES[x]);
        delay(WAIT_TURN_SENSOR);

        delay(WAIT);
        if (ultrasonicSensor() <= OBSTACLE_DISTANCE)
        {
            if(ANGLES[x] < 90 && RIGHT_COUNTER == 0)
            {
                stack.push(RIGHT_OBSTACLE);
                ++RIGHT_COUNTER;
            }
            else if (ANGLES[x] > 90 && LEFT_COUNTER == 0)
            {
                stack.push(LEFT_OBSTACLE);
                ++LEFT_COUNTER;
            }
        }
    }

    // Return to center
    delay(WAIT_TURN_SENSOR);
    servo.write(90);
}

// Function to read the sensor
int ultrasonicSensor()
{
    // Perform a 10 microsecond pulse on the sensor's trigger
    digitalWrite(PINO_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(PINO_TRIGGER, LOW);

    // Measure the pulse in microseconds returned to the sensor's echo
    // and convert the time to distance by dividing by 58
    return pulseIn(PINO_ECHO, HIGH) / 58;
}

void removeStack(std::stack<uint_fast8_t> &stack)
{
    const uint_fast8_t size = stack.size();
    for(int x = 0; x < size; ++x)
        stack.pop();
}