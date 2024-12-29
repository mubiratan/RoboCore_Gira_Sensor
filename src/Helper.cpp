#include <stack>
#include "Helper.hpp"

// Walk backwards
void walkBackwards(VespaMotors &motores, const uint_fast8_t espera_movimento)
{
    motores.stop(); // stop the robot's motors
    delay(WAIT);
    motores.backward(SPEED); // move the robot backwards by turning the motors backwards
    delay(espera_movimento); // maintain the robot's movement
    motores.stop(); // stop the robot's motors
}

// Spin the robot to the sides
void spinsRobot(VespaMotors &motores, const uint_fast8_t &velocidade, const uint_fast8_t &velocidade2, const uint_fast8_t &tempo_espera)
{
    delay(WAIT);
    motores.turn(velocidade, velocidade2);
    delay(tempo_espera); // maintain the robot's movement
    motores.stop(); // stop the robot's motors
    delay(WAIT);
}

// Spin the robot to the sides and check for obstacles
void checksObstacles(VespaServo &servo, std::stack<uint_fast8_t> &pilha)
{
    removeStack(pilha);
    uint_fast8_t contadorDireita = 0;
    uint_fast8_t contadorEsquerda = 0;

    for(int x = 0; x < sizeof(ANGLES) / sizeof(ANGLES[0]); x++)
    {
        // Spin Sensor
        servo.write(ANGLES[x]);
        delay(WAIT_TURN_SENSOR);

        delay(WAIT);
        if (ultrasonicSensor() <= OBSTACLE_DISTANCE)
        {
            if(ANGLES[x] < 90 && contadorDireita == 0)
            {
                pilha.push(RIGHT_OBSTACLE);
                ++contadorDireita;
            }
            else if (ANGLES[x] > 90 && contadorEsquerda == 0)
            {
                pilha.push(LEFT_OBSTACLE);
                ++contadorEsquerda;
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

void removeStack(std::stack<uint_fast8_t> &pilha)
{
    for(int x = 0; x < pilha.size(); x++)
        pilha.pop();
}