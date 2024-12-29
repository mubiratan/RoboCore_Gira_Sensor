#include <Arduino.h>
#include <stack>
#include "AntiColisao.hpp"
#include "RoboCore_Vespa.h"
#include "Helper.hpp"

VespaServo servo;
VespaMotors motores;

void removePilha(std::stack<uint_fast8_t> &pilha);

enum RobotState {
    MOVING_FORWARD,
    CHECKING_OBSTACLE,
    AVOIDING_OBSTACLE
};

RobotState currentState = MOVING_FORWARD;
unsigned long previousMillis = 0;
const long interval = ESPERA;
std::stack<uint_fast8_t> pilha{};
bool sensor = true;
uint_fast8_t sensor_angulo[] {90, 80, 70, 60, 90, 100, 110, 120, 130};
uint_fast8_t count = 0;
constexpr uint_fast8_t size_sensor_angulo = sizeof(sensor_angulo) / sizeof(sensor_angulo[0]);

void setup_anti_colisao() {
    pinMode(PINO_TRIGGER, OUTPUT);
    pinMode(PINO_ECHO, INPUT);
    digitalWrite(PINO_TRIGGER, LOW);
    servo.attach(VESPA_SERVO_S4, SERVO_MIN, SERVO_MAX);
}

void loop_anti_colisao() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        switch (currentState) {
            case MOVING_FORWARD:
                motores.forward(VELOCIDADE);
                if (sensor_ultrassonico() <= DISTANCIA_OBSTACULO) {
                    motores.stop();
                    currentState = CHECKING_OBSTACLE;
                }
                break;

            case CHECKING_OBSTACLE:
                servo.write(sensor_angulo[count]);
                if (sensor_ultrassonico() <= DISTANCIA_OBSTACULO) {
                    motores.stop();
                    verificaObstaculos(servo, pilha);
                    currentState = AVOIDING_OBSTACLE;
                }
                ++count;
                if (count == size_sensor_angulo) count = 0;
                break;

            case AVOIDING_OBSTACLE:
                if (pilha.size() >= 2) {
                    removePilha(pilha);
                    andaParaTras(motores, ESPERA_MOVIMENTO);
                    verificaObstaculos(servo, pilha);
                } else if (pilha.empty()) {
                    if (millis() % 2 == 0) {
                        giraRobo(motores, VELOCIDADE, -VELOCIDADE, ROTACIONA_90);
                    } else {
                        giraRobo(motores, -VELOCIDADE, VELOCIDADE, ROTACIONA_90);
                    }
                    motores.forward(VELOCIDADE);
                    currentState = MOVING_FORWARD;
                } else if (pilha.top() == OBSTACULO_ESQUERDA) {
                    removePilha(pilha);
                    giraRobo(motores, VELOCIDADE, -VELOCIDADE, ROTACIONA_90);
                } else {
                    removePilha(pilha);
                    giraRobo(motores, -VELOCIDADE, VELOCIDADE, ROTACIONA_90);
                }
                break;
        }
    }
}