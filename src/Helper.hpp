#ifndef HELPER_HPP
#define HELPER_HPP

#include <stack>
#include <array>
#include "RoboCore_Vespa.h"

//declaracao dos pinos do sensor
const uint_fast8_t PINO_TRIGGER = 25;
const uint_fast8_t PINO_ECHO = 26;
//declaracao da variavel que armazena a distancia do obstaculo
const uint_fast8_t DISTANCIA_OBSTACULO = 25;
const uint_fast8_t DISTANCIA_OBSTACULO_LONGE = 40;
//declaracao das variaveis que armazenam a velocidade de giro e de reta dos motores do robo
const uint_fast8_t VELOCIDADE = 80;
//declaracao da variavel que armazena o tempo de espera entre leituras do sensor
const uint_fast8_t ESPERA = 100;
const uint_fast8_t ESPERA_GIRO = 200;
//declaracao da variavel que armazena o tempo que os movimentos de desvio irao durar
const unsigned int ESPERA_MOVIMENTO = 1000;
const unsigned int ROTACIONA_90 = ESPERA_MOVIMENTO;
constexpr unsigned int ROTACIONA_180 = ESPERA_MOVIMENTO * 2;

// Status na Pilha
const uint_fast8_t OBSTACULO_FRENTE = 0, OBSTACULO_DIREITA = 1, OBSTACULO_ESQUERDA = 2;

static uint_fast8_t angulo[] = {0, 30, 150, 180};

//Servo
const uint16_t SERVO_MAX = 2200;
const uint16_t SERVO_MIN = 700;
const uint_fast8_t ESPERA_GIRO_SENSOR = 300;
const uint_fast8_t ESPERA_GIRO_SENSOR_MENOR = 150;

// Funcoes
void andaParaTras(VespaMotors &, const uint_fast8_t espera);
void giraRobo(VespaMotors &motores, const uint_fast8_t &, const uint_fast8_t &, const uint_fast8_t &);
void verificaObstaculos(VespaServo &, std::stack<uint_fast8_t> &);
int sensor_ultrassonico();
void removePilha(std::stack<uint_fast8_t> &pilha);

#endif