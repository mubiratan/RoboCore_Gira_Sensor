#ifndef ANTI_COLISAO
#define ANTI_COLISAO

//adiciona a biblioteca de controle de motores ao codigo
#include <RoboCore_Vespa.h>
#include <stack>

static std::stack<uint_fast8_t> pilha{};

class AntiColisao 
{
    private:
    //declaracao dos pinos do sensor
    const uint_fast8_t PINO_TRIGGER = 25;
    const uint_fast8_t PINO_ECHO = 26;
    //declaracao da variavel que armazena a distancia do obstaculo
    const uint_fast8_t DISTANCIA_OBSTACULO = 30;
    //declaracao das variaveis que armazenam a velocidade de giro e de reta dos motores do robo
    const uint_fast8_t VELOCIDADE = 80;
    //declaracao da variavel que armazena o tempo de espera entre leituras do sensor
    const uint_fast8_t ESPERA = 150;
    //declaracao da variavel que armazena o tempo que os movimentos de desvio irao durar
    const unsigned int ESPERA_MOVIMENTO = 1000;
    //declaracao da variavel que armazena a distancia medida pelo sensor
    int distancia;
    const unsigned int ROTACIONA_90 = ESPERA_MOVIMENTO;
    const unsigned int ROTACIONA_180 = ESPERA_MOVIMENTO * 2;

    // Pilha
    const uint_fast8_t OBSTACULO_FRENTE = 0, OBSTACULO_DIREITA = 1, OBSTACULO_ESQUERDA = 2;
    //Servo
    const uint16_t SERVO_MAX = 2200;
    const uint16_t SERVO_MIN = 700;
    const uint_fast8_t ESPERA_GIRO_SENSOR = 300;

    public:
    //declaracao do objeto "motores"
    VespaMotors motores;
    VespaServo servo;

    void loop_anti_colisao();
    void setup_anti_colisao();
    void verificaObstaculos();
    inline void giraRobo(const uint_fast8_t, const uint_fast8_t, const uint_fast8_t);
    inline void andaParaTras(const uint_fast8_t espera);
    inline int sensor_ultrassonico();
};

#endif