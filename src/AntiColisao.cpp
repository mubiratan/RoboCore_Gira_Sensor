#include <stack>
#include "AntiColisao.hpp"
#include "RoboCore_Vespa.h"
#include "Helper.hpp"

VespaServo servo;
VespaMotors motores;

void removePilha(std::stack<uint_fast8_t> &pilha);

void setup_anti_colisao() {
    pinMode(PINO_TRIGGER, OUTPUT); //configuracao do pino trigger como saida
    pinMode(PINO_ECHO, INPUT); //configuracao do pino echo como entrada
    digitalWrite(PINO_TRIGGER, LOW); //inicia o pino trigger com o nivel logico baixo

    // Servo motor
    servo.attach(VESPA_SERVO_S4, SERVO_MIN, SERVO_MAX);
}

void loop_anti_colisao() {
    // Pilha 
    std::stack<uint_fast8_t> pilha{};
    bool sensor = true;
    uint_fast8_t sensor_angulo[] = {90, 80, 70, 60, 90, 100, 110, 120};
    uint_fast8_t count = 0;
    constexpr uint_fast8_t size_sensor_angulo = sizeof(sensor_angulo) / sizeof(sensor_angulo[0]);

    while(true) 
    {
        delay(ESPERA);
        motores.forward(VELOCIDADE); 
        
        // Gira sensor nos ângulos da variável sensor_angulo
        while(sensor)
        {
            servo.write(sensor_angulo[count]);
            delay(ESPERA);
            if (sensor_ultrassonico() <= DISTANCIA_OBSTACULO)
            {
                motores.stop();
                sensor = false;
                break;
            }
            
            ++count;
            if(count == size_sensor_angulo)
                count = 0;            
        }

        motores.stop();            
        delay(ESPERA); //aguarda o tempo de espera para leitura do sensor

        //confirma se a distancia lida pelo sensor e menor ou igual ao valor configurado na variavel "DISTANCIA_OBSTACULO"
        if (sensor_ultrassonico() <= DISTANCIA_OBSTACULO) 
        {
            motores.stop();
            delay(ESPERA);          
            verificaObstaculos(servo, pilha);

            // Enquanto tiver obstáculos dos lados, continua andanda para trás
            while (pilha.size() >= 2) {
                removePilha(pilha);
                andaParaTras(motores, ESPERA_MOVIMENTO);
                delay(200);
                verificaObstaculos(servo, pilha);
            }

            // Se a pilha estiver vazia, ou seja, sem obstáculos dos lados
            if (pilha.empty()) {
                if (millis() % 2 == 0) { // Gira para direita
                    delay(ESPERA_GIRO);
                    giraRobo(motores, VELOCIDADE, -VELOCIDADE, ROTACIONA_90);
                    motores.forward(VELOCIDADE);
                } else { // Gira para esquerda
                    delay(ESPERA_GIRO);
                    giraRobo(motores, -VELOCIDADE, VELOCIDADE, ROTACIONA_90);
                    delay(ESPERA);
                }    
            } else if(pilha.top() == OBSTACULO_ESQUERDA) {
                delay(ESPERA_GIRO);
                removePilha(pilha);
                giraRobo(motores, VELOCIDADE, -VELOCIDADE, ROTACIONA_90);
                delay(200);   
            } else {
                delay(ESPERA_GIRO);
                removePilha(pilha);
                giraRobo(motores,-VELOCIDADE, VELOCIDADE, ROTACIONA_90);
                delay(200);
            }                
        }

        sensor = true;
    }
}