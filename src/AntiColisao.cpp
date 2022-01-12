#include <stack>
#include "AntiColisao.hpp"
#include "Helper.hpp"
#include "RoboCore_Vespa.h"

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
    
    while(true) 
    {
        delay(ESPERA); //aguarda o tempo de espera para leitura do sensor

        //verifica se a distancia lida pelo sensor e menor ou igual ao valor configurado na variavel "DISTANCIA_OBSTACULO"
        if (sensor_ultrassonico() <= DISTANCIA_OBSTACULO) //se for verdadeiro
        {            
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
                    delay(ESPERA);
                    verificaObstaculos(servo, pilha);
                }

                // Se a pilha estiver vazia, ou seja, sem obstáculos dos lados
                if (pilha.empty()) {
                    if (millis() % 2 == 0) { // Gira para direita
                        delay(ESPERA);
                        giraRobo(motores, VELOCIDADE, -VELOCIDADE, ROTACIONA_90);
                        motores.forward(VELOCIDADE);
                    } else { // Gira para esquerda
                        delay(ESPERA);
                        giraRobo(motores, -VELOCIDADE, VELOCIDADE, ROTACIONA_90);
                        delay(ESPERA);
                        motores.forward(VELOCIDADE);
                    }    
                } else if(pilha.top() == OBSTACULO_ESQUERDA) {
                    delay(ESPERA);
                    removePilha(pilha);
                    giraRobo(motores, VELOCIDADE, -VELOCIDADE, ROTACIONA_90);
                    delay(ESPERA);
                    motores.forward(VELOCIDADE);     
                } else {
                    delay(ESPERA);
                    removePilha(pilha);
                    giraRobo(motores,-VELOCIDADE, VELOCIDADE, ROTACIONA_90);
                    delay(ESPERA);
                    motores.forward(VELOCIDADE);     
                }                
            }
        } else { //caso a distancia do sensor não seja menor que o valor "DISTANCIA_OBSTACULO" na primeira verificacao

            motores.forward(VELOCIDADE); //mantem o robo andando para frente
        }
    }
}