#include <stack>
#include "AntiColisao.hpp"
#include "Helper.hpp"
#include "RoboCore_Vespa.h"

VespaServo servo;
VespaMotors motores;

void setup_anti_colisao() {
    pinMode(PINO_TRIGGER, OUTPUT); //configuracao do pino trigger como saida
    pinMode(PINO_ECHO, INPUT); //configuracao do pino echo como entrada
    digitalWrite(PINO_TRIGGER, LOW); //inicia o pino trigger com o nivel logico baixo

    // Servo motor
    servo.attach(VESPA_SERVO_S4, SERVO_MIN, SERVO_MAX);
}

void loop_anti_colisao() {
    // Ponteiro pra função
    int (*pSensor)() = &sensor_ultrassonico;
    void (*pVerificaObstaculos)(VespaServo &, std::stack<uint_fast8_t> &) = &verificaObstaculos;
    
    // Pilha 
    std::stack<uint_fast8_t> pilha{};

    int distancia  = 0;
    
    while(true) 
    {
        delay(ESPERA); //aguarda o tempo de espera para leitura do sensor
        distancia = pSensor(); //armazena a distancia lida pelo sensor a variavel distancia

        //verifica se a distancia lida pelo sensor e menor ou igual ao valor configurado na variavel "DISTANCIA_OBSTACULO"
        if (distancia <= DISTANCIA_OBSTACULO) //se for verdadeiro
        {            
            delay(ESPERA); //aguarda o tempo de espera para leitura do sensor
            distancia = pSensor(); //atualiza a leitura do sensor

            //confirma se a distancia lida pelo sensor e menor ou igual ao valor configurado na variavel "DISTANCIA_OBSTACULO"
            if (distancia <= DISTANCIA_OBSTACULO) 
            {
                motores.stop();
                delay(ESPERA);          
                pVerificaObstaculos(servo, pilha);

                // Enquanto tiver 2 obstáculos dos lados, continua andanda para trás
                while (pilha.size() == 2) {
                    pilha.pop();
                    pilha.pop();
                    andaParaTras(motores, ESPERA_MOVIMENTO);
                    delay(ESPERA);
                    pVerificaObstaculos(servo, pilha);
                }

                // Se a pilha estiver vazia, ou seja, sem obstáculos dos lados
                if (pilha.empty()) {
                    if (millis() % 2 == 0) { // Gira para direita
                        delay(ESPERA);
                        pilha.pop();
                        giraRobo(motores, VELOCIDADE, -VELOCIDADE, ROTACIONA_90);
                        motores.forward(VELOCIDADE);
                    } else { // Gira para esquerda
                        delay(ESPERA);
                        pilha.pop();
                        giraRobo(motores, -VELOCIDADE, VELOCIDADE, ROTACIONA_90);
                        delay(ESPERA);
                        motores.forward(VELOCIDADE);
                    }    
                } else if(pilha.top() == OBSTACULO_ESQUERDA) {
                    delay(ESPERA);
                    pilha.pop();
                    giraRobo(motores, VELOCIDADE, -VELOCIDADE, ROTACIONA_90);
                    delay(ESPERA);
                    motores.forward(VELOCIDADE);     
                } else {
                    delay(ESPERA);
                    pilha.pop();
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