#include "AntiColisao.hpp"
#include "Helper.hpp"

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
    std::stack<uint_fast8_t> pilha{};
    int distancia;
    
    while(true) 
    {
        delay(ESPERA); //aguarda o tempo de espera para leitura do sensor
        distancia = sensor_ultrassonico(); //armazena a distancia lida pelo sensor a variavel distancia

        //verifica se a distancia lida pelo sensor e menor ou igual ao valor configurado na variavel "DISTANCIA_OBSTACULO"
        if (distancia <= DISTANCIA_OBSTACULO) //se for verdadeiro
        {            
            delay(ESPERA); //aguarda o tempo de espera para leitura do sensor
            distancia = sensor_ultrassonico(); //atualiza a leitura do sensor

            //confirma se a distancia lida pelo sensor e menor ou igual ao valor configurado na variavel "DISTANCIA_OBSTACULO"
            if (distancia <= DISTANCIA_OBSTACULO) 
            {
                motores.stop();
                delay(ESPERA);          
                verificaObstaculos(servo, pilha);

                // Enquanto tiver 2 obstáculos dos lados, continua andanda para trás
                while (pilha.size() == 2) {
                    pilha.pop();
                    pilha.pop();
                    andaParaTras(motores, ESPERA_MOVIMENTO);
                    delay(ESPERA);
                    verificaObstaculos(servo, pilha);
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

// Gira o robô para os lados e verifica se tem obstáculos 
void verificaObstaculos(VespaServo &servo, std::stack<uint_fast8_t> &pilha) 
{
    int distancia;

    // Gira Sensor para direita
    servo.write(0);
    delay(ESPERA_GIRO_SENSOR);
    distancia = sensor_ultrassonico();
    
    if (distancia <= DISTANCIA_OBSTACULO)
    {
        pilha.push(OBSTACULO_DIREITA);
    }

    delay(ESPERA);
    
    // Gira Sensor para esquerda
    servo.write(180);
    delay(ESPERA_GIRO_SENSOR);
    distancia = sensor_ultrassonico();
    
    if (distancia <= DISTANCIA_OBSTACULO)
    {
        pilha.push(OBSTACULO_ESQUERDA);
    }

    // Volta pra o centro
    delay(ESPERA_GIRO_SENSOR);
    servo.write(90);    
}

//funcao para a leitura do sensor
int sensor_ultrassonico() 
{
    //realiza o pulso de 10 microsegundos no trigger do sensor
    digitalWrite(PINO_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(PINO_TRIGGER, LOW);

    //mede o pulso em microsegundos retornado para o echo do sensor
    //e converte o tempo para distancia divindo por 58
    return pulseIn(PINO_ECHO, HIGH) / 58;
}