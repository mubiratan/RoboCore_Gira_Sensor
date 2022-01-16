#include <stack>
#include "Helper.hpp"

// Anda para trás
void andaParaTras(VespaMotors &motores, const uint_fast8_t espera_movimento)
{
    // Anda para trás
    motores.stop(); //para os motores do robo
    delay(ESPERA);
    motores.backward(VELOCIDADE); //recua o robo girando os motores para tras
    delay(espera_movimento); //matem o movimento do robo
    motores.stop(); //para os motores do robo
}

// Gira o robô para os lados
void giraRobo(VespaMotors &motores, const uint_fast8_t &velocidade, const uint_fast8_t &velocidade2, const uint_fast8_t &tempo_espera)
{   
    delay(ESPERA);
    motores.turn(velocidade, velocidade2);
    delay(tempo_espera); //matem o movimento do robo
    motores.stop(); //para os motores do robo
    delay(ESPERA);
}

// Gira o robô para os lados e verifica se tem obstáculos 
void verificaObstaculos(VespaServo &servo, std::stack<uint_fast8_t> &pilha) 
{
    removePilha(pilha);
    uint_fast8_t contadorDireita = 0;
    uint_fast8_t contadorEsquerda = 0;

    for(int x = 0; x < sizeof(angulo) / sizeof(angulo[0]); x++)
    {
        // Gira Sensor
        servo.write(angulo[x]);
        delay(ESPERA_GIRO_SENSOR);
        
        delay(ESPERA);
        if (sensor_ultrassonico() <= DISTANCIA_OBSTACULO)
        {
            if(angulo[x] < 90 && contadorDireita == 0) 
            {
                pilha.push(OBSTACULO_DIREITA);
                ++contadorDireita;
            }   
            else if (angulo[x] > 90 && contadorEsquerda == 0)
            {
                pilha.push(OBSTACULO_ESQUERDA);
                ++contadorEsquerda;
            }
        }
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

void removePilha(std::stack<uint_fast8_t> &pilha)
{
    for(int x = 0; x < pilha.size(); x++)
        pilha.pop();
}