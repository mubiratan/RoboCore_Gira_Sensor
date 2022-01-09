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
void giraRobo(VespaMotors &motores, const uint_fast8_t velocidade, const uint_fast8_t velocidade2, const uint_fast8_t tempo_espera)
{   
    delay(ESPERA);
    motores.turn(velocidade, velocidade2);
    delay(tempo_espera); //matem o movimento do robo
    motores.stop(); //para os motores do robo
    delay(ESPERA);
}