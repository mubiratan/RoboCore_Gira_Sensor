/********************************************************
 * RoboCore - Kit Robo Explorer - Robo Anticolisao
 * 
 * O robo fica constantemente medindo a distancia do
 * sensor ultrassonico e, quando a distancia for menor que 
 * a distancia de obstaculo configurada, o robo ira girar
 * para a direita ou para a esquerda, desviando do 
 * obstaculo e voltando a andar para frente.
********************************************************/

#include "AntiColisao.h"

AntiColisao antiColisao;

void setup() {  
  antiColisao.setup_anti_colisao();
}

void loop() {
  antiColisao.loop_anti_colisao();  
}