/********************************************************
 * RoboCore - Kit Robo Explorer - Robo Anticolisao
 * 
 * O robo fica constantemente medindo a distancia do
 * sensor ultrassonico e, quando a distancia for menor que 
 * a distancia de obstaculo configurada, o robo ira girar
 * para a direita ou para a esquerda, desviando do 
 * obstaculo e voltando a andar para frente.
********************************************************/

#include "AntiColisao.hpp"
#include "Helper.hpp"

TaskHandle_t Task1;
VespaBattery vbat;

const int LED_Vespa = 15;
const uint32_t TEMPO_ATUALIZACAO_VBAT = 60000; // [ms]

/** Tarefa em Core separado
**  Se a bateria ficar com a voltagem abaixo de 5V, o LED irá piscar
**/
void Task1code( void * pvParameters ) {
  uint32_t capacidade;

  while(true)
  {
    capacidade = (vbat.readVoltage() * 100 / 9000);
    if(capacidade < 5.0)
    {
      while(true)
      {
        digitalWrite(LED_Vespa, HIGH);
        delay(300);
        digitalWrite(LED_Vespa, LOW);
        delay(300);
      }
    }
    delay(TEMPO_ATUALIZACAO_VBAT);
  }
}

void setup() {
  // Setup da Task para o core 0 para verificar a bateria
  pinMode(LED_Vespa, OUTPUT);

  xTaskCreatePinnedToCore(
                  Task1code,   /* Task function. */
                  "Task1",     /* name of task. */
                  10000,       /* Stack size of task */
                  NULL,        /* parameter of the task */
                  0,           /* priority of the task */
                  &Task1,      /* Task handle to keep track of created task */
                  0);          /* pin task to core 0 */                  
  delay(500);
  
  // Setup do programa principal
  setup_anti_colisao();
}

// Chama loop principal
void loop() {
  loop_anti_colisao();  
}