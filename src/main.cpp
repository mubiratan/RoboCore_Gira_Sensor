/********************************************************
* RoboCore - Kit Robo Explorer - Anti-Collision Robot
 *
 * The robot constantly measures the distance from the
 * ultrasonic sensor and, when the distance is less than
 * the configured obstacle distance, the robot will turn
 * to the right or left, avoiding the obstacle and
 * continuing to move forward.
********************************************************/

#include "AntiColisao.hpp"
#include "Helper.hpp"

TaskHandle_t Task1;
VespaBattery vbat;

/** Task on separate core
**  If the battery voltage drops below 5V, the LED will blink
**/
void Task1code( void * pvParameters ) {
  uint8_t capacity;

  while(true)
  {
    capacity = (vbat.readVoltage() * 100 / 9000);

    if(capacity < 5.0)
    {
      while(true)
      {
        digitalWrite(LED_VESPA, HIGH);
        delay(300);
        digitalWrite(LED_VESPA, LOW);
        delay(300);
      }
    }
    delay(TIME_UPDATE_VBAT);
  }
}

void setup() {
  // Task setup for core 0 to check battery
  pinMode(LED_VESPA, OUTPUT);

  xTaskCreatePinnedToCore(
                  Task1code,   /* Task function. */
                  "Task1",     /* name of task. */
                  10000,       /* Stack size of task */
                  NULL,        /* parameter of the task */
                  0,           /* priority of the task */
                  &Task1,      /* Task handle to keep track of created task */
                  0);          /* pin task to core 0 */
  delay(500);

  // Main program setup
  setup_anti_collision();
}

// Call main loop
void loop() {
  loop_anti_collision();
}