#ifndef ANTI_COLISAO_HPP
#define ANTI_COLISAO_HPP

#include "RoboCore_Vespa.h"
#include <stack>

void loop_anti_colisao();
void setup_anti_colisao();

int sensor_ultrassonico();
void verificaObstaculos(VespaServo &, std::stack<uint_fast8_t> &);

#endif