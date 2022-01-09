# anti-colisao-verifica-lados

Foi escrito para o hardware ESP32 compatível com a arquitetura Arduino.

O programa foi escrito em C++ usando o plugin PlatformIO no VSCode e a biblioteca "RoboCore_Vespa".

1) Robô inicia andando para a frente;
2) Se achar um obstáculo, para, vira para esquerda e direita, se encontrar obstáculos grava na stack;
3) Se tiver obstáculos nos dois lados, o robô deve andar para traś por um tempo definido e verificar novamente se existem obstáculos;
4) Se encontrar apenas um obstáculo, segue para o lado oposto, ex: encontrou na direita, gira em 180 graus e segue para esquerda.
5) Se ambos os lados não tiverem obstáculos, faz um sorteio de qual lado seguir.
