/*****************************************************************************************/
/*****************************************************************************************/
/************************** Função para carregar o mapa **********************************/
/*****************************************************************************************/
/*****************************************************************************************/

// O mapa deve ser um arquivo de texto organizado da seguinte forma:

//        4
//        0 12 999 5 0 0
//        12 0 999 999 1 4
//        999 0 7 999 5 4
//        5 7 999 0 8 8

// O primeiro número indica a dimensão do mapa (quantidade de landmarks).
// A matriz 4x4 abaixo da dimensão são os pesos e as conexões entre as landmarks.
// O vetor 4x2, a direita da matriz, indica a as coordenadas das landmarks.
// O número 999 representa peso infinito, e o valor 0 indica a ligação do ponto com ele mesmo.

#ifndef LOADMAP_H
#define LOADMAP_H

#endif // LOADMAP_H

// Bibliotecas do sistema
#include <QtCore/QFile>
#include <iostream>

using namespace std;

typedef struct{

    int x;
    int y;

}cord;

void loadMapFile();
