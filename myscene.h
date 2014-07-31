#ifndef MYSCENE_H
#define MYSCENE_H

#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QObject>
#include <QGraphicsEllipseItem>
#include <iostream>
#include <Aria.h>

// Bibliotecas criadas pelo Frodo
#include "LoadMap.h"

using namespace std;


extern QGraphicsEllipseItem **elipse;   //Landmarks
extern cord *coordenada;                //Coordenadas x y das landmarks
extern int dim;                         //Dimensão do mapa
extern int origem;                      //Nó de origem
extern int alvo;                        //Destino


class MyScene : public QGraphicsScene
{
    Q_OBJECT

public:

    enum Mode { Inicial, InserirOrigem, InserirDestino };

    MyScene(QObject *parent = 0);

signals:
    void OrigemInserida();
    void DestinoInserido();
    void MovimentarRobo();

public slots:
    void setMode(Mode mode);        // Seta o modo de operação

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent);

private:
    Mode myMode;
    int i;

//  Variáveis do controlador
 double v,
        vr,
        w,
        wr,
        rho,
        theta,
        phi,
        alpha,
        contador,
        kv,
        kw,
        PI;
};

#endif // MYSCENE_H
