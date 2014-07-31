#include "myscene.h"

MyScene::MyScene(QObject *parent) {
    myMode = Inicial;
}

void MyScene::setMode(Mode mode)
{
    myMode = mode;
}

void MyScene::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
{

    if (mouseEvent->button() != Qt::LeftButton)
        return;

    switch (myMode) {
    case InserirOrigem:{
        for ( i = 0 ; i < dim ; i++){
            if (mouseEvent->scenePos().x() > coordenada[i].x &&
                    (mouseEvent->scenePos().x() < (coordenada[i].x+0.05*coordenada[dim].x)  ))

                if (mouseEvent->scenePos().y() > coordenada[i].y &&
                        ( mouseEvent->scenePos().y() < (coordenada[i].y+0.05*coordenada[dim].x))  ){

                    elipse[i]->setBrush(QBrush(Qt::yellow));

                    origem= i;

                    emit OrigemInserida();

                    break;
                }
        }



        break;
    }
    case InserirDestino:{

        for ( i = 0 ; i < dim ; i++){
            if (mouseEvent->scenePos().x() > coordenada[i].x &&
                    (mouseEvent->scenePos().x() < (coordenada[i].x+0.05*coordenada[dim].x)  ))

                if (mouseEvent->scenePos().y() > coordenada[i].y &&
                        ( mouseEvent->scenePos().y() < (coordenada[i].y+0.05*coordenada[dim].x))  ){

                    elipse[i]->setBrush(QBrush(Qt::green));

                    alvo= i;

                    //  setMode(Inicial);

                    setMode(Inicial);
                    emit DestinoInserido();

                    break;
                }
        }

        break;
    }

    default:{
        break;
    }

    }
    QGraphicsScene::mousePressEvent(mouseEvent);
}
