#ifndef PIONEERNAVIGATION_H
#define PIONEERNAVIGATION_H

// Bibliotecas do sistema
#include <QMainWindow>
#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <math.h>
#include "Aria.h"
#include <QGraphicsItem>
#include <QObject>
#include <QTimer>
#include <QThread>

// Bibliotecas criadas pelo Frodo
#include "myscene.h"

// Variáveis criadas em LoadMap.h
extern cord                 *coordenada;     //Coordenadas x y das landmarks
extern int                  **mapa;          //Mapa com os pesos e as conexões
extern int                  dim;             //Dimensão do mapa
extern int                  origem;          //Origem do Pionner
extern int                  alvo;            //Destino do Pionner
extern ArRobot              robot;           //Robo Aria
extern ArSick               laser;           //Laser
extern int                  *caminho;        //Vetor com o caminho de menor peso
extern int                  tamanho;         //Quantidade de nós entre origem e destino, inclusive
extern int                  d;               //Diâmetro das landmarks
extern MyScene              *scene;          //Cena do mapa
extern MyScene              *scene2;         //Cena do video do pioneer
extern QGraphicsEllipseItem **elipse;        //Vetor com as landmarks e robo

class PioneerNavigation : public QThread
{
    Q_OBJECT
public:
    explicit PioneerNavigation(QObject *parent = 0);
    void run();
    bool stop;


signals:
    void RedefinirRota();

    void MoverRobo();

    
public slots:


private slots:

    double  calcalpha(double orient, double erdest);

    double sign(double phi);


protected:

    void navigation();

private:

    // Dentro do botão iniciar;


    const std::list<ArSensorReading *> *readingsList;
    std::list<ArSensorReading *>::const_iterator it;

    bool    RecalcularRota;
    int     Obstaculo;


    double reading, readingAngle, reading2, readingAngle2, readinglaser[181],                  //To hold minimum reading and angle
    contador,
    v,
    vr,
    w,
    wr,
    xv,
    yv,
    rho,
    theta,
    phi,
    alpha,
    zeta,
    kv,
    kw,
    PI;
    
};

#endif // PIONEERNAVIGATION_H
