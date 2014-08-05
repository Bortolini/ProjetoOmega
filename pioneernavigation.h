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
#include <QtCore>
#include <iostream>
#include <string>
#include <fstream>


// Bibliotecas criadas pelo Frodo
#include "myscene.h"
#include "CameraPioneer.h"

using namespace std;


// Variáveis criadas em outras Threads
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
extern CameraPioneer        *PioneerCamera;  //Thread para processamento das imagens do Pioneer
extern bool                 PontoAtingido;   //Flag para indicar que o Pioneer atingiu a landmark

class PioneerNavigation : public QThread
{
    Q_OBJECT
public:
    explicit PioneerNavigation(QObject *parent = 0);
    void run();
    enum Mode { Estado_1, Estado_2, Estado_3, Estado_4 };  //Estados da máquina de estados para detecção da landmark

signals:
    void RedefinirRota();
    void MoverRobo();
    void PioneerCameraOFF();
    void ObjetivoAlcancado();

public slots:


private slots:

    double  calcalpha(double orient, double erdest);
    double  sign(double phi);
    void    setMode(Mode mode);            // Seta o modo de operação da máquina de estados para detecção da landmark


protected:

    void navigation();
    void controlador( ArPose posicao , ArPose destino );        //Controlador de Posição Final
    void controlador_giro( ArPose posicao , ArPose destino );   //Controlador de giro corrigir a direção de navegação para a próxima landmark


private:

    Mode ModoOperacao;                  //Seta o modo de operação

    // Dentro do botão iniciar;

    const std::list<ArSensorReading *> *readingsList;
    std::list<ArSensorReading *>::const_iterator it;

    bool
    RecalcularRota;

    int
    Obstaculo,
    i;

    bool
    atualizar_landmark_pioneer;

    double reading, readingAngle, reading2, readingAngle2,    //To hold minimum reading and angle
    contador,
    v,
    vr,
    w,
    wr,
    xv,
    yv,
    zeta,
    rho,
    kv,
    kw,
    PI = 3.14159265,
    theta,
    alpha;

    Mode
    myMode;             // Modo da máquina de estados para detecção da landmark

    //Para imprimir no arquivo
    ofstream*     salvar;

};

#endif // PIONEERNAVIGATION_H

