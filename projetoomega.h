#ifndef PROJETOOMEGA_H
#define PROJETOOMEGA_H

// Bibliotecas do sistema
#include <QMainWindow>
#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <math.h>
#include "Aria.h"
#include <QGraphicsItem>
#include <QObject>
#include <QTimer>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;

// Bibliotecas criadas pelo Frodo
#include "myscene.h"
#include "ui_projetoomega.h"
#include "pioneernavigation.h"



// Variáveis criadas em LoadMap.h

extern cord     *coordenada;    //Coordenadas x y das landmarks
extern int      **mapa;          //Mapa com os pesos e as conexões
extern int      dim;             //Dimensão do mapa
extern int      origem;          //Origem do Pionner
extern int      alvo;            //Destino do Pionner
extern ArRobot  robot;       //Robo Aria
extern ArSick   laser;        //Laser
extern int      *caminho;        //Vetor com o caminho de menor peso
extern int      tamanho;         //Quantidade de nós entre origem e destino, inclusive
extern bool     nav;

namespace Ui {
class ProjetoOmega;
}

class ProjetoOmega : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit ProjetoOmega(QWidget *parent = 0);
    Ui::ProjetoOmega *ui;

    ~ProjetoOmega();

    PioneerNavigation   *PNavigation;

    
private slots:

    void OrigemInserida();

    void DestinoIserido();

    void RecalculandoRota();

    void MovimentandoRobo();

    void initializeDistance(bool marker[], int predecessor[], int distance[]);

    int getClosestUnmarkedNode(bool marker[], int distance[]);

    void dijkstra(bool marker[], int distance[], int predecessor[]);

    void printPath( int destination, int predecessor[], int caminho[], int &num);

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();    //  rota = new QGraphicsLineItem*[tamanho];

    void on_pushButton_4_clicked();


signals:

private:

    int     i, j, *predecessor, *distance;
    double  d_pontos, offset_x, offset_y;
    bool*   marker;
    //QGraphicsLineItem **rota;

    vector<QGraphicsLineItem*>      rota;
    QGraphicsEllipseItem            *robo;
    vector<QGraphicsEllipseItem*>   trilha;

};

#endif // PROJETOOMEGA_H
