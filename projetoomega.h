#ifndef PROJETOOMEGA_H
#define PROJETOOMEGA_H

//Qt
#include <QMainWindow>
#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <math.h>
#include "Aria.h"
#include <QGraphicsItem>
#include <QObject>
#include <QTimer>

//OpenCV
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;

//Frodo
#include "myscene.h"
#include "ui_projetoomega.h"
#include "pioneernavigation.h"
#include "PioneerView.h"
#include "QuadrotorView.h"
#include "CameraPioneer.h"
#include "CameraQuadrotor.h"
#include "QuadrotorNavigation.h"


// Variáveis criadas em LoadMap.h

extern cord                 *coordenada;            //Coordenadas x y das landmarks
extern int                  **mapa;                 //Mapa com os pesos e as conexões
extern int                  dim;                    //Dimensão do mapa
extern int                  origem;                 //Origem do Pionner
extern int                  alvo;                   //Destino do Pionner
extern ArRobot              robot;                  //Robo Aria
extern ArSick               laser;                  //Laser
extern int                  *caminho;               //Vetor com o caminho de menor peso
extern int                  tamanho;                //Quantidade de nós entre origem e destino, inclusive
extern CameraPioneer        *PioneerCamera;         //Thread da câmera do pioneer
extern CameraQuadrotor      *QuadrotorCamera;       //Thread da câmera do Quadrotor
extern QuadrotorNavigation  *NavigationQuadrotor;   //Thread para Navegação do Quadrotor
extern int                  pioneer_landmark;       //landmark procurada pelo pioneer
extern QMutex               semaforo_robo;          // Semáforo para sincronizar a leitura e escrita no robo

extern  bool    mari;

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

    void PioneerCameraOFF();

    void QuadrotorCameraOFF();

    void PrintLandMarkPionnerAtual();

    void PrintLandMarkQuadrotorAtual();

    void initializeDistance(bool marker[], int predecessor[], int distance[]);

    int getClosestUnmarkedNode(bool marker[], int distance[]);

    void dijkstra(bool marker[], int distance[], int predecessor[]);

    void printPath( int destination, int predecessor[], int caminho[], int &num);

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();    //  rota = new QGraphicsLineItem*[tamanho];

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

    void ObjetivoAlcancado();

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
