#ifndef CAMERAPIONEER_H
#define CAMERAPIONEER_H

// Qt
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
#include <QImage>
#include <QPixmap>

// OpenCV
#include <opencv2/opencv.hpp>

// Zbar
#include <zbar.h>

//Frodo
#include "MatToQImage.h"

using namespace cv;
using namespace zbar;
using namespace std;

extern Mat              frame_pioneer;
extern VideoCapture     cap_pioneer;
extern QImage           qframe_pioneer;
extern int              pioneer_landmark;
extern double           readinglaser[181];        // Leitura do laser. Realizada na Thread PioneerNavigation
extern QMutex           semaforo_leitura_laser;   // Semáforo para sincronizar a variável de leitura do laser
extern QMutex           semaforo_robo;            // Semáforo para sincronizar a leitura e escrita no robo
extern QMutex           semaforo_landmark;        // Senáforo para sincronizar a leitura e escrita da landmark
extern double           phi;                      // Orientação do robô
extern ArRobot          robot;                    // Robo Aria
extern int              x_landmark;               // Coordenada x da landmark
extern int              y_landmark;               // Coordenada y da landmark
extern bool             vai_fifi;

class CameraPioneer : public QThread
{
    Q_OBJECT
public:
    explicit CameraPioneer(QObject *parent = 0);
    void run();
    enum Mode { Estado_1, Estado_2, Estado_3, Estado_4 };  //Estados da máquina de estados para detecção da landmark

signals:
    void PrintImagePioneer();
    void PioneerCameraOFF();
    void PrintLandMarkPionnerAtual();
    
public slots:

    void setMode(Mode mode);            // Seta o modo de operação da máquina de estados para detecção da landmark

private:
    ImageScanner
    scanner;            // Create a zbar reader

    Mat
    frame_grayscale;    // Matriz para imagem em escala de cinza

    uchar
    *raw;               // Dados da imagem

    int
    contador,           // Contador para confirmar detecção das landmarks
    i,j,                // Contadores para loops
    offset,             // Offset para gerar vetor de leitura do laser
    width,height,       // Dados da imagem
    inicio,fim,         // Posições do vetor de medidas onde foi identificada a lankmark
    posicao;            // Posição da landmark medida pelo laser
//    erro_x,             // Erro calculado na direção x
//    erro_y;             // Erro calcualdo na direção y

    double
    angle,              // Posição da landmark reconehcida pela câmera
    laser[71],          // Vetor com as medidas do laser
    dist_to_landmark,   // Distância do robô à landmark detectada
    ang_to_landmark,    // Ângulo entre a orientação do robô e a landmark
    PI = 3.14159265,    // Constante PI
    erro_angular,       // Erro angular entre o calculado pelo laser e robô
    erro_modulo,
    x_land_calculado,   // Coordenada x da landmark, calculada
    y_land_calculado,   // Coordenada y da landmark, calculada
    erro_x,
    erro_y,
    x_robo_inicial,
    y_robo_inicial,
    x_robo_real,
    y_robo_real;

    bool
    LandmarkDetected;   // Flag que indica detecção da landmark pelo laser

    Mode
    myMode;             // Modo da máquina de estados para detecção da landmark

    ArPose
    posicao_corrigida;  // Posicao corrigida do robô

};

#endif // CAMERAPIONEER_H
