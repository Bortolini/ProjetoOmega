#ifndef CAMERAQUADROTOR_H
#define CAMERAQUADROTOR_H

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

//Zbar
#include <zbar.h>

//Frodo
#include "MatToQImage.h"


extern Mat              frame_quadrotor;
extern VideoCapture     cap_quadrotor;

using namespace cv;
using namespace zbar;
using namespace std;

class CameraQuadrotor : public QThread
{
    Q_OBJECT
public:
    explicit CameraQuadrotor(QObject *parent = 0);
    void run();

signals:
    void PrintImageQuadrotor();
    void PrintLandMarkQuadrotorAtual();

public slots:
//    int xioctl(int fd, int request, void *arg);

private:
    ImageScanner    scanner;            // Create a zbar reader
    int             width,height;       // Dados da imagem
    uchar           *raw;               // Dados da imagem
    int             counter;            // Contador de detecção das landmarks
    Mat             frame_hsv,          // Matriz de imagem em HSV
                    frame_grayscale,    // Matriz para imagem em escala de cinza
                    frame_bin,          // Matriz de imagem binarizada
                    frame_bin2;         // Matriz de imagem binarizada 2 - pós filtro
    
};

#endif // CAMERAQUADROTOR_H
