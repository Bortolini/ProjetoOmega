#ifndef QUADROTORNAVIGATION_H
#define QUADROTORNAVIGATION_H

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
#include <iostream>


class QuadrotorNavigation : public QThread
{
    Q_OBJECT
public:
    explicit QuadrotorNavigation(QObject *parent = 0);
    void run();

signals:
    void PrintLandMarkQuadrotorAtual();

public slots:
    
};

#endif // QUADROTORNAVIGATION_H
