#ifndef QUADROTORVIEW_H
#define QUADROTORVIEW_H

//Qt
#include <QWidget>

//OpenCv
#include "opencv2/opencv.hpp"

//Frodo
#include "CameraQuadrotor.h"
#include "MatToQImage.h"
#include "myscene.h"
#include "QuadrotorNavigation.h"

using namespace cv;

namespace Ui {
class QuadrotorView;
}

class QuadrotorView : public QWidget
{
    Q_OBJECT
    
public:
    explicit QuadrotorView(QWidget *parent = 0);
    ~QuadrotorView();
    
private:
    Ui::QuadrotorView *ui;

private slots:

   void ProcessingImageFromQuadrotor();

};

#endif // QUADROTORVIEW_H
