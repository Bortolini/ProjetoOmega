#ifndef PIONEERVIEW_H
#define PIONEERVIEW_H

//Qt
#include <QWidget>

//OpenCv
#include "opencv2/opencv.hpp"

//Frodo
#include "CameraPioneer.h"
#include "MatToQImage.h"
#include "myscene.h"

using namespace cv;

namespace Ui {
class PioneerView;
}

class PioneerView : public QWidget
{
    Q_OBJECT
    
public:
    explicit PioneerView(QWidget *parent = 0);
    ~PioneerView();

private:
    Ui::PioneerView *ui;

private slots:

    void ProcessingImageFromPioneer();

signals:

};

#endif // PIONEERVIEW_H
