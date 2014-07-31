#include "PioneerView.h"
#include "ui_PioneerView.h"

MyScene         *scene1;
CameraPioneer   *PioneerCamera;
VideoCapture    cap_pioneer(1);
Mat             frame_pioneer;
QImage          qframe_pioneer;

PioneerView::PioneerView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PioneerView)
{
    ui->setupUi(this);

    scene1 = new MyScene(this);
    ui->graphicsView->setScene(scene1);

    //Criando a Thread para a câmera do Pioneer
    PioneerCamera = new CameraPioneer(this);

    //Sinais provenientes da Thread PioneerNavigation
    connect(PioneerCamera, SIGNAL(PrintImagePioneer()), this, SLOT(ProcessingImageFromPioneer()));



}

void PioneerView::ProcessingImageFromPioneer()
{

    qframe_pioneer = MatToQImage(frame_pioneer);
    ui->graphicsView->scene()->clear();
    ui->graphicsView->scene()->addPixmap(QPixmap::fromImage(qframe_pioneer).scaled(ui->graphicsView->width(),ui->graphicsView->height(),Qt::KeepAspectRatio));

}

PioneerView::~PioneerView()
{
    cap_pioneer.~VideoCapture();

    delete ui;
}
