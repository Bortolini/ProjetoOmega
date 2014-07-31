#include "QuadrotorView.h"
#include "ui_QuadrotorView.h"

MyScene             *scene4;
CameraQuadrotor     *QuadrotorCamera;
QuadrotorNavigation *NavigationQuadrotor;
VideoCapture        cap_quadrotor(2);
Mat                 frame_quadrotor;
QImage              qframe_quadrotor;

QuadrotorView::QuadrotorView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::QuadrotorView)
{
    ui->setupUi(this);

    scene4 = new MyScene(this);
    ui->graphicsView->setScene(scene4);

    //Criando a Thread para a camera do Quadrotor
    QuadrotorCamera = new CameraQuadrotor(this);

    //Criando a Thread para a navegação do Quadrotor
    NavigationQuadrotor = new QuadrotorNavigation(this);

    //Sinais provenientes da Thread PioneerNavigation
    connect(QuadrotorCamera, SIGNAL(PrintImageQuadrotor()), this, SLOT(ProcessingImageFromQuadrotor()));

}

void QuadrotorView::ProcessingImageFromQuadrotor()
{

    qframe_quadrotor = MatToQImage(frame_quadrotor);
    ui->graphicsView->scene()->clear();
    ui->graphicsView->scene()->addPixmap(QPixmap::fromImage(qframe_quadrotor).scaled(ui->graphicsView->width(),ui->graphicsView->height(),Qt::KeepAspectRatio));
    ui->graphicsView->scene()->addText("To na Area!!!!");

}

QuadrotorView::~QuadrotorView()
{
    cap_quadrotor.~VideoCapture();
    delete ui;
}
