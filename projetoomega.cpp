#include "projetoomega.h"

QGraphicsEllipseItem **elipse;
MyScene *scene, *scene2;

int d;

ProjetoOmega::ProjetoOmega(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ProjetoOmega)
{
    ui->setupUi(this);

    scene = new MyScene(this);
    scene2 = new MyScene(this);

    ui->graphicsView_2->setScene(scene2);

    ui->graphicsView->setScene(scene);

    ui->graphicsView->setAlignment(Qt::AlignLeft | Qt::AlignBottom);


    //  Sinais provenientes da classe MyScene para informar quando selecinados
    //  os pontos de origem e destino

    connect(scene, SIGNAL(OrigemInserida()),
            this, SLOT(OrigemInserida()));

    connect(scene, SIGNAL(DestinoInserido()),
            this, SLOT(DestinoIserido()));

    PNavigation = new PioneerNavigation(this);
    PNavigation->start();

    //  Sinais provenientes da Thread PioneerNavigation

    connect(PNavigation, SIGNAL(RedefinirRota()),
            this, SLOT(RecalculandoRota()));

    connect(PNavigation, SIGNAL(MoverRobo()),
            this, SLOT(MovimentandoRobo()));

}

void ProjetoOmega::on_pushButton_clicked()
{
    robo = new QGraphicsEllipseItem;
    ui->textEdit->insertPlainText("Carregando mapa... \n");

    // Ler arquivo .txt com mapa de pesos e coordenadas
    loadMapFile();

    elipse = new QGraphicsEllipseItem*[dim+1];

    d = (int) (0.05*coordenada[dim].x);
    if (d<=2)
        d =2;

    ui->graphicsView->scale((float)ui->graphicsView->width()/(coordenada[dim].x+2*d),
                            (float)ui->graphicsView->width()/(coordenada[dim].x+2*d));

    for ( i = 0 ; i < dim ; i++)

        for ( j = i ; j < dim ; j++ )
        {

            if ( mapa[i][j] != 0 ){

                ui->graphicsView->scene()->addLine(coordenada[i].x + d/2,coordenada[i].y + d/2,
                                                   coordenada[j].x + d/2 ,coordenada[j].y + d/2 ,
                                                   QPen(Qt::black,d/8));

                d_pontos = sqrt((pow((coordenada[j].x - coordenada[i].x),2))+
                                (pow(-1*coordenada[j].y + coordenada[i].y,2)));

                mapa[i][j]= (int)d_pontos;
                mapa[j][i]= (int)d_pontos;
            }
            else{
                if (i != j){
                    mapa[i][j]= 99999;
                    mapa[j][i]= 99999;
                }
            }

        }


    // Imprime a matriz mapa
    for (i=0; i<dim; i++){
        for (j=0; j<dim; j++)
        {
            cout << mapa[i][j] << " ";
        }
        cout << endl;
    }


    for ( i = 0 ; i < dim ; i++)
        elipse[i] = ui->graphicsView->scene()->addEllipse(coordenada[i].x,coordenada[i].y,d,d,QPen (Qt::black),QBrush(Qt::black));

    ui->textEdit->insertPlainText("Mapa pronto!\n");


}

void ProjetoOmega::on_pushButton_2_clicked()
{

    ui->textEdit->insertPlainText("Inserir Origem.\n");

    scene->setMode(MyScene::InserirOrigem);

}

void ProjetoOmega::on_pushButton_3_clicked()
{
    nav=true;
}

void ProjetoOmega::on_pushButton_4_clicked()
{

}

void ProjetoOmega::OrigemInserida()
{
    scene->setMode(MyScene::InserirDestino);
    ui->textEdit->insertPlainText("Inserir Destino.\n");

}

void ProjetoOmega::DestinoIserido()
{
    ui->textEdit->insertPlainText("Calculando menor distancia... \n");

    distance = new int[dim];
    marker = new bool[dim];

    caminho = new int[dim];
    predecessor = new int[dim];

    tamanho = 0;

    initializeDistance(marker, predecessor, distance);

    dijkstra(marker, distance, predecessor);

    printPath( alvo, predecessor, caminho, tamanho);

    rota.clear();

    for(int i = 0 ; (i < (tamanho -1)) ; i++)
    {

        rota.push_back(ui->graphicsView->scene()->addLine(coordenada[caminho[i]].x + d/2,coordenada[caminho[i]].y + d/2,
                coordenada[caminho[i+1]].x + d/2 ,coordenada[caminho[i+1]].y + d/2 ,
                QPen(Qt::green,d/8)));
    }

    ui->textEdit->insertPlainText("Caminho selecionado \n");

    robo = ui->graphicsView->scene()->addEllipse(coordenada[origem].x,coordenada[origem].y,d,d,QPen (Qt::black),QBrush(Qt::red));

    offset_x = coordenada[origem].x;
    offset_y = coordenada[origem].y;

}

void ProjetoOmega::RecalculandoRota()
{

    for ( i = 0 ; i < dim ; i++)
    {
    elipse[i]->setBrush(QBrush(Qt::black));
    }

    elipse[origem]->setBrush(QBrush(Qt::red));

    elipse[alvo]->setBrush(QBrush(Qt::blue));

    for( int i = 0 ; i<rota.size() ; i++ )
    {
        ui->graphicsView->scene()->removeItem(rota[i]);
    }


    rota.clear();


    initializeDistance(marker, predecessor, distance);

    dijkstra(marker, distance, predecessor);

    printPath( alvo, predecessor, caminho, tamanho);

    for(int i = 0 ; (i < (tamanho -1)) ; i++)
    {
        rota.push_back(ui->graphicsView->scene()->addLine(coordenada[caminho[i]].x + d/2,coordenada[caminho[i]].y + d/2,
                coordenada[caminho[i+1]].x + d/2 ,coordenada[caminho[i+1]].y + d/2 ,
                QPen(Qt::green,d/8)));
    }

    ui->textEdit->insertPlainText("Caminho selecionado \n");
}

void ProjetoOmega::MovimentandoRobo()
{
    robo->setX((robot.getX())-offset_x);
    robo->setY((-robot.getY())-offset_y);

    trilha.push_back(ui->graphicsView->scene()->addEllipse((robot.getX())-offset_x+d/2,(-robot.getY())-offset_y+d/2,
                                                           d/10,d/10,QPen (Qt::red),QBrush(Qt::red)));
}

void ProjetoOmega::initializeDistance( bool marker[], int predecessor[], int distance[] )
{
    int i;

    for(i = 0; ( i < dim); i++){

        marker[i] = false;
        predecessor[i] = -1;
        distance[i] = 99999;

    }
    distance[origem]=0;
}

int ProjetoOmega::getClosestUnmarkedNode( bool marker[], int distance[] )
{
    int minDistance = 99999;
    int closestUnmarkedNode=0, i;

    for(i=0; ( i < dim ) ; i++)
    {
        if((!marker[i]) && ( minDistance >= distance[i]))
        {
            minDistance = distance[i];
            closestUnmarkedNode = i;
        }
    }
    return closestUnmarkedNode;
}

void ProjetoOmega::dijkstra( bool marker[], int distance[], int predecessor[] )
{
    int closestUnmarkedNode;
    int count = 0;


    while(count < dim)

    {
        closestUnmarkedNode = getClosestUnmarkedNode(marker, distance);
        marker[closestUnmarkedNode] = true;

        for(int i=0;i<dim;i++)
        {
            if((!marker[i]) && (mapa[closestUnmarkedNode][i]>0) )
            {
                if(distance[i] > (distance[closestUnmarkedNode]+mapa[closestUnmarkedNode][i]))
                {
                    distance[i] = distance[closestUnmarkedNode]+mapa[closestUnmarkedNode][i];
                    predecessor[i] = closestUnmarkedNode;
                }
            }
        }
        count++;
    }
}

void ProjetoOmega::printPath( int destination , int predecessor[], int path[], int &num )
{


    if(destination == origem)
    {
        cout<<(char)(destination + 97)<<"..";

        path[num]= destination;
        num++;
    }
    else
    {
        if(predecessor[destination] == -1)
        {
            cout<<"No path from " << origem << " to "<< destination << endl;
        }
        else
        {

            printPath( predecessor[destination], predecessor, path, num );
            path[num] = destination;
            num++;
            cout<<(char) (destination + 97)<<"..";

        }
    }
}

ProjetoOmega::~ProjetoOmega()
{
    delete ui;
}
