#include "pioneernavigation.h"
#include <QtCore>

bool        nav = false;
QMutex      mutex;

PioneerNavigation::PioneerNavigation(QObject *parent) :
    QThread(parent)
{

    RecalcularRota = false;


}

void PioneerNavigation::run()
{
    while (1)
    {

        mutex.lock();
        if (nav == true)
        {
            navigation();

        }
        mutex.unlock();
    }


}


void PioneerNavigation::navigation()
{

    ArPose destino(coordenada[caminho[1]].x, -coordenada[caminho[1]].y, 0); // Posicao final desejada [mm, mm , graus]
    ArPose posicao(coordenada[caminho[0]].x, -coordenada[caminho[0]].y,0);

    if (RecalcularRota == false)
    {
        robot.setEncoderPose(posicao);
    }

    //Inicialização
    int j,i;

    RecalcularRota = false;
    Obstaculo = 0;
    contador = 0;
    kv = 0.3;
    kw = 0.8;
    PI = 3.1415926;


    for( i = 0 ; (i < (tamanho - 1)) ; i++){

        posicao.setX(coordenada[caminho[i]].x);
        posicao.setY(-coordenada[caminho[i]].y);

        destino.setX(coordenada[caminho[i+1]].x);
        destino.setY(-coordenada[caminho[i+1]].y);

        cout << endl << "Origem = " << coordenada[caminho[i]].x << " , " <<  -coordenada[caminho[i]].y
             << endl << "Destino = " << coordenada[caminho[i+1]].x << " , " << -coordenada[caminho[i+1]].y << endl;
        cout << robot.getX() << endl;
        cout << robot.getY() << endl;

        rho = (posicao.findDistanceTo(destino))/1000.0; // Erro de posicao do robo [m]

        while(rho > 0.05) // erro maior que 5 cm
        {
            emit MoverRobo();

            reading = laser.currentReadingPolar( -90, 90,&readingAngle );      //Get minimum reading and angle
            reading2 = laser.currentReadingPolar( -20,20,&readingAngle2 );     //Get minimum reading and angle


            theta = posicao.findAngleTo(destino); // [graus]
            phi = robot.getTh(); // [graus] orientacao do robo
            alpha = calcalpha(phi, theta);

            Obstaculo = 0;

            if ( (reading < 600.0) & (rho >= (reading/1000.0) )){



                zeta = sign(readingAngle) * (fabs(readingAngle)-90) - alpha;

                xv = robot.getX()/1000.0 + cos(theta * PI / 180.0);
                yv = robot.getY()/1000.0 + sin(theta * PI / 180.0);

                destino.setX( (((xv-robot.getX()) * cos(zeta * PI / 180)) + ((yv-robot.getY()) * sin(zeta * PI / 180)) * 1000.0 ));
                destino.setY( -(-(xv-robot.getX()) * sin(zeta * PI / 180) + ((yv-robot.getY())*cos(zeta * PI / 180)) * 1000.0 ));

                alpha = alpha + zeta;


            }
            //            else if((reading2 < 1100.0) & (rho >= (reading2/1000.0) )){

            //                readingsList = thisLaser->getRawReadings();     //Get list of readings


            //                zeta = sign(readingAngle2) * (fabs(readingAngle2)-90) - alpha;

            //                xv = robot.getX()/1000.0 + cos(theta * PI / 180.0);
            //                yv = robot.getY()/1000.0 + sin(theta * PI / 180.0);

            //                destino.setX( (((xv-robot.getX()) * cos(zeta * PI / 180)) + ((yv-robot.getY()) * sin(zeta * PI / 180)) * 1000.0 ));
            //                destino.setY( -(-(xv-robot.getX()) * sin(zeta * PI / 180) + ((yv-robot.getY())*cos(zeta * PI / 180)) * 1000.0 ));

            //                alpha = alpha + zeta;

            //            }

            else{

                readingsList= laser.getRawReadings();     //Get list of readings

                j = 0;

                for (it = readingsList->begin(); it != readingsList->end(); it++)
                {

                    readinglaser[j] = (*it)->getRange();

                    if ((readinglaser[j] < 1200.0) & ( j > 45 ) & ( j < 135 ))
                        Obstaculo++;

                    j++;

                }
                //                cout << endl << Obstaculo;

                if ( Obstaculo >= 80 ){
                    RecalcularRota = true;
                    break;
                }

                destino.setX(coordenada[caminho[i+1]].x);
                destino.setY(-coordenada[caminho[i+1]].y);
            }

            rho = posicao.findDistanceTo(destino)/1000.0;

            v = (kv * tanh(rho) * cos(alpha * PI / 180.0)) * 1000.0 ; //[mm/s]
            w = (kw * (alpha * PI / 180.0) + kv * (tanh(rho)/(rho)) * sin(alpha * PI / 180.0) *cos(alpha * PI / 180.0))*180.0/PI;


            robot.setVel(v);
            robot.setRotVel(w);
            vr = robot.getVel();
            wr = robot.getRotVel();
            posicao = robot.getPose();

            rho = posicao.findDistanceTo(destino)/1000.0;
            //fprintf(posicaofinal,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",contador/10.0, v,w,vr,wr,posicao.getX(),posicao.getY(),rho);
            contador++;

            ArUtil::sleep(100);


        }


        if ( RecalcularRota == true ){


            robot.setVel(0);
            robot.setRotVel(0);


            posicao.setX(robot.getX());
            posicao.setY(robot.getY());

            destino.setX(coordenada[caminho[i]].x);
            destino.setY(-coordenada[caminho[i]].y);

            rho = (posicao.findDistanceTo(destino))/1000.0; // Erro de posicao do robo [m]

            ArUtil::sleep(2000);

            while(rho > 0.05) // erro maior que 5 cm
            {
                emit MoverRobo();

                reading = laser.currentReadingPolar( -90, 90,&readingAngle );      //Get minimum reading and angle
                reading2 = laser.currentReadingPolar( -20,20,&readingAngle2 );     //Get minimum reading and angle


                theta = posicao.findAngleTo(destino); // [graus]
                phi = robot.getTh(); // [graus] orientacao do robo
                alpha = calcalpha(phi, theta);

                Obstaculo = 0;


                if ( (reading < 600.0) & (rho >= (reading/1000.0) )){



                    zeta = sign(readingAngle) * (fabs(readingAngle)-90) - alpha;

                    xv = robot.getX()/1000.0 + cos(theta * PI / 180.0);
                    yv = robot.getY()/1000.0 + sin(theta * PI / 180.0);

                    destino.setX( (((xv-robot.getX()) * cos(zeta * PI / 180)) + ((yv-robot.getY()) * sin(zeta * PI / 180)) * 1000.0 ));
                    destino.setY( -(-(xv-robot.getX()) * sin(zeta * PI / 180) + ((yv-robot.getY())*cos(zeta * PI / 180)) * 1000.0 ));

                    alpha = alpha + zeta;


                }

                else{

                    destino.setX(coordenada[caminho[i]].x);
                    destino.setY(-coordenada[caminho[i]].y);
                }

                rho = posicao.findDistanceTo(destino)/1000.0;

                v = (kv * tanh(rho) * cos(alpha * PI / 180.0)) * 1000.0 ; //[mm/s]
                w = (kw * (alpha * PI / 180.0) + kv * (tanh(rho)/(rho)) * sin(alpha * PI / 180.0) *cos(alpha * PI / 180.0))*180.0/PI;


                robot.setVel(v);
                robot.setRotVel(w);
                vr = robot.getVel();
                wr = robot.getRotVel();
                posicao = robot.getPose();

                rho = posicao.findDistanceTo(destino)/1000.0;
                //fprintf(posicaofinal,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",contador/10.0, v,w,vr,wr,posicao.getX(),posicao.getY(),rho);
                contador++;

                ArUtil::sleep(100);

            }

            break;
        }


    }

    robot.setVel(0);
    robot.setRotVel(0);


    if (RecalcularRota == true){

        //Alterando mapa inserindo impedimento da rota atual e recalculando rota
        mapa[caminho[i]][caminho[i+1]] = 99999;
        mapa[caminho[i+1]][caminho[i]] = 99999;

        origem = caminho[i];

        tamanho = 0;

        emit RedefinirRota();

        ArUtil::sleep(2000);

        navigation();

    }

    nav = false;
}

double  PioneerNavigation::calcalpha(double orient, double erdest){

    double alpha;
    int a;

    alpha = erdest - orient;
    if (fabs(alpha) > 180.0)
    {
        if (alpha > 0)
        {
            a = -1;
        }
        else
        {
            a = 1;
        }
        alpha = (360 - fabs(alpha)) * a;
    }
    return alpha;
}

double PioneerNavigation::sign(double phi){

    if (phi > 0){
        return (1.0);
    }

    else if (phi < 0){

        return (-1.0);
    }

    else{
        return(0);
    }

}
