#include "pioneernavigation.h"

//Variáveis globais
QMutex
semaforo_leitura_laser,
semaforo_robo,
semaforo_landmark;

int
pioneer_landmark,
x_landmark,             // Coordenada x da landmark
y_landmark;             // Coordenada y da landmark

double
readinglaser[181],      // Medidas do laser
phi;                    // Orientação do robô (graus)

bool
vai_fifi;

/******************************************************************************************************/
/******************************************* INICIALIZAÇÕES *******************************************/
/******************************************************************************************************/
PioneerNavigation::PioneerNavigation(QObject *parent) :
    QThread(parent)
{

    RecalcularRota = false;

}


/******************************************************************************************************/
/******************************** THREAD PARA NAVEGAÇÃO DO PIONEER ************************************/
/******************************************************************************************************/
void PioneerNavigation::run()
{

    salvar = new ofstream("Dados_Pioneer.m");

    navigation();

    salvar->close();

    //        pioneer_landmark = 2;
    //        int j;

    //        ArPose destino( 9000, 0000 , 0 );
    //        x_landmark = 9500;
    //        y_landmark = 500 ;

    //        ArPose posicao(8500 , 18 , -1.0 );

    //        while (true)
    //        {

    //            j = 0;
    //            semaforo_leitura_laser.lock();

    //            readingsList= laser.getRawReadings();     //Get list of readings

    //            for (it = readingsList->begin(); it != readingsList->end(); it++)
    //            {
    //                readinglaser[j] = (*it)->getRange();
    //                j++;
    //            }

    //            semaforo_leitura_laser.unlock();

    //            //Leitura do robô
    //            semaforo_robo.lock();


    //            theta = posicao.findAngleTo(destino); // [graus]
    //            phi = robot.getTh(); // [graus] orientacao do robo
    //            alpha = calcalpha(phi, theta);

    //            //        cout << endl << "Erro de orientacao = " << alpha << endl;

    //            //                    cout << "X = " << robot.getX() << endl;
    //            //                    cout << "Y = " << robot.getY() << endl << endl;
    //            semaforo_robo.unlock();
    //            emit MoverRobo();
    //            msleep(100); //teste
    //        }

}


/******************************************************************************************************/
/******************************* ALGORÍTIMO DE NAVEGAÇÃO DO PIONEER ***********************************/
/******************************************************************************************************/
void PioneerNavigation::navigation()
{
    ArPose destino(coordenada[caminho[1]].x, -coordenada[caminho[1]].y, 0); // Posicao final desejada [mm, mm , graus]
    ArPose posicao(coordenada[caminho[0]].x, -coordenada[caminho[0]].y, 0);

    if (RecalcularRota == false)
    {
        semaforo_leitura_laser.lock();
        robot.setEncoderPose(posicao);
        semaforo_leitura_laser.unlock();
    }

    //Inicialização
    int j,i;
    RecalcularRota = false;
    Obstaculo = 0;
    contador = 0;
    kv = 0.3;
    kw = 0.8;

    emit PioneerCameraOFF();

    //Inicio da navegação para todos os pontos
    for( i = 0 ; (i < (tamanho - 1)) ; i++){

        posicao.setX(coordenada[caminho[i]].x);
        posicao.setY(-coordenada[caminho[i]].y);

        destino.setX(coordenada[caminho[i+1]].x);
        destino.setY(-coordenada[caminho[i+1]].y);

        semaforo_landmark.lock();
        x_landmark = coordenada[caminho[i+1]].x + 500;
        y_landmark = -coordenada[caminho[i+1]].y + 500;
        semaforo_landmark.unlock();

        cout << endl << "Origem = " << coordenada[caminho[i]].x << " , " <<  -coordenada[caminho[i]].y
             << endl << "Destino = " << coordenada[caminho[i+1]].x << " , " << -coordenada[caminho[i+1]].y << endl;
        cout << "landmark_x = " << x_landmark << endl;
        cout << "landmark_y = " << y_landmark << endl;

        rho = (posicao.findDistanceTo(destino))/1000.0; // Erro de posicao do robo [m]

        atualizar_landmark_pioneer = true;

        vai_fifi = true;


        while(rho > 0.05) // erro maior que 5 cm
        {
            emit MoverRobo();

            if ((rho < 4.0) && (atualizar_landmark_pioneer) )
            {
                atualizar_landmark_pioneer = false;

                pioneer_landmark = (caminho[i+1] + 1);

                //Iniciando a Thread para câmera do Pioneer
                PioneerCamera->start();
            }

            semaforo_leitura_laser.lock();
            reading = laser.currentReadingPolar( -90, 90,&readingAngle );      //Get minimum reading and angle
            reading2 = laser.currentReadingPolar( -20,20,&readingAngle2 );     //Get minimum reading and angle
            readingsList = laser.getRawReadings();                             //Get list of readings
            semaforo_leitura_laser.unlock();


            semaforo_robo.lock();
            theta = posicao.findAngleTo(destino); // [graus]
            phi = robot.getTh();                  // [graus] orientacao do robo
            alpha = calcalpha(phi, theta);
            semaforo_robo.unlock();


            j = 0;
            Obstaculo = 0;

            semaforo_leitura_laser.lock();
            for (it = readingsList->begin(); it != readingsList->end(); it++)
            {

                readinglaser[j] = (*it)->getRange();

                if ((readinglaser[j] < 1200.0) && ( j > 45 ) && ( j < 135 ))
                    Obstaculo++;

                j++;

            }
            semaforo_leitura_laser.unlock();

            if ( Obstaculo >= 80 ){
                RecalcularRota = true;
                break;
            }


            if ( (reading < 600.0) && (rho >= (reading/1000.0) )){

                zeta = sign(readingAngle) * (fabs(readingAngle)-90) - alpha;

                semaforo_robo.lock();
                xv = robot.getX()/1000.0 + cos(theta * PI / 180.0);
                yv = robot.getY()/1000.0 + sin(theta * PI / 180.0);

                destino.setX( (((xv-robot.getX()) * cos(zeta * PI / 180)) + ((yv-robot.getY()) * sin(zeta * PI / 180)) * 1000.0 ));
                destino.setY( -(-(xv-robot.getX()) * sin(zeta * PI / 180) + ((yv-robot.getY()) * cos(zeta * PI / 180)) * 1000.0 ));
                semaforo_robo.unlock();

                alpha = alpha + zeta;

            }
            else
            {

                destino.setX(coordenada[caminho[i+1]].x);
                destino.setY(-coordenada[caminho[i+1]].y);
                //                semaforo_landmark.lock();
                //                x_landmark = coordenada[caminho[i+1]].x + 1000;
                //                y_landmark = -coordenada[caminho[i+1]].y + 1000;
                //                semaforo_landmark.unlock();
            }

            //            else if((reading2 < 1100.0) && (rho >= (reading2/1000.0) )){

            //                readingsList = thisLaser->getRawReadings();     //Get list of readings


            //                zeta = sign(readingAngle2) * (fabs(readingAngle2)-90) - alpha;

            //                xv = robot.getX()/1000.0 + cos(theta * PI / 180.0);
            //                yv = robot.getY()/1000.0 + sin(theta * PI / 180.0);

            //                destino.setX( (((xv-robot.getX()) * cos(zeta * PI / 180)) + ((yv-robot.getY()) * sin(zeta * PI / 180)) * 1000.0 ));
            //                destino.setY( -(-(xv-robot.getX()) * sin(zeta * PI / 180) + ((yv-robot.getY())*cos(zeta * PI / 180)) * 1000.0 ));

            //                alpha = alpha + zeta;

            //            }


            //Cálculo do controlador
            rho = posicao.findDistanceTo(destino)/1000.0;

            v = (kv * tanh(rho) * cos(alpha * PI / 180.0)) * 1000.0 ; //[mm/s]
            w = (kw * (alpha * PI / 180.0) + kv * (tanh(rho)/(rho)) * sin(alpha * PI / 180.0) *cos(alpha * PI / 180.0))*180.0/PI;

            semaforo_robo.lock();
            robot.setVel(v);
            robot.setRotVel(w);
            vr = robot.getVel();
            wr = robot.getRotVel();
            posicao = robot.getPose();
            rho = posicao.findDistanceTo(destino)/1000.0;
            (*salvar) << contador/10.0 << " " << v  << " " << w  << " " << vr  << " " << wr
                      << " " << posicao.getX() << " " << posicao.getY() << " " << rho << " " << pioneer_landmark << endl;
            semaforo_robo.unlock();

            contador++;

            //Correção do encoder
            if (PontoAtingido == true)
            {

                i = 0;
                semaforo_leitura_laser.lock();
                for ( j = 55 ; j <= 125 ; j++ )
                {
                    laser[i] = readinglaser[j+1] - readinglaser[j];
                    i++;
                }
                semaforo_leitura_laser.unlock();

                // Máquina de estados para procurar a landmark nas medidas do laser
                i = 0;
                j = 0;
                inicio = 0;
                fim = 0;
                setMode(Estado_1);

                LandmarkDetected = true;

                while ( i <= 70 )
                {

                    switch (myMode) {

                    case Estado_1:
                    {
                        if ( laser[i] <= -150 )
                        {
                            inicio = i;
                            i++;
                            setMode(Estado_2);
                        }
                        else
                        {
                            i++;
                        }
                        break;
                    }

                    case Estado_2:
                    {
                        if ( laser[i] <= -150 )
                        {
                            i++;
                        }
                        else
                        {
                            if ( laser[i] >= 50 )
                            {
                                i++;
                                setMode(Estado_1);
                            }
                            else
                            {
                                i++;
                                setMode(Estado_3);
                            }
                        }
                        break;
                    }

                    case Estado_3:
                    {
                        if ( ( laser[i] >= -50 ) && ( laser[i] <= 50 ) )
                        {
                            j++;
                            i++;
                            if (j > 8 )
                            {
                                j = 0;
                                setMode(Estado_1);
                            }

                        }
                        else
                        {
                            if ( ( laser[i] >= 50 ) && ( j >= 3 ) && ( j <= 8 ) )
                            {
                                fim = i;
                                j = 0;
                                setMode(Estado_4);
                            }
                            else
                            {
                                i++;
                                j = 0;
                                setMode(Estado_1);
                            }
                        }
                        break;
                    }

                    case Estado_4:
                    {

                        //                            inicio = inicio + angle;
                        //                            fim = fim + angle;
                        //                            posicao = inicio + ( (fim - inicio) / 2 );

                        posicao = 55 + inicio + ( (fim - inicio) / 2 );

                        //cout << "inicio = " << inicio << endl;
                        //cout << "fim = " << fim << endl;
                        //cout << "posicao = " << posicao << endl;

                        // Deve-se somar 5cm do raio do tubo
                        semaforo_leitura_laser.lock();
                        dist_to_landmark = readinglaser[(int)(posicao)] + 50;
                        semaforo_leitura_laser.unlock();
                        //cout << "Distancia = " << dist_to_landmark << endl;

                        // A distância entre o laser e o centro do robô é de 25cm
                        // Essa equação corrige a distância medida pelo laser para a distância
                        // entre o centro do robô e a landmark


                        if ( posicao > 90 )
                        {
                            angle = posicao - 90;
                            dist_to_landmark = sqrt( pow( ( ( dist_to_landmark * cos(angle * PI / 180.0) ) + 250 ),2) + pow( ( dist_to_landmark * sin(angle * PI / 180.0) ),2) );
                        }
                        else
                        {
                            if (angle == 90)
                            {
                                angle = 0;
                                dist_to_landmark = dist_to_landmark + 250;
                            }
                            else
                            {
                                angle = 90 - posicao;
                                dist_to_landmark = sqrt( pow( ( ( dist_to_landmark * cos(angle * PI / 180.0) ) + 250 ),2) + pow( ( dist_to_landmark * sin(angle * PI / 180.0) ),2) );
                                angle = -1 * angle;
                            }
                        }


                        ////////////////////////////////
                        // Correção da posição do Robô//
                        ////////////////////////////////

                        semaforo_landmark.lock();
                        x_robo_real = x_landmark - dist_to_landmark * cos( ( angle + phi ) * PI / 180.0 );
                        y_robo_real = y_landmark - dist_to_landmark * sin( ( angle + phi ) * PI / 180.0 );
                        semaforo_landmark.unlock();

                        semaforo_robo.lock();
                        cout << "angle_landmark = " << angle << endl;
                        cout << "distância = " << dist_to_landmark << endl;
                        cout << "orientação do robo = " << phi << endl;
                        cout << "x_odometria = " << robot.getX() << endl;
                        cout << "y_odometria = " << robot.getY() << endl;
                        cout << "x_corrigido = " << x_robo_real << endl;
                        cout << "y_corrigido = " << y_robo_real << endl;
                        semaforo_robo.unlock();

                        // Alterando x e y do encoder do robô
                        //                            semaforo_robo.lock();
                        //                            posicao_corrigida.setPose( x_robo_real , y_robo_real , phi );
                        //                            robot.setEncoderPose( posicao_corrigida );
                        //                            semaforo_robo.unlock();
                        // Finaliza a busca da landmark
                        i= 71;

                        break;
                    }

                    default:
                    {

                        break;
                    }

                    }
                }

                PontoAtingido = false;

            }

            msleep(100);

            // ArUtil::sleep(100);

        }


        if ( RecalcularRota == true ){

            emit PioneerCameraOFF();

            semaforo_robo.lock();
            robot.setVel(0);
            robot.setRotVel(0);

            posicao.setX(robot.getX());
            posicao.setY(robot.getY());
            semaforo_robo.unlock();

            destino.setX(coordenada[caminho[i]].x);
            destino.setY(-coordenada[caminho[i]].y);
            semaforo_landmark.unlock();
            x_landmark = coordenada[caminho[i]].x + 500;
            y_landmark = -coordenada[caminho[i]].y + 500;
            semaforo_landmark.unlock();

            rho = (posicao.findDistanceTo(destino))/1000.0; // Erro de posicao do robo [m]

            ArUtil::sleep(2000);

            atualizar_landmark_pioneer = true;

            while(rho > 0.05) // erro maior que 5 cm
            {
                emit MoverRobo();

                if ((rho < 4.0) && (atualizar_landmark_pioneer) )
                {
                    atualizar_landmark_pioneer = false;

                    pioneer_landmark = (caminho[i+1] + 1);

                    //Iniciando a Thread para câmera do Pioneer
                    PioneerCamera->start();
                }

                semaforo_leitura_laser.lock();
                reading = laser.currentReadingPolar( -90, 90,&readingAngle );      //Get minimum reading and angle
                reading2 = laser.currentReadingPolar( -20,20,&readingAngle2 );     //Get minimum reading and angle
                readingsList= laser.getRawReadings();     //Get list of readings
                semaforo_leitura_laser.unlock();

                j=0;
                semaforo_leitura_laser.lock();
                for (it = readingsList->begin(); it != readingsList->end(); it++)
                {

                    readinglaser[j] = (*it)->getRange();
                    j++;

                }
                semaforo_leitura_laser.unlock();

                semaforo_robo.lock();
                theta = posicao.findAngleTo(destino); // [graus]
                phi = robot.getTh(); // [graus] orientacao do robo
                alpha = calcalpha(phi, theta);
                semaforo_robo.unlock();

                if ( (reading < 600.0) && (rho >= (reading/1000.0) )){



                    zeta = sign(readingAngle) * (fabs(readingAngle)-90) - alpha;

                    semaforo_robo.lock();
                    xv = robot.getX()/1000.0 + cos(theta * PI / 180.0);
                    yv = robot.getY()/1000.0 + sin(theta * PI / 180.0);

                    destino.setX( (((xv-robot.getX()) * cos(zeta * PI / 180)) + ((yv-robot.getY()) * sin(zeta * PI / 180)) * 1000.0 ));
                    destino.setY( -(-(xv-robot.getX()) * sin(zeta * PI / 180) + ((yv-robot.getY())*cos(zeta * PI / 180)) * 1000.0 ));
                    semaforo_robo.unlock();

                    alpha = alpha + zeta;


                }

                else
                {

                    destino.setX(coordenada[caminho[i]].x);
                    destino.setY(-coordenada[caminho[i]].y);
                    //                    semaforo_landmark.lock();
                    //                    x_landmark = coordenada[caminho[i]].x + 1000;
                    //                    y_landmark = -coordenada[caminho[i]].y + 1000;
                    //                    semaforo_landmark.unlock();

                }

                rho = posicao.findDistanceTo(destino)/1000.0;
                v = (kv * tanh(rho) * cos(alpha * PI / 180.0)) * 1000.0 ; //[mm/s]
                w = (kw * (alpha * PI / 180.0) + kv * (tanh(rho)/(rho)) * sin(alpha * PI / 180.0) *cos(alpha * PI / 180.0))*180.0/PI;

                semaforo_robo.lock();
                robot.setVel(v);
                robot.setRotVel(w);
                vr = robot.getVel();
                wr = robot.getRotVel();
                posicao = robot.getPose();
                semaforo_robo.unlock();

                rho = posicao.findDistanceTo(destino)/1000.0;

                (*salvar) << contador/10.0 << " " << v  << " " << w  << " " << vr  << " " << wr
                          << " " << posicao.getX() << " " << posicao.getY() << " " << rho << " " << pioneer_landmark << endl;

                contador++;

                ArUtil::sleep(100);

            }

            if (PontoAtingido == false)
            {
                semaforo_robo.lock();
                robot.setVel(0);
                robot.setRotVel(0);
                ArUtil::sleep(500);

                // Girando em torno do eixo, procurando a landmark
                robot.setRotVel(10.0);
                ArUtil::sleep(36000);
                robot.setRotVel(0);
                semaforo_robo.unlock();


                if (PontoAtingido == false)
                {
                    RecalcularRota = true;
                    break;
                }
                else
                {
                    PontoAtingido = false;
                }

            }
            else
            {

                PontoAtingido = false;

            }

            break;
        }


        if (PontoAtingido == false)
        {
            semaforo_robo.lock();
            robot.setVel(0);
            robot.setRotVel(0);
            ArUtil::sleep(500);
            robot.setRotVel(10.0);
            ArUtil::sleep(36000);
            robot.setRotVel(0);
            semaforo_robo.unlock();

            if (PontoAtingido == false)
            {
                emit PioneerCameraOFF();

                RecalcularRota = true;

                cout << endl << "Recalcular a Rota." << endl;

                semaforo_robo.lock();
                posicao.setX(robot.getX());
                posicao.setY(robot.getY());
                semaforo_robo.unlock();

                destino.setX(coordenada[caminho[i]].x);
                destino.setY(-coordenada[caminho[i]].y);
                semaforo_landmark.lock();
                x_landmark = coordenada[caminho[i]].x + 500;
                y_landmark = -coordenada[caminho[i]].y + 500;
                semaforo_landmark.unlock();

                rho = (posicao.findDistanceTo(destino))/1000.0; // Erro de posicao do robo [m]

                ArUtil::sleep(2000);

                atualizar_landmark_pioneer = true;

                while(rho > 0.05) // erro maior que 5 cm
                {
                    emit MoverRobo();

                    if ((rho < 4.0) && (atualizar_landmark_pioneer) )
                    {
                        atualizar_landmark_pioneer = false;

                        pioneer_landmark = (caminho[i+1] + 1);

                        //Iniciando a Thread para câmera do Pioneer
                        PioneerCamera->start();
                    }

                    semaforo_leitura_laser.lock();
                    reading = laser.currentReadingPolar( -90, 90,&readingAngle );      //Get minimum reading and angle
                    reading2 = laser.currentReadingPolar( -20,20,&readingAngle2 );     //Get minimum reading and angle
                    readingsList= laser.getRawReadings();     //Get list of readings
                    semaforo_leitura_laser.unlock();

                    j=0;
                    semaforo_leitura_laser.lock();
                    for (it = readingsList->begin(); it != readingsList->end(); it++)
                    {

                        readinglaser[j] = (*it)->getRange();
                        j++;

                    }
                    semaforo_leitura_laser.unlock();

                    semaforo_robo.lock();
                    theta = posicao.findAngleTo(destino); // [graus]
                    phi = robot.getTh(); // [graus] orientacao do robo
                    alpha = calcalpha(phi, theta);
                    semaforo_robo.unlock();


                    if ( (reading < 600.0) && (rho >= (reading/1000.0) )){



                        zeta = sign(readingAngle) * (fabs(readingAngle)-90) - alpha;

                        semaforo_robo.lock();
                        xv = robot.getX()/1000.0 + cos(theta * PI / 180.0);
                        yv = robot.getY()/1000.0 + sin(theta * PI / 180.0);

                        destino.setX( (((xv-robot.getX()) * cos(zeta * PI / 180)) + ((yv-robot.getY()) * sin(zeta * PI / 180)) * 1000.0 ));
                        destino.setY( -(-(xv-robot.getX()) * sin(zeta * PI / 180) + ((yv-robot.getY())*cos(zeta * PI / 180)) * 1000.0 ));
                        semaforo_robo.unlock();

                        alpha = alpha + zeta;

                    }

                    else{

                        destino.setX(coordenada[caminho[i]].x);
                        destino.setY(-coordenada[caminho[i]].y);
                        //                        semaforo_landmark.lock();
                        //                        x_landmark = coordenada[caminho[i]].x + 1000;
                        //                        y_landmark = -coordenada[caminho[i]].y + 1000;
                        //                        semaforo_landmark.unlock();

                    }

                    rho = posicao.findDistanceTo(destino)/1000.0;

                    v = (kv * tanh(rho) * cos(alpha * PI / 180.0)) * 1000.0 ; //[mm/s]
                    w = (kw * (alpha * PI / 180.0) + kv * (tanh(rho)/(rho)) * sin(alpha * PI / 180.0) *cos(alpha * PI / 180.0))*180.0/PI;

                    semaforo_robo.lock();
                    robot.setVel(v);
                    robot.setRotVel(w);
                    vr = robot.getVel();
                    wr = robot.getRotVel();
                    posicao = robot.getPose();
                    semaforo_robo.unlock();

                    rho = posicao.findDistanceTo(destino)/1000.0;

                    (*salvar) << contador/10.0 << " " << v  << " " << w  << " " << vr  << " " << wr
                              << " " << posicao.getX() << " " << posicao.getY() << " " << rho << " " << pioneer_landmark << endl;

                    contador++;

                    ArUtil::sleep(100);

                }


                break;
            }
            else
            {
                PontoAtingido = false;
            }

        }
        else
        {
            PontoAtingido = false;
        }

    }

    semaforo_robo.lock();
    robot.setVel(0);
    robot.setRotVel(0);
    semaforo_robo.unlock();

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


    emit ObjetivoAlcancado();

}


/******************************************************************************************************/
/******************************************* ERRO ANGULAR *********************************************/
/******************************************************************************************************/
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


/******************************************************************************************************/
/*************************************** SINAL DA ORIENTAÇÃO ******************************************/
/******************************************************************************************************/
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


/******************************************************************************************************/
/******************************** FUNÇÃO PARA SELECIONAR O ESTADO *************************************/
/******************************************************************************************************/
void CameraPioneer::setMode(Mode mode)
{
    myMode = mode;
}
