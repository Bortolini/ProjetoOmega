#include "pioneernavigation.h"

//Variáveis globais
QMutex
semaforo_leitura_laser,
semaforo_landmark,
semaforo_robo;

double
readinglaser[181],
phi;

int
pioneer_landmark,
x_landmark,             // Coordenada x da landmark
y_landmark;             // Coordenada y da landmark


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
    //Inicializações
    contador = 0;

    //Abrir arquivo para salvar os dados de navegação
    salvar = new ofstream("Dados_Pioneer.m");

    //Função para navegação entre os pontos selecionados em ProjetoOmega
    navigation();

    //Fechar arquivos com os dados da navegação
    salvar->close();

}


/******************************************************************************************************/
/******************************* ALGORÍTIMO DE NAVEGAÇÃO DO PIONEER ***********************************/
/******************************************************************************************************/
void PioneerNavigation::navigation()
{
    ArPose destino(coordenada[caminho[1]].x, -coordenada[caminho[1]].y,(double)0.0); // Posicao final desejada [mm, mm , graus]
    ArPose posicao(coordenada[caminho[0]].x, -coordenada[caminho[0]].y, (double)0.0); // Posição atual do robô


    // Atualiza posição do robô se a função for chamada de um recálculo de rota
    if (RecalcularRota == false)
    {
        robot.setEncoderPose(posicao);
    }

    //Inicialização
    RecalcularRota = false; // Solicita o recálculo de rota do Robô

    //Inicio da navegação para todos os pontos
    for( i = 0 ; (i < (tamanho - 1)) ; i++)
    {

        destino.setX(coordenada[caminho[i+1]].x);
        destino.setY(-coordenada[caminho[i+1]].y);

        semaforo_landmark.lock();
        x_landmark = coordenada[caminho[i+1]].x + 1000;
        y_landmark = -coordenada[caminho[i+1]].y - 1000;
        semaforo_landmark.unlock();

        atualizar_landmark_pioneer = true;

        //Solicita encerramento da Thread da Câmera do Pioneer
        emit PioneerCameraOFF();

        controlador_giro( posicao , destino ); // Controlador para corrigir a orientação do robô

        ArUtil::sleep(500);

        controlador( posicao , destino );      //Controlador de posição final


        if ( RecalcularRota == true )
        {
            if (atualizar_landmark_pioneer == false)
                emit PioneerCameraOFF();

            destino.setX(coordenada[caminho[i]].x);
            destino.setY(-coordenada[caminho[i]].y);

            semaforo_landmark.lock();
            x_landmark = coordenada[caminho[i]].x + 1000;
            y_landmark = -coordenada[caminho[i]].y - 1000;
            semaforo_landmark.unlock();

            atualizar_landmark_pioneer = true;

            ArUtil::sleep(500);

            posicao = robot.getPose();
            controlador_giro( posicao , destino ); // Controlador para corrigir a orientação do robô

            ArUtil::sleep(500);

            posicao = robot.getPose();
            controlador( posicao , destino ); //Controlador de posição final


            if (PontoAtingido == false)
            {
                //Girando em torno do eixo, procurando a landmark
                controlador_busca_landmark();

                if (PontoAtingido == false)
                {
                    emit ObjetivoAlcancado();
                    break;
                }

                ///////////////////////////////////
                // Correção do erro de odometria //
                ///////////////////////////////////
                posicao = robot.getPose();

                controlador_giro( posicao , destino ); // Controlador para corrigir a orientação do robô
                ArUtil::sleep(500);

                posicao = robot.getPose();
                controlador( posicao , destino );      //Controlador de posição final

                ArUtil::sleep(500);
                PontoAtingido = false;

            }
            else
            {
                ArUtil::sleep(500);
                PontoAtingido = false;
            }

            break;
        }

        if (PontoAtingido == false)
        {

            //Girando em torno do eixo, procurando a landmark
            controlador_busca_landmark();
            ArUtil::sleep(500);


            if (PontoAtingido == false)
            {

                RecalcularRota = true;

                if (atualizar_landmark_pioneer == false)
                    emit PioneerCameraOFF();

                destino.setX(coordenada[caminho[i]].x);
                destino.setY(-coordenada[caminho[i]].y);

                semaforo_landmark.lock();
                x_landmark = coordenada[caminho[i]].x + 1000;
                y_landmark = -coordenada[caminho[i]].y - 1000;
                semaforo_landmark.unlock();

                atualizar_landmark_pioneer = true;

                ArUtil::sleep(500);

                posicao = robot.getPose();
                controlador_giro( posicao , destino ); // Controlador para corrigir a orientação do robô

                ArUtil::sleep(500);

                posicao = robot.getPose();
                controlador( posicao , destino ); //Controlador de posição final

                if (PontoAtingido == false)
                {
                    //Girando em torno do eixo, procurando a landmark
                    controlador_busca_landmark();

                    if (PontoAtingido == false)
                    {
                        emit ObjetivoAlcancado();
                        break;
                    }

                }

            }
            ///////////////////////////////////
            // Correção do erro de odometria //
            ///////////////////////////////////
            posicao = robot.getPose();

            controlador_giro( posicao , destino ); // Controlador para corrigir a orientação do robô
            ArUtil::sleep(500);

            posicao = robot.getPose();
            controlador( posicao , destino );      //Controlador de posição final

            ArUtil::sleep(500);
            PontoAtingido = false;

        }
    }

    semaforo_robo.lock();
    robot.setVel(0);
    robot.setRotVel(0);
    semaforo_robo.unlock();

    if (RecalcularRota == true)
    {
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
void PioneerNavigation::setMode(Mode mode)
{
    myMode = mode;
}


/******************************************************************************************************/
/**************************** FUNÇÃO PARA O CONTROLADOR DE POSIÇÃO FINAL ******************************/
/******************************************************************************************************/
void PioneerNavigation::controlador( ArPose posicao , ArPose destino )
{
    int
            j,
            Obstaculo;

    rho = (posicao.findDistanceTo(destino))/1000.0; // Erro de posicao do robo [m]

    kv = 0.3;
    kw = 0.8;

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

            if (readinglaser[j] > 5000.0)
                readinglaser[j] = 5000.0;

            if ((readinglaser[j] < 1200.0) && ( j > 45 ) && ( j < 135 ))
                Obstaculo ++;
            j++;

        }
        semaforo_leitura_laser.unlock();

        if ( Obstaculo >= 80 )
        {
            RecalcularRota = true;
            robot.setVel(0.0);
            robot.setRotVel(0.0);
            break;
        }


        if ( (reading < 600.0) && (rho >= (reading/1000.0) ))
        {

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

            destino.setX(x_landmark - 1000);
            destino.setY(y_landmark + 1000);

        }

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
                  << " " << robot.getX() << " " << robot.getY() << " " << rho << " " << pioneer_landmark << endl;

        semaforo_robo.unlock();

        contador++;
        ArUtil::sleep(100);
    }
}


/******************************************************************************************************/
/********************************* FUNÇÃO PARA O CONTROLADOR DE GIRO **********************************/
/******************************************************************************************************/
void PioneerNavigation::controlador_giro( ArPose posicao , ArPose destino )
{
    int
            j;

    //inicializações
    kv = 0.0;
    kw = 0.4;

    semaforo_robo.lock();
    theta = posicao.findAngleTo(destino); // [graus]
    phi = robot.getTh();                  // [graus] orientacao do robo
    alpha = calcalpha(phi, theta);
    semaforo_robo.unlock();

    while( (alpha > 2.0) || (alpha < -2.0) ) // erro maior que 0.5 graus
    {

        emit MoverRobo();

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

        semaforo_leitura_laser.lock();
        for (it = readingsList->begin(); it != readingsList->end(); it++)
        {

            readinglaser[j] = (*it)->getRange();

            if (readinglaser[j] > 5000.0)
                readinglaser[j] = 5000.0;

            if ((readinglaser[j] < 1200.0) && ( j > 45 ) && ( j < 135 ))
                Obstaculo ++;
            j++;

        }
        semaforo_leitura_laser.unlock();

        if ( (reading < 600.0) && (rho >= (reading/1000.0) ))
        {

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

            destino.setX(x_landmark - 1000);
            destino.setY(y_landmark + 1000);

        }

        //Cálculo do controlador
        rho = posicao.findDistanceTo(destino)/1000.0;

        v = 0.0;
        w = (kw * (alpha * PI / 180.0) + kv * (tanh(rho)/(rho)) * sin(alpha * PI / 180.0) *cos(alpha * PI / 180.0))*180.0/PI;

        semaforo_robo.lock();
        robot.setVel(v);
        robot.setRotVel(w);
        vr = robot.getVel();
        wr = robot.getRotVel();
        posicao = robot.getPose();
        rho = posicao.findDistanceTo(destino)/1000.0;

        (*salvar) << contador/10.0 << " " << v  << " " << w  << " " << vr  << " " << wr
                  << " " << robot.getX() << " " << robot.getY() << " " << rho << " " << pioneer_landmark << endl;

        semaforo_robo.unlock();

        contador++;
        ArUtil::sleep(100);
    }
}

/******************************************************************************************************/
/********************************* FUNÇÃO PARA O CONTROLADOR DE GIRO 2 *********************************/
/******************************************************************************************************/
void PioneerNavigation::controlador_busca_landmark( )
{
    int
            j,
            psi = 0;

    //inicializações
    kv = 0.0;
    kw = 0.4;

    robot.setVel(0.0);
    robot.setRotVel(10.0);

    while( psi < 360 ) // erro maior que 0.5 graus
    {
        if (PontoAtingido)
        {
            atualizar_landmark_pioneer = false;

            pioneer_landmark = (caminho[i+1] + 1);
            break;
        }

        emit MoverRobo();

        semaforo_leitura_laser.lock();
        readingsList = laser.getRawReadings();                             //Get list of readings
        semaforo_leitura_laser.unlock();

        j = 0;

        semaforo_leitura_laser.lock();
        for (it = readingsList->begin(); it != readingsList->end(); it++)
        {

            readinglaser[j] = (*it)->getRange();

            if (readinglaser[j] > 5000.0)
                readinglaser[j] = 5000.0;
            j++;

        }
        semaforo_leitura_laser.unlock();

        semaforo_robo.lock();
        phi = robot.getTh();                  // [graus] orientacao do robo
        semaforo_robo.unlock();

        semaforo_robo.lock();
        (*salvar) << contador/10.0 << " " << v  << " " << w  << " " << vr  << " " << wr
                  << " " << robot.getX() << " " << robot.getY() << " " << rho << " " << pioneer_landmark << endl;
        semaforo_robo.unlock();

        contador++;
        psi ++;
        cout << "psi = " << psi << endl;

        ArUtil::sleep(100);
    }
    robot.setRotVel(0.0);
}
