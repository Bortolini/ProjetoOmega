#include "CameraPioneer.h"

bool PontoAtingido = false;

CameraPioneer::CameraPioneer(QObject *parent) :
    QThread(parent)
{
    
}

/******************************************************************************************************/
/******************** THREAD PARA CAPTURA E PROCESSAMENTO DAS IMAGENS DO PIONEER **********************/
/******************************************************************************************************/
void CameraPioneer::run()
{
    
    //   VideoCapture cap(1); // open the default camera
    if(!cap_pioneer.isOpened())  // check if we succeeded
        return;
    
    // Configure the reader
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
    
    // Inicializa variável que armazena a quantidade de vezes que a landmark foi detectada
    contador= 0;
    
    
    while (1)
    {
        
        //       cap_pioneer.open(1);
        
        //cap_pioneer >> frame_pioneer; // get a new frame from camera
        cap_pioneer >> frame_pioneer; // get a new frame from camera
        
        cvtColor(frame_pioneer, frame_grayscale, CV_BGR2GRAY);
        
        // Obtain image data
        width = frame_grayscale.cols;
        height = frame_grayscale.rows;
        raw = (uchar *)(frame_grayscale.data);
        
        // Wrap image data
        Image image(width, height, "Y800", raw, width * height);
        
        // Procura QRCODE ou Código de Barras na imagem capturada
        scanner.scan(image);
        
        // Procura pela landmark
        for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
            
            //cout << pioneer_landmark <<endl ;
            
            if ( ( (symbol->get_data()[0] ) - 48 ) == pioneer_landmark )
            {

                contador ++;

                if ( contador >= 3 )
                {
                    emit PrintLandMarkPionnerAtual();
                    PontoAtingido = true;
                    contador = 0;
                    msleep(1000);
                    //                    //Cálculo da região de busca da landmark no vetor de medidas do laser
                    //                    //angle = ( 60 * ( (symbol->get_location_x(3) + symbol->get_location_x(0)) / 2 ) / width );

                    //                    //angle = 105 - angle;
                    //                    //i = angle;
                    //                    //                cout << "largura = "<< width << endl;
                    //                    //                cout << "x1 = " << symbol->get_location_x(3) << endl;
                    //                    //                cout << "x0 = "<< symbol->get_location_x(0) << endl;
                    //                    //                cout << "angle = " <<angle << endl;
                    //                    //                cout << "angle laser inicial = " << i << endl;
                    //                    //                cout << "angle laser final = " << ( i+30 ) << endl << endl;

                    //                    i = 0;
                    //                    semaforo_leitura_laser.lock();
                    //                    for ( j = 55 ; j <= 125 ; j++ )
                    //                    {
                    //                        laser[i] = readinglaser[j+1] - readinglaser[j];
                    //                        i++;
                    //                    }
                    //                    semaforo_leitura_laser.unlock();

                    //                    // Máquina de estados para procurar a landmark nas medidas do laser
                    //                    i = 0;
                    //                    j = 0;
                    //                    inicio = 0;
                    //                    fim = 0;
                    //                    setMode(Estado_1);

                    //                    LandmarkDetected = true;

                    //                    while ( i <= 70 )
                    //                    {

                    //                        switch (myMode) {

                    //                        case Estado_1:
                    //                        {
                    //                            if ( laser[i] <= -150 )
                    //                            {
                    //                                inicio = i;
                    //                                i++;
                    //                                setMode(Estado_2);
                    //                            }
                    //                            else
                    //                            {
                    //                                i++;
                    //                            }
                    //                            break;
                    //                        }

                    //                        case Estado_2:
                    //                        {
                    //                            if ( laser[i] <= -150 )
                    //                            {
                    //                                i++;
                    //                            }
                    //                            else
                    //                            {
                    //                                if ( laser[i] >= 50 )
                    //                                {
                    //                                    i++;
                    //                                    setMode(Estado_1);
                    //                                }
                    //                                else
                    //                                {
                    //                                    i++;
                    //                                    setMode(Estado_3);
                    //                                }
                    //                            }
                    //                            break;
                    //                        }

                    //                        case Estado_3:
                    //                        {
                    //                            if ( ( laser[i] >= -50 ) && ( laser[i] <= 50 ) )
                    //                            {
                    //                                j++;
                    //                                i++;
                    //                                if (j > 8 )
                    //                                {
                    //                                    j = 0;
                    //                                    setMode(Estado_1);
                    //                                }

                    //                            }
                    //                            else
                    //                            {
                    //                                if ( ( laser[i] >= 50 ) && ( j >= 3 ) && ( j <= 8 ) )
                    //                                {
                    //                                    fim = i;
                    //                                    j = 0;
                    //                                    setMode(Estado_4);
                    //                                }
                    //                                else
                    //                                {
                    //                                    i++;
                    //                                    j = 0;
                    //                                    setMode(Estado_1);
                    //                                }
                    //                            }
                    //                            break;
                    //                        }

                    //                        case Estado_4:
                    //                        {

                    //                            //                            inicio = inicio + angle;
                    //                            //                            fim = fim + angle;
                    //                            //                            posicao = inicio + ( (fim - inicio) / 2 );

                    //                            posicao = 55 + inicio + ( (fim - inicio) / 2 );

                    //                            //cout << "inicio = " << inicio << endl;
                    //                            //cout << "fim = " << fim << endl;
                    //                            //cout << "posicao = " << posicao << endl;

                    //                            // Deve-se somar 5cm do raio do tubo
                    //                            semaforo_leitura_laser.lock();
                    //                            dist_to_landmark = readinglaser[(int)(posicao)] + 50;
                    //                            semaforo_leitura_laser.unlock();
                    //                            //cout << "Distancia = " << dist_to_landmark << endl;

                    //                            // A distância entre o laser e o centro do robô é de 25cm
                    //                            // Essa equação corrige a distância medida pelo laser para a distância
                    //                            // entre o centro do robô e a landmark


                    //                            if ( posicao > 90 )
                    //                            {
                    //                                angle = posicao - 90;
                    //                                dist_to_landmark = sqrt( pow( ( ( dist_to_landmark * cos(angle * PI / 180.0) ) + 250 ),2) + pow( ( dist_to_landmark * sin(angle * PI / 180.0) ),2) );
                    //                            }
                    //                            else
                    //                            {
                    //                                if (angle == 90)
                    //                                {
                    //                                    angle = 0;
                    //                                    dist_to_landmark = dist_to_landmark + 250;
                    //                                }
                    //                                else
                    //                                {
                    //                                    angle = 90 - posicao;
                    //                                    dist_to_landmark = sqrt( pow( ( ( dist_to_landmark * cos(angle * PI / 180.0) ) + 250 ),2) + pow( ( dist_to_landmark * sin(angle * PI / 180.0) ),2) );
                    //                                    angle = -1 * angle;
                    //                                }
                    //                            }

                    //                            //vai_fifi = false;

                    //                            ////////////////////////////////
                    //                            // Correção da posição do Robô//
                    //                            ////////////////////////////////

                    //                            semaforo_landmark.lock();
                    //                            x_robo_real = x_landmark - dist_to_landmark * cos( ( angle + phi ) * PI / 180.0 );
                    //                            y_robo_real = y_landmark - dist_to_landmark * sin( ( angle + phi ) * PI / 180.0 );
                    //                            semaforo_landmark.unlock();

                    //                            semaforo_robo.lock();
                    //                            cout << "angle_landmark = " << angle << endl;
                    //                            cout << "distância = " << dist_to_landmark << endl;
                    //                            cout << "orientação do robo = " << phi << endl;
                    //                            cout << "x_odometria = " << robot.getX() << endl;
                    //                            cout << "y_odometria = " << robot.getY() << endl;
                    //                            cout << "x_corrigido = " << x_robo_real << endl;
                    //                            cout << "y_corrigido = " << y_robo_real << endl;
                    //                            semaforo_robo.unlock();

                    //                            // Alterando x e y do encoder do robô
                    ////                            semaforo_robo.lock();
                    ////                            posicao_corrigida.setPose( x_robo_real , y_robo_real , phi );
                    ////                            robot.setEncoderPose( posicao_corrigida );
                    ////                            semaforo_robo.unlock();

                    //                            msleep(100);

                    //                            // Finaliza a busca da landmark
                    //                            i= 71;
                    //                            //vai_fifi = true;
                    //                            break;
                    //                        }

                    //                        default:
                    //                        {

                    //                            break;
                    //                        }

                    //                        }

                    //                    }
                }
            }
        }
        
        image.set_data(NULL, 0);
        
        if(waitKey(30) >= 0) break;
        
        // Solicita print da imagem na tela
        emit PrintImagePioneer();
        
    }
}




/******************************************************************************************************/
/******************************** FUNÇÃO PARA SELECIONAR O ESTADO *************************************/
/******************************************************************************************************/
void CameraPioneer::setMode(Mode mode)
{
    myMode = mode;
}


