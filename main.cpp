#include <QtGui/QApplication>
#include "projetoomega.h"
#include "ArAnalogGyro.h"

//Variáveis globais para Robô e Laser
ArRobot         robot;                                    //Instantiate robot
ArSick          laser;                                    //Instantiate laser

/******************************************************************************************************/
/********************************** INICIALIZAÇÃO DO ARIA E DA UI *************************************/
/******************************************************************************************************/
int main(int argc, char *argv[])
{

    // Inicialização do Aria de acordo com a apostila.

    // ARIA Initialization
    
    Aria::init();    //Initialise ARIA library
    
    ArArgumentParser parser(&argc, argv);       //Instantiate argument parser
    ArSimpleConnector connector(& parser);      //Instantiate connector
    
    ArAnalogGyro    myGyro(&robot);
    
    
    /* Connection to robot */
    parser.loadDefaultArguments();              //Load default values
    
    if (!connector.parseArgs())                 //Parse connector arguments
    {
        cout << "Unknown settings\n";           //Exit for errors
        Aria::exit(0);
        exit(1);
    }
    
    
    if (!connector.connectRobot(&robot))        //Connect to the robot
    {
        cout << "Unable to connect\n";          //Exit for errors
        Aria::exit(0);
        exit(1);
    }
    
    robot.runAsync(true);                       //Run in asynchronous mode
    robot.lock();                               //Lock robot during set up
    robot.enableMotors();
    robot.comInt(ArCommands::SOUNDTOG, 0);
    robot.comInt(ArCommands::ENABLE, 1);        //Turn on the motors
    robot.unlock();                             //Unlock the robot

    //  Inicialização do Laser
    
    robot.addRangeDevice(&laser);               //Add laser to robot
    laser.runAsync();                           //Asynchronous laser mode
    
    if (!connector.connectLaser(&laser))  //Connect laser to robot
    {
        cout << "Can’t connect to laser\n";     //Exit if error
        Aria::exit(0);
        exit(1);
    }
    laser.asyncConnect();                       //Asynchronous laser mode
    

    // Final da inicialização

    //    Inicialização alternativa para o laser
    //    connector.setupLaser(&laser);
    //    laser.runAsync();
    //    if (!laser.blockingConnect())
    //    {
    //    cout << "Could not connect to SICK laser... exiting" << endl;
    //    Aria::exit(0);
    //    exit(1);
    //    }


    QApplication a(argc, argv);
    ProjetoOmega w;
    w.show();
    
    return a.exec();
}
