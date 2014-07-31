#include <QtGui/QApplication>
#include "projetoomega.h"


ArRobot robot;                                   //Instantiate robot
ArSick laser;                                    //Instantiate laser

int main(int argc, char *argv[])
{


    // ARIA Initialization

    Aria::init();    //Initialise ARIA library

    ArArgumentParser parser(&argc, argv);       //Instantiate argument parser
    ArSimpleConnector connector(& parser);      //Instantiate connector
    /* Connection to robot */
    parser.loadDefaultArguments();              //Load default values
    if (!connector.parseArgs())                 //Parse connector arguments
    {
        cout << "Unknown settings\n";               //Exit for errors
        Aria::exit(0);
        exit(1);
    }


    if (!connector.connectRobot(&robot))        //Connect to the robot 13
    {
        cout << "Unable to connect\n";          //Exit for errors 14
        Aria::exit(0);
        exit(1);
    }

    robot.runAsync(true); //Run in asynchronous mode
    robot.lock(); //Lock robot during set up
    robot.comInt(ArCommands::ENABLE, 1); //Turn on the motors
    robot.unlock(); //Unlock the robot

    //Inicialização do Laser

    robot.addRangeDevice(&laser);                    //Add laser to robot
    laser.runAsync();                                //Asynchronous laser mode

    if (!connector.connectLaser(&laser))             //Connect laser to robot
    {
        cout << "Can’t connect to laser\n";              //Exit if error
        Aria::exit(0);
        exit(1);
    }
    laser.asyncConnect();                           //Asynchronous laser mode

    // Final da inicialização

//    Mat frodao;
//    VideoCapture fifizao("http://10.5.5.9:8080/live/amba.m3u8");

//    do{
//        fifizao >> frodao;
//        if(frodao.empty())
//        {
//            return -1;
//        }

//        else
//        {
//            imshow("patrone",frodao);
//        }

//    }while(cvWaitKey(1)!='q');

//        return 0;

        QApplication a(argc, argv);
    ProjetoOmega w;
    w.show();

    return a.exec();
}
