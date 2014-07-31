#include "CameraQuadrotor.h"

CameraQuadrotor::CameraQuadrotor(QObject *parent) :
    QThread(parent)
{
}

/******************************************************************************************************/
/******************** THREAD PARA CAPTURA E PROCESSAMENTO DAS IMAGENS DO PIONEER **********************/
/******************************************************************************************************/

/******************************************************************************************************/
/******************************** UTILIZANDO A VIDEOCAPTURE DO OPENCV *********************************/
/******************************************************************************************************/
void CameraQuadrotor::run()
{


    //   VideoCapture cap(0); // open the default camera
    if(!cap_quadrotor.isOpened())  // check if we succeeded
        return;

    // Configure the reader
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    for(;;)
    {
        cap_quadrotor >> frame_quadrotor;

        //Converter de BGR para HSV
        cvtColor(frame_quadrotor,frame_hsv, CV_BGR2HSV);

        //Binarização para selecionar a cor azul
        inRange(frame_hsv, Scalar(25,100,0), Scalar(35,200,255), frame_bin);

        //Filtro para retirar ruidos de outras cores (não azul)
        //cvSmooth(frame_bin, frame_bin2, CV_MEDIAN, 3, 3);
        medianBlur(frame_bin, frame_bin, 5);

        //Converter para escala de cinza para usar no Zbar
        cvtColor(frame_quadrotor, frame_grayscale, CV_BGR2GRAY);

        // Obtain image data
        width = frame_grayscale.cols;
        height = frame_grayscale.rows;
        raw = (uchar *)(frame_grayscale.data);

        // Wrap image data
        Image image(width, height, "Y800", raw, width * height);

        scanner.scan(image);

        counter = 0;

        for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
            time_t now;
            tm *current;
            now = time(0);
            current = localtime(&now);

            // do something useful with results
            cout    << "[" << current->tm_hour << ":" << current->tm_min << ":" << setw(2) << setfill('0') << current->tm_sec << "] " << counter << " "
                    << "decoded " << symbol->get_type_name()
                    << " symbol \"" << symbol->get_data() << '"' << endl;

            if ( symbol->get_data() == "1" )
            {
                //cout << endl << cout<< "achei" << endl;
                cout<< endl;
                cout<< "achei" ;
            }

            //cout << "Location: (" << symbol->get_location_x(0) << "," << symbol->get_location_y(0) << ")" << endl;
            //cout << "Size: " << symbol->get_location_size() << endl;

            // Draw location of the symbols found
            if (symbol->get_location_size() == 4) {
                //rectangle(frame, Rect(symbol->get_location_x(i), symbol->get_location_y(i), 10, 10), Scalar(0, 255, 0));
                line(frame_quadrotor, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame_quadrotor, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame_quadrotor, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame_quadrotor, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(0, 255, 0), 2, 8, 0);
            }

            // Get points
            /*for (Symbol::PointIterator point = symbol.point_begin(); point != symbol.point_end(); ++point) {
                         cout << point << endl;
                     } */
            counter++;
        }

        image.set_data(NULL, 0);



        if(waitKey(30) >= 0) break;

        frame_quadrotor = frame_bin;

        emit PrintImageQuadrotor();
    }
    // the camera will be deinitialized automatically in VideoCapture destructor


}
