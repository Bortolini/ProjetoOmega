/*****************************************************************************************/
/*****************************************************************************************/
/************************** Função para carregar o mapa **********************************/
/*****************************************************************************************/
/*****************************************************************************************/

// O mapa, em milímetros, deve ser um arquivo de texto organizado da seguinte forma:

//        4
//        0 12 999 5 0 0
//        12 0 999 999 1 4
//        999 0 7 999 5 4
//        5 7 999 0 8 8

// O primeiro número indica a dimensão do mapa (quantidade de landmarks).
// A matriz 4x4 abaixo da dimensão são os pesos e as conexões entre as landmarks.
// O vetor 4x2, a direita da matriz, indica a as coordenadas das landmarks.
// O número 999 representa peso infinito, e o valor 0 indica a ligação do ponto com ele mesmo.

#include "LoadMap.h"

//Variáveis globais relacionadas ao mapa e aos nós

cord *coordenada; //Coordenadas x y das landmarks

int **mapa;  //Mapa com os pesos e as conexões

int dim = 0; //Dimensão do mapa

int origem, alvo, tamanho;

int *caminho;


/******************************************************************************************************/
/************************************ FUNÇÃO PARA CARREGAR O MAPA *************************************/
/******************************************************************************************************/
void loadMapFile(){

    int i = -1;
    int j = 0;

    // Abre arquivo
    QFile inputFile(":/mapa01.txt");
    inputFile.open(QIODevice::ReadOnly);

    // Transfere dados do arquivo para a QString dados
    QString dados = inputFile.readAll();
    inputFile.close();

    // Trata individualmente cada Char da QString dados
    foreach (QString str, dados)
    {
        // Testa espaços e quebras de linha entre os dados do mapa
        if ( ( str != "\n" ) && ( str != " ") ){

            if (i>=0){


                if (i<dim){
                    // Aramazena os pesos e as conexões no mapa
                    mapa[j][i] = mapa[j][i] *10 + str.toInt();
                }
                else{
                    // Aramazena as coordenadas das landmarks
                    if (i==dim){
                        coordenada[j].x= coordenada[j].x*10 + str.toInt();
                        if ((coordenada[j].x>coordenada[dim].x) && (coordenada[j].x>coordenada[dim].y))
                            coordenada[dim].x = coordenada[j].x;
                    }
                    else{
                        coordenada[j].y= coordenada[j].y*10 + str.toInt();
                        if ((coordenada[j].y>coordenada[dim].x) && (coordenada[j].y>coordenada[dim].y))
                            coordenada[dim].x = coordenada[j].y;

                    }
                }

            }

            else{

                // Armazena a dimensão do mapa em dim
                dim = dim *10 + str.toInt();
            }
        }
        else
        {

            if (i == -1){

                // Cria matriz dinâmicamente de dimensão dim para armazenar o mapa
                mapa = new int*[dim];

                for ( i = 0 ; i < (dim) ; i++ )

                    mapa[i] = new int[dim];


                for (i=0; i<dim; i++)

                    for (j=0; j<dim; j++)
                        mapa[i][j] = 0;


                // Cria struct com as coordenadas x y das landmarks
                coordenada = new cord[dim+1];
                for ( i = 0 ; i < dim+1 ; i++){
                    coordenada[i].x= 0;
                    coordenada[i].y= 0;
                }

                i=-1;
                j=0;
            }

            // Verifica final da linha

            if (i < dim+1)
            {
                i++;
            }

            // Incrementa o número da linnha e zera o vetor coluna
            else{
                i = 0;
                j++;
            }


        }

    }


    // Espelha eixo y
    for (i=0; i<dim+1; i++)
        coordenada[i].y = -1*coordenada[i].y;

}
