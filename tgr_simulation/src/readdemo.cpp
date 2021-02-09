#include <iostream>
#include <fstream>
#include <vector>


int main(int argc, char * argv[])
{
    std::fstream myfile;
    myfile.open("/home/berkay/finalprj_ws/src/tgr_simulation/read.txt", std::ios_base::in);
    if (!myfile)
    {
        std::cout << "\nError opening file.\n";
        return 0;
    }

    int M,N;
    myfile >> M;

   double sender[M][2];
   double receiver[M][2];
    for(int i=0; i< M; i++){
        for(int j=0; j<2;j++){
            myfile >> sender[i][j];
            std::cout<< "sender "<<sender[i][j]<<std::endl;
        }
        for(int j=0; j<2;j++){
            myfile >> receiver[i][j];
            std::cout<< "receiver "<<receiver[i][j]<<std::endl;
        }
    }
    myfile >> N;
    double obstacles[N][4];

    for(int i=0; i< N; i++){
        for(int j=0; j<4;j++){
            myfile >> obstacles[i][j];
            std::cout<< "obstacles "<<obstacles[i][j]<<std::endl;
        }
    }


    return 0;
}