#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
using namespace std;
double distance(double x,double y,double targetx,double targety){
    return (sqrt((targetx-x)*(targetx-x) + (targety-y)*(targety-y)));
}

typedef pair<int, int> Pair;

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
    vector< pair <int,int> > sender;
    vector< pair <int,int> > receiver;
    double a,b,c,d;

    for(int i=0; i< M; i++){

            myfile >> a;
            myfile >> b;
        sender.push_back(make_pair(a, b));
//            std::cout<< "sender "<<sender[i].first <<" " << sender[i].second<<std::endl;
            myfile >> c;
            myfile >> d;
        receiver.push_back(make_pair(c, d));

//            std::cout<< "receiver "<<receiver[i].first <<" " << receiver[i].second<<std::endl;
    }
    myfile >> N;
    double obstacles[N][4];

    for(int i=0; i< N; i++){
        for(int j=0; j<4;j++){
            myfile >> obstacles[i][j];
            std::cout<< "obstacles "<<obstacles[i][j]<<std::endl;
        }
    }
    double path[2*M+1][2];
    int s=M,r=0;
    path[0][0] = 0; path[0][1]=0;
    int se[M];
    for(int i=0; i<2*M + 1; i++){
        double min_distance = 100;
        bool sent = true;
        int c;
        static int a = 0;

        for(int j=0; j<s;j++){
            double d = distance(path[i][0],path[i][1],sender[j].first,sender[j].second);
            if(min_distance > d) {
                min_distance = d;
                c = j;
            }
        }
        for (int k=r; k<(M-s) ; k++){
            double d = distance(path[i][0],path[i][1],receiver[se[k]].first,receiver[se[k]].second);
            if(min_distance > d){
                min_distance = d;
                sent = false;
                c=se[k];
            }
        }
        if(sent){
            path[i+1][0] = sender[c].first;path[i+1][1] = sender[c].second;
            se[a]=c;
            sender.erase(sender.begin() + c);
            a++;
            s--;
        }
        else {
            path[i+1][0] = receiver[c].first;path[i+1][1] = receiver[c].second;
            receiver.erase(receiver.begin() + c);

            r++;
        }

    }
    for(int i=0; i< 2*M+1; i++){
        for(int j=0; j<2;j++){
            std::cout<<path[i][j]<< " ";
        }
        cout<<std::endl;
    }

    return 0;
}