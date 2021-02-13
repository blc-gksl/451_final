#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <algorithm>

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
    int r=0;
    path[0][0] = 0; path[0][1]=0;
    vector <int> se;
    for(int i=0; i<2*M + 1; i++){
        double min_distance = 1000;
        double de = 0;
        bool sent = true;
        bool brek = false;
        int c=0;
        static int a = 0;
        for(int j=0; j<sender.size();j++){
            for(int count = 0 ; count <se.size() ; count++){
                int q = se[count];
                if(sender[j] == sender[q]){
                    brek = true;
                }
            }
            if(brek) {
                brek = false;
                continue;
            }
            de = distance(path[i][0], path[i][1], sender[j].first, sender[j].second);
            if(min_distance > de) {
                min_distance = de;
                c = j;
                sent=true;
            }
        }
        for (int k=0; k<se.size() ; k++){
            de = distance(path[i][0], path[i][1], receiver[se[k]].first, receiver[se[k]].second);
            if(min_distance > de){
                min_distance = de;
                sent = false;
                c=k;
                cout << de << endl;
            }
        }
        if(sent){
            path[i+1][0] = sender[c].first;path[i+1][1] = sender[c].second;

            se.push_back(c);

        }
        else {
            path[i+1][0] = receiver[se[c]].first;path[i+1][1] = receiver[se[c]].second;

                for(int l=0 ; l<se.size() ;l++){
                    if(se[c] < se[l]){
                        se[l]=se[l] -1;
                    }
                }
            receiver.erase(receiver.begin() + se[c]);
            sender.erase(sender.begin() + se[c]);
            se.erase(se.begin() + c);
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