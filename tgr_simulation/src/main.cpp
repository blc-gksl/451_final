#include <ros/ros.h>
#include <gazebo/gazebo_client.hh>
#include "gazebo_msgs/GetModelState.h"
#include <iostream>
#include <vector>
#include <memory>

double distance(double x,double y,double targetx,double targety){
    return (sqrt((targetx-x)*(targetx-x) + (targety-y)*(targety-y)));
}

bool isAchieved(double x,double y,double targetx,double targety){
    double treshold = 0.1;
    return (distance(x,y,targetx,targety)<treshold);
}


bool** addObstacles(bool** myMatrix, double **myObstacles, int N){
    double theDistance;
    for(int k=0; k<N ;k++){
        double myBorder = myObstacles[k][2]+0.1+0.5;

        for(int i=0; i<50; i++){
            for(int j=0; j<50; j++){
                theDistance = sqrt(pow(myObstacles[k][0] - (double)i,2) + pow(myObstacles[k][1] - (double)j,2));
                if(theDistance < myBorder){
                    myMatrix[i][j] = false;
                }
            }
        }
    }
    return myMatrix;
}


geometry_msgs::Point toEulerAngle(geometry_msgs::Quaternion q){
    geometry_msgs::Point euler_angles;
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    euler_angles.x = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (std::fabs(sinp) >= 1)
        euler_angles.y = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
    else
        euler_angles.y = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    euler_angles.z = std::atan2(siny_cosp, cosy_cosp);

    return euler_angles;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
    ros::ServiceClient client ;

    ros::Rate rate(50.0);

    std::string modelName = (std::string)"tgr" ;
    std::string relativeEntityName = (std::string)"world" ;
    gazebo_msgs::GetModelState getModelState ;

    char buffer[256];
    char *val = getcwd(buffer, sizeof(buffer)); //it takes the current path of workspace
    if (val) {
        std::cout << buffer << std::endl;
    }

    char file_path[256] = "/../../../src/tgr_simulation/read.txt";
    std::string path;
    path = strcat(buffer,file_path);
    std::fstream myfile;
    std::cout << path << std::endl;
    myfile.open(path, std::ios_base::in);
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
        }

        for(int j=0; j<2;j++){
            myfile >> receiver[i][j];
        }
    }
    myfile >> N;
    double **myObstacles = new double*[N];
    for(int i=0;i<N;i++){
        myObstacles[i] = new double[4];
    }

    for(int i=0; i< N; i++){
        for(int j=0; j<4;j++){
            myfile >> myObstacles[i][j];
        }
    }
        for(int i=0; i< N; i++){
        for(int j=0; j<4;j++){
            myfile >> myObstacles[i][j];
        }
    }
    bool **myMatrix = new bool*[50];
    for(int i=0;i<50;i++){
        myMatrix[i] = new bool[50];
    }
    // for the start initialize all points to be able to use in the path
    for(int i = 0; i < 50; ++i){
        for(int j = 0; j < 50; ++j){
            myMatrix[i][j] = true;

        }
    }

    //obstac[engelin numarasý][0,1,2,3] ikinci kýsým 0 xi 1 yyi 2radiusu 3de heightýný döndürüyor
    // create Obstacle matrix with a dynamic 2D array matrix -> myObstacles[obstacleNumber-1][x,y,r,h]
    myMatrix = addObstacles(myMatrix, myObstacles, N);

    //print the map
    for(int i=0; i<50; i++) {
        for (int j = 0; j < 50; j++) {
            std::cout << myMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
    while (ros::ok())
    {
        static double previous_angle = 0;
        geometry_msgs::Quaternion q = getModelState.response.pose.orientation;
        static double uncertainty = 0;
        client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state") ;
        getModelState.request.model_name = modelName ;
        getModelState.request.relative_entity_name = relativeEntityName ;
        client.call(getModelState);
        double x = getModelState.response.pose.position.x;
        double y = getModelState.response.pose.position.y;
        double yaw = toEulerAngle(q).z ;
        double target_x = 5;
        double target_y = 5;
        double angle_diff = atan2((target_y - y),(target_x-x)) - yaw;
        double d_angle = angle_diff - previous_angle;
        previous_angle = angle_diff;
        double vel = 1; // m/s
        if(abs(angle_diff) > 0.3) vel = 0.1;

        double P_gain = 1, D_gain = 0.1;


        geometry_msgs::Twist vel_output;
        vel_output.angular.x = 0;
        vel_output.angular.y = 0;
        vel_output.angular.z = P_gain * angle_diff + D_gain * d_angle;
        vel_output.linear.x = vel;
        vel_output.linear.y = 0;
        vel_output.linear.z = 0;
        cmd_vel_pub.publish(vel_output);


        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}