#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>
#include <gazebo_cube_spawner.h>
#include <string>
#include <iostream>
#include <fstream>

using gazebo_test_tools::GazeboCubeSpawner;
using ros::NodeHandle;


#define SPAWN_OBJECT_TOPIC "gazebo/spawn_sdf_model"

GazeboCubeSpawner::GazeboCubeSpawner(NodeHandle &n) : nh(n){
    spawn_object = n.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_OBJECT_TOPIC);
}


void GazeboCubeSpawner::spawnPrimitive(const std::string& name,const std::string& color, const bool doCube,
                                       const std::string& frame_id,
                                       float x, float y, float z, float qx, float qy, float qz, float qw,
                                       float widthOrRadius, float height, float depth, float _mass)
{

    geometry_msgs::Pose pose;
    pose.position.x=x;
    pose.position.y=y;
    pose.position.z=z;
    pose.orientation.x=qx;
    pose.orientation.y=qy;
    pose.orientation.z=qz;
    pose.orientation.w=qw;

    gazebo_msgs::SpawnModel spawn;
    spawn.request.model_name=name;

    // just so the variable names are shorter..
    float w=widthOrRadius;
    float h=height;
    float d=depth;

    std::stringstream _s;
    if (doCube)
    {
        _s<<"<box>\
            <size>"<<w<<" "<<h<<" "<<d<<"</size>\
          </box>";
    }else{

        _s<<"<cylinder>\
                <length>"<<h<<"</length>\
                <radius>"<<w<<"</radius>\
            </cylinder>";
    }
    std::string geometryString = _s.str();


    float mass=_mass;
    float mass12=mass/12.0;

    double mu1=500; //500 for PR2 finger tip. In first experiment had it on 1000000
    double mu2=mu1;
    double kp=10000000; //10000000 for PR2 finger tip
    double kd=1; //100 for rubber? 1 fir OR2 finger tip

    bool do_surface=false;
    bool do_inertia=true;

    std::stringstream s;\
    s<<"<?xml version='1.0'?>\
    <sdf version='1.4'>\
    <model name='"<<name<<"'>\
        <static>false</static>\
        <link name='link'>";

    // inertia according to https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    if (do_inertia)
    {
        double xx, yy, zz;
        if (doCube)
        {
            xx=mass12*(h*h+d*d);
            yy=mass12*(w*w+d*d);
            zz=mass12*(w*w+h*h);
        }
        else
        {
            xx=mass12*(3*w*w + h*h);
            yy=mass12*(3*w*w + h*h);
            zz=0.5*mass*w*w;
        }
        s<<"<inertial>\
        <mass>"<<mass<<"</mass>\
        <inertia>\
          <ixx>"<<xx<<"</ixx>\
          <ixy>0.0</ixy>\
          <ixz>0.0</ixz>\
          <iyy>"<<yy<<"</iyy>\
          <iyz>0.0</iyz>\
          <izz>"<<zz<<"</izz>\
        </inertia>\
          </inertial>";
    }

       if(!doCube) {
           s<<"<collision name='collision'>\
        <geometry>"<<geometryString;
           s<<"</geometry>\
           </collision>";

        }
       else            s << "<gravity>0</gravity>";
    s<<"<visual name='visual'>";
    s<<"<geometry>"<<geometryString;
    s<<"</geometry>\
        <material>\
            <script>\
                <uri>file://media/materials/scripts/gazebo.material</uri> \
                <name>Gazebo/"<<color;
    s<<"</name>\
            </script>\
        </material>\
          </visual>\
        </link>\
      </model>\
    </sdf>";

    spawn.request.model_xml=s.str();
    spawn.request.robot_namespace="cube_spawner";
    spawn.request.initial_pose=pose;
    spawn.request.reference_frame=frame_id;

    //ROS_INFO("Resulting model: \n %s",s.str().c_str());

    //ROS_INFO("Waiting for service");
    spawn_object.waitForExistence();
    //ROS_INFO("Calling service");

    //std::cout<<spawn.request<<std::endl;

    if (!spawn_object.call(spawn)) {
        ROS_ERROR("Failed to call service %s",SPAWN_OBJECT_TOPIC);
    }
    ROS_INFO("Result: %s, code %u",spawn.response.status_message.c_str(), spawn.response.success);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_cube_spawner");


    ros::NodeHandle node;
    gazebo_test_tools::GazeboCubeSpawner spawner(node);
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
        }
        std::string text = "sender";
        text += std::to_string(i);
        spawner.spawnPrimitive(text,"Blue",true,"world",sender[i][0],sender[i][1],0.25,0,0,0,1,1,1,0.5,500000);

        for(int j=0; j<2;j++){
            myfile >> receiver[i][j];
        }
        text = "receiver";
        text += std::to_string(i);
        spawner.spawnPrimitive(text,"Green",true,"world",receiver[i][0],receiver[i][1],0.25,0,0,0,1,1,1,0.5,500000);
    }
    myfile >> N;
    double obstacles[N][4];

    for(int i=0; i< N; i++){
        for(int j=0; j<4;j++){
            myfile >> obstacles[i][j];
        }
        std::string text = "obstacle";
        text += std::to_string(i);
        spawner.spawnPrimitive(text,"Yellow",false,"world",obstacles[i][0],obstacles[i][1],obstacles[i][3]/2,0,0,0,1,obstacles[i][2],obstacles[i][3],0.00001,5);

    }


    return 0;
}