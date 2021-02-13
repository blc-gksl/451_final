//
// Created by ahmet on 1.12.2020.
//

#ifndef SRC_COMMON_H
#define SRC_COMMON_H

#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tgr_msgs/Line.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>



class common {
public:
    static geometry_msgs::Point toEulerAngle(geometry_msgs::Quaternion q);
    static geometry_msgs::Point line_direction(tgr_msgs::Line a);
    static geometry_msgs::Point projection_on_line(geometry_msgs::Point a, tgr_msgs::Line l);
    static float distance_between_points (geometry_msgs::Point a, geometry_msgs::Point b);
    static float distance_to_line(geometry_msgs::Point a, tgr_msgs::Line l);



};


#endif //SRC_COMMON_H
