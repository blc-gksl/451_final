//
// Created by ahmet on 1.12.2020.
//

#include "common.h"


geometry_msgs::Point common::toEulerAngle(geometry_msgs::Quaternion q){
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

geometry_msgs::Point common::line_direction(tgr_msgs::Line a){
    geometry_msgs::Point direction_of;

    float len=sqrt((a.point_end.y-a.point_begin.y)*(a.point_end.y-a.point_begin.y)+(a.point_end.x-a.point_begin.x)*(a.point_end.x-a.point_begin.x));
    direction_of.y=(a.point_end.y-a.point_begin.y)/len;
    direction_of.x=(a.point_end.x-a.point_begin.x)/len;
    direction_of.z=0;
    return direction_of;
}

geometry_msgs::Point common::projection_on_line(geometry_msgs::Point a, tgr_msgs::Line l){
    float c=common::line_direction(l).x;
    float d=common::line_direction(l).y;
    float m =d/c;
    float Xl=l.point_begin.x;
    float Yl=l.point_begin.y;
    float b=Yl-m*Xl;

    geometry_msgs::Point projected;
    projected.x=(a.x+m*a.y-m*b)/(1.0+m*m);
    projected.y=(m*a.x+m*m*a.y+b)/(1.0+m*m);
    projected.z=0;
    return projected;
}

float common::distance_between_points(geometry_msgs::Point a, geometry_msgs::Point b) {
    float distance = sqrt(pow((a.x-b.x), 2.0) + pow((a.y-b.y),2.0));

    return distance;

}

float common::distance_to_line(geometry_msgs::Point a, tgr_msgs::Line l) {

    geometry_msgs::Point projected = common::projection_on_line(a, l);
    float distance = common::distance_between_points(a, projected);
    return distance;

}
