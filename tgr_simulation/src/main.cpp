#include <ros/ros.h>
#include <gazebo/gazebo_client.hh>
#include "gazebo_msgs/GetModelState.h"
#include <iostream>
#include <vector>
#include <memory>
#include <tgr_msgs/Line.h>
#include <common.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <bits/stdc++.h>
#include <bits/stdc++.h>
#include <nav_msgs/Odometry.h>
const double pi = 3.14159265358979;
using namespace std;

#define ROW 50
#define COL 50

float look_ahead_dist = 5.0;  //in meters. for carrot guidance.
float cruise_vel = 2;  //in meters/sec. maximum and straight line velocity.
float vehicle_width = 0.71;   //in meters. distance between right-left wheels.
float Pgain_direction= 1;
float Dgain_direction= -0.05;//controller gains for reahing commanded direction.
bool turn_direction;

geometry_msgs::Twist line_follower(tgr_msgs::Line l, nav_msgs::Odometry vehicle_pose){    //implements the carrot guidance algorithm. output is either thrust + angular velocity or left wheel velocity + right wheel velocity.


    double theta_command;
    double theta_current = common::toEulerAngle(vehicle_pose.pose.pose.orientation).z;
    double theta_rate_current = vehicle_pose.twist.twist.angular.z;
    double theta_error;
    float vel_right_wheel;
    float vel_left_wheel;
    bool turn_direction;  //0:clockwise, 1:counter clockwise



    if((l.point_end.y==l.point_begin.y && l.point_end.x ==l.point_begin.x) &&(l.point_end.y!=0 || l.point_end.x!=0)){

        theta_command = atan2(l.point_end.y, l.point_end.x);

    }
    else{
        geometry_msgs::Point projected_point;
        projected_point.x = common::projection_on_line(vehicle_pose.pose.pose.position, l).x;
        projected_point.y = common::projection_on_line(vehicle_pose.pose.pose.position, l).y;

        geometry_msgs::Point ahead_point;
        ahead_point.x = projected_point.x + common::line_direction(l).x*look_ahead_dist;
        ahead_point.y = projected_point.y + common::line_direction(l).y*look_ahead_dist;

        theta_command = atan2(ahead_point.y - vehicle_pose.pose.pose.position.y , ahead_point.x - vehicle_pose.pose.pose.position.x );

    }

    if(theta_command<0){theta_command=theta_command + 2*pi;}

    if(theta_current<0){theta_current=theta_current + 2*pi;}


    if(theta_command>theta_current){    //chooses the turn direction
        if(theta_command-theta_current<pi)  {turn_direction=1;}
        else                                {turn_direction=0;}
    }
    else{
        if(theta_current-theta_command<pi)  {turn_direction=0; }
        else                                {turn_direction=1;}
    }

    if(turn_direction)  {theta_error=theta_command-theta_current;   }
    else                {theta_error=theta_current-theta_command;   theta_rate_current=-theta_rate_current;}

    if(theta_error<0) {theta_error=theta_error + 2*pi;}

    if(l.point_end.y==l.point_begin.y && l.point_end.x ==l.point_begin.x){
        if(l.point_end.y==0 && l.point_end.x==0){
            vel_right_wheel=0;
            vel_left_wheel=0;
        }
        else {
            if (turn_direction) {
                vel_right_wheel = vehicle_width * (Pgain_direction * theta_error + Dgain_direction * theta_rate_current);  //thera_rate_current is also equal to d(theta_error)/d(t) and has less noise.
                vel_left_wheel = -vehicle_width * (Pgain_direction * theta_error + Dgain_direction * theta_rate_current);
            }
            else {
                vel_right_wheel = -vehicle_width * (Pgain_direction * theta_error + Dgain_direction * theta_rate_current);
                vel_left_wheel = vehicle_width * (Pgain_direction * theta_error + Dgain_direction * theta_rate_current);
            }
        }
    }
    else{
        if(turn_direction){
            vel_right_wheel = cruise_vel*((pi-theta_error)/pi) + vehicle_width*(Pgain_direction*theta_error + Dgain_direction*theta_rate_current);  //thera_rate_current is also equal to d(theta_error)/d(t) and has less noise.
            vel_left_wheel   = cruise_vel*((pi-theta_error)/pi) - vehicle_width*(Pgain_direction*theta_error + Dgain_direction*theta_rate_current);
        }
        else{
            vel_right_wheel = cruise_vel*((pi-theta_error)/pi) - vehicle_width*(Pgain_direction*theta_error + Dgain_direction*theta_rate_current);
            vel_left_wheel   = cruise_vel*((pi-theta_error)/pi) + vehicle_width*(Pgain_direction*theta_error + Dgain_direction*theta_rate_current);
        }
    }


    /*    the output version that gives right, left wheel velocities. Not suitable for simulation.
    tgr_msgs::MotorVal;
    MotorVal.right = vel_right_wheel;
    MotorVal.left = vel_left_wheel;

     return MotorVal;
    */

    geometry_msgs::Twist output;

    output.linear.x = (vel_right_wheel+vel_left_wheel)/2.0;
    output.angular.z= (vel_right_wheel-vel_left_wheel)/vehicle_width;

    return output;


}

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int> > pPair;

// A structure to hold the neccesary parameters
struct cell {
    // Row and Column index of its parent
    // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    int parent_i, parent_j;
    // f = g + h
    double f, g, h;
};

// A Utility Function to check whether given cell (row, col)
// is a valid cell or not.
bool isValid(int row, int col)
{
    // Returns true if row number and column number
    // is in range
    return (row >= 0) && (row < ROW) && (col >= 0)
           && (col < COL);
}

// A Utility Function to check whether the given cell is
// blocked or not
bool isUnBlocked(int grid[][COL], int row, int col)
{
    // Returns true if the cell is not blocked else false
    if (grid[row][col] == 1)
        return (true);
    else
        return (false);
}

// A Utility Function to check whether destination cell has
// been reached or not
bool isDestination(int row, int col, Pair dest)
{
    if (row == dest.first && col == dest.second)
        return (true);
    else
        return (false);
}

// A Utility Function to calculate the 'h' heuristics.
double calculateHValue(int row, int col, Pair dest)
{
    // Return using the distance formula
    return ((double)sqrt(
            (row - dest.first) * (row - dest.first)
            + (col - dest.second) * (col - dest.second)));
}

// A Utility Function to trace the path from the source
// to destination

vector< pair <int,int> > Path;
void tracePath(cell cellDetails[][COL], Pair dest)
{
    printf("\nThe Path is ");
    int row = dest.first;
    int col = dest.second;


    while (!(cellDetails[row][col].parent_i == row
             && cellDetails[row][col].parent_j == col)) {
        Path.push_back(make_pair(row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }

    Path.push_back(make_pair(row, col));
//    while (!Path.empty()) {
//        pair<int, int> p = Path.top();
//        Path.pop();
//        printf("-> (%d,%d) ", p.first, p.second);
//    }

    return;
}

// A Function to find the shortest path between
// a given source cell to a destination cell according
// to A* Search Algorithm
void aStarSearch(int grid[][COL], Pair src, Pair dest)
{
    // If the source is out of range
    if (isValid(src.first, src.second) == false) {
        printf("Source is invalid\n");
        return;
    }

    // If the destination is out of range
    if (isValid(dest.first, dest.second) == false) {
        printf("Destination is invalid\n");
        return;
    }

    // Either the source or the destination is blocked
    if (isUnBlocked(grid, src.first, src.second) == false
        || isUnBlocked(grid, dest.first, dest.second)
           == false) {
        printf("Source or the destination is blocked\n");
        return;
    }

    // If the destination cell is the same as source cell
    if (isDestination(src.first, src.second, dest)
        == true) {
        printf("We are already at the destination\n");
        return;
    }

    // Create a closed list and initialise it to false which
    // means that no cell has been included yet This closed
    // list is implemented as a boolean 2D array
    bool closedList[ROW][COL];
    memset(closedList, false, sizeof(closedList));

    // Declare a 2D array of structure to hold the details
    // of that cell
    cell cellDetails[ROW][COL];

    int i, j;

    for (i = 0; i < ROW; i++) {
        for (j = 0; j < COL; j++) {
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }

    // Initialising the parameters of the starting node
    i = src.first, j = src.second;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;

    /*
     Create an open list having information as-
     <f, <i, j>>
     where f = g + h,
     and i, j are the row and column index of that cell
     Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
     This open list is implenented as a set of pair of
     pair.*/
    set<pPair> openList;

    // Put the starting cell on the open list and set its
    // 'f' as 0
    openList.insert(make_pair(0.0, make_pair(i, j)));

    // We set this boolean value as false as initially
    // the destination is not reached.
    bool foundDest = false;

    while (!openList.empty()) {
        pPair p = *openList.begin();

        // Remove this vertex from the open list
        openList.erase(openList.begin());

        // Add this vertex to the closed list
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;

        /*
         Generating all the 8 successor of this cell

             N.W   N   N.E
               \   |   /
                \  |  /
             W----Cell----E
                  / | \
                /   |  \
             S.W    S   S.E

         Cell-->Popped Cell (i, j)
         N -->  North       (i-1, j)
         S -->  South       (i+1, j)
         E -->  East        (i, j+1)
         W -->  West           (i, j-1)
         N.E--> North-East  (i-1, j+1)
         N.W--> North-West  (i-1, j-1)
         S.E--> South-East  (i+1, j+1)
         S.W--> South-West  (i+1, j-1)*/

        // To store the 'g', 'h' and 'f' of the 8 successors
        double gNew, hNew, fNew;

        //----------- 1st Successor (North) ------------

        // Only process this cell if this is a valid one
        if (isValid(i - 1, j) == true) {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i - 1, j, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i - 1][j].parent_i = i;
                cellDetails[i - 1][j].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }
                // If the successor is already on the closed
                // list or if it is blocked, then ignore it.
                // Else do the following
            else if (closedList[i - 1][j] == false
                     && isUnBlocked(grid, i - 1, j)
                        == true) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i - 1, j, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i - 1][j].f == FLT_MAX
                    || cellDetails[i - 1][j].f > fNew) {
                    openList.insert(make_pair(
                            fNew, make_pair(i - 1, j)));

                    // Update the details of this cell
                    cellDetails[i - 1][j].f = fNew;
                    cellDetails[i - 1][j].g = gNew;
                    cellDetails[i - 1][j].h = hNew;
                    cellDetails[i - 1][j].parent_i = i;
                    cellDetails[i - 1][j].parent_j = j;
                }
            }
        }

        //----------- 2nd Successor (South) ------------

        // Only process this cell if this is a valid one
        if (isValid(i + 1, j) == true) {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i + 1, j, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i + 1][j].parent_i = i;
                cellDetails[i + 1][j].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }
                // If the successor is already on the closed
                // list or if it is blocked, then ignore it.
                // Else do the following
            else if (closedList[i + 1][j] == false
                     && isUnBlocked(grid, i + 1, j)
                        == true) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i + 1, j, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i + 1][j].f == FLT_MAX
                    || cellDetails[i + 1][j].f > fNew) {
                    openList.insert(make_pair(
                            fNew, make_pair(i + 1, j)));
                    // Update the details of this cell
                    cellDetails[i + 1][j].f = fNew;
                    cellDetails[i + 1][j].g = gNew;
                    cellDetails[i + 1][j].h = hNew;
                    cellDetails[i + 1][j].parent_i = i;
                    cellDetails[i + 1][j].parent_j = j;
                }
            }
        }

        //----------- 3rd Successor (East) ------------

        // Only process this cell if this is a valid one
        if (isValid(i, j + 1) == true) {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i, j + 1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i][j + 1].parent_i = i;
                cellDetails[i][j + 1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }

                // If the successor is already on the closed
                // list or if it is blocked, then ignore it.
                // Else do the following
            else if (closedList[i][j + 1] == false
                     && isUnBlocked(grid, i, j + 1)
                        == true) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i, j + 1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i][j + 1].f == FLT_MAX
                    || cellDetails[i][j + 1].f > fNew) {
                    openList.insert(make_pair(
                            fNew, make_pair(i, j + 1)));

                    // Update the details of this cell
                    cellDetails[i][j + 1].f = fNew;
                    cellDetails[i][j + 1].g = gNew;
                    cellDetails[i][j + 1].h = hNew;
                    cellDetails[i][j + 1].parent_i = i;
                    cellDetails[i][j + 1].parent_j = j;
                }
            }
        }

        //----------- 4th Successor (West) ------------

        // Only process this cell if this is a valid one
        if (isValid(i, j - 1) == true) {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i, j - 1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i][j - 1].parent_i = i;
                cellDetails[i][j - 1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }

                // If the successor is already on the closed
                // list or if it is blocked, then ignore it.
                // Else do the following
            else if (closedList[i][j - 1] == false
                     && isUnBlocked(grid, i, j - 1)
                        == true) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i, j - 1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i][j - 1].f == FLT_MAX
                    || cellDetails[i][j - 1].f > fNew) {
                    openList.insert(make_pair(
                            fNew, make_pair(i, j - 1)));

                    // Update the details of this cell
                    cellDetails[i][j - 1].f = fNew;
                    cellDetails[i][j - 1].g = gNew;
                    cellDetails[i][j - 1].h = hNew;
                    cellDetails[i][j - 1].parent_i = i;
                    cellDetails[i][j - 1].parent_j = j;
                }
            }
        }

        //----------- 5th Successor (North-East)
        //------------

        // Only process this cell if this is a valid one
        if (isValid(i - 1, j + 1) == true) {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i - 1, j + 1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i - 1][j + 1].parent_i = i;
                cellDetails[i - 1][j + 1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }

                // If the successor is already on the closed
                // list or if it is blocked, then ignore it.
                // Else do the following
            else if (closedList[i - 1][j + 1] == false
                     && isUnBlocked(grid, i - 1, j + 1)
                        == true) {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i - 1, j + 1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i - 1][j + 1].f == FLT_MAX
                    || cellDetails[i - 1][j + 1].f > fNew) {
                    openList.insert(make_pair(
                            fNew, make_pair(i - 1, j + 1)));

                    // Update the details of this cell
                    cellDetails[i - 1][j + 1].f = fNew;
                    cellDetails[i - 1][j + 1].g = gNew;
                    cellDetails[i - 1][j + 1].h = hNew;
                    cellDetails[i - 1][j + 1].parent_i = i;
                    cellDetails[i - 1][j + 1].parent_j = j;
                }
            }
        }

        //----------- 6th Successor (North-West)
        //------------

        // Only process this cell if this is a valid one
        if (isValid(i - 1, j - 1) == true) {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i - 1, j - 1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i - 1][j - 1].parent_i = i;
                cellDetails[i - 1][j - 1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }

                // If the successor is already on the closed
                // list or if it is blocked, then ignore it.
                // Else do the following
            else if (closedList[i - 1][j - 1] == false
                     && isUnBlocked(grid, i - 1, j - 1)
                        == true) {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i - 1, j - 1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i - 1][j - 1].f == FLT_MAX
                    || cellDetails[i - 1][j - 1].f > fNew) {
                    openList.insert(make_pair(
                            fNew, make_pair(i - 1, j - 1)));
                    // Update the details of this cell
                    cellDetails[i - 1][j - 1].f = fNew;
                    cellDetails[i - 1][j - 1].g = gNew;
                    cellDetails[i - 1][j - 1].h = hNew;
                    cellDetails[i - 1][j - 1].parent_i = i;
                    cellDetails[i - 1][j - 1].parent_j = j;
                }
            }
        }

        //----------- 7th Successor (South-East)
        //------------

        // Only process this cell if this is a valid one
        if (isValid(i + 1, j + 1) == true) {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i + 1, j + 1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i + 1][j + 1].parent_i = i;
                cellDetails[i + 1][j + 1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }

                // If the successor is already on the closed
                // list or if it is blocked, then ignore it.
                // Else do the following
            else if (closedList[i + 1][j + 1] == false
                     && isUnBlocked(grid, i + 1, j + 1)
                        == true) {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i + 1, j + 1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i + 1][j + 1].f == FLT_MAX
                    || cellDetails[i + 1][j + 1].f > fNew) {
                    openList.insert(make_pair(
                            fNew, make_pair(i + 1, j + 1)));

                    // Update the details of this cell
                    cellDetails[i + 1][j + 1].f = fNew;
                    cellDetails[i + 1][j + 1].g = gNew;
                    cellDetails[i + 1][j + 1].h = hNew;
                    cellDetails[i + 1][j + 1].parent_i = i;
                    cellDetails[i + 1][j + 1].parent_j = j;
                }
            }
        }

        //----------- 8th Successor (South-West)
        //------------

        // Only process this cell if this is a valid one
        if (isValid(i + 1, j - 1) == true) {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i + 1, j - 1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i + 1][j - 1].parent_i = i;
                cellDetails[i + 1][j - 1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }

                // If the successor is already on the closed
                // list or if it is blocked, then ignore it.
                // Else do the following
            else if (closedList[i + 1][j - 1] == false
                     && isUnBlocked(grid, i + 1, j - 1)
                        == true) {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i + 1, j - 1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i + 1][j - 1].f == FLT_MAX
                    || cellDetails[i + 1][j - 1].f > fNew) {
                    openList.insert(make_pair(
                            fNew, make_pair(i + 1, j - 1)));

                    // Update the details of this cell
                    cellDetails[i + 1][j - 1].f = fNew;
                    cellDetails[i + 1][j - 1].g = gNew;
                    cellDetails[i + 1][j - 1].h = hNew;
                    cellDetails[i + 1][j - 1].parent_i = i;
                    cellDetails[i + 1][j - 1].parent_j = j;
                }
            }
        }
    }

    // When the destination cell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destiantion cell. This may happen when the
    // there is no way to destination cell (due to
    // blockages)
    if (foundDest == false)
        printf("Failed to find the Destination Cell\n");

    return;
}

double distance(double x,double y,double targetx,double targety){
    return (sqrt((targetx-x)*(targetx-x) + (targety-y)*(targety-y)));
}

bool isAchieved(double x,double y,double targetx,double targety){
    double treshold = 0.5;
    return (distance(x,y,targetx,targety)<treshold);
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

    std::fstream myfile;
    string file_Path = strcat(buffer,file_path);
    std::cout << strcat(buffer,file_path) << std::endl;
    myfile.open(file_Path, std::ios_base::in);
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

    myfile >> N;
    double myObstacles[N][4];

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
    for(int i=0; i< N; i++){
        for(int j=0; j<4;j++){
            myfile >> myObstacles[i][j];
        }
    }

    int myMatrix[50][50];
    for(int i=0;i<50;i++) for(int j=0; j<50; j++) myMatrix[i][j] = 1;

    double theDistance;
    for(int k=0; k<N ;k++){
        double myBorder = myObstacles[k][2]+1+0.5;

        for(int i=0; i<50; i++){
            for(int j=0; j<50; j++){
                theDistance = sqrt(pow(myObstacles[k][0] - (double)i,2) + pow(myObstacles[k][1] - (double)j,2));
                if(theDistance <= myBorder){
                    myMatrix[i][j] = 0;
                }
            }
        }
    }


    //print the map
    for(int i=0; i<50; i++) {
        for (int j = 0; j < 50; j++) {
            std::cout << myMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }





    bool start_search = true;
    bool got_cargo = false;
    int mission = 0;
    double stop_x,stop_y;
    int path_iterator = 0;
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
        nav_msgs::Odometry vehicle_pose;
        vehicle_pose.pose.pose.position.x = x;
        vehicle_pose.pose.pose.position.y = y;
        vehicle_pose.pose.pose.orientation = getModelState.response.pose.orientation;
        vehicle_pose.twist.twist = getModelState.response.twist;


        if (start_search){


            stop_x = path[mission+1][0]  ;stop_y = path[mission+1][1];

            // Source is the left-most bottom-most corner
            Pair src = make_pair(x, y);

            // Destination is the left-most top-most corner
            Pair dest = make_pair(stop_x, stop_y);

            aStarSearch(myMatrix, src, dest);
            start_search = false;
            for(int i=0; i < Path.size();i++) cout <<Path[i].first<<Path[i].second<<endl;

        }
        double target_x = Path[Path.size()-1-path_iterator].first;
        double target_y = Path[Path.size()-1-path_iterator].second;


        tgr_msgs::Line current_line;
        current_line.point_begin.x = x; current_line.point_begin.y = y;
        current_line.point_end.x = target_x; current_line.point_end.y = target_y;
        if(isAchieved(x,y,target_x,target_y)) path_iterator++;

        geometry_msgs::Twist vel_output;

        if(!isAchieved(x,y,stop_x,stop_y)){
            cmd_vel_pub.publish(line_follower(current_line,vehicle_pose));
        }

        else{
            vel_output.angular.x = 0;
            vel_output.angular.y = 0;
            vel_output.angular.z = 0;
            vel_output.linear.x = 0;
            vel_output.linear.y = 0;
            vel_output.linear.z = 0;
            cmd_vel_pub.publish(vel_output);
            mission++;
            path_iterator=0;
            start_search = true;
            Path.clear();
        }


        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}