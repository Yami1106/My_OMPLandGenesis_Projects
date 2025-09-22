///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Ashish Sukumar
// Date: 09/18/2025
//////////////////////////////////////

#include "CollisionChecking.h"

//header file included for using mathematical functions
#include<cmath>

bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles)
{
    //Fillout 
    for(const auto& obs:obstacles){
        // define the lower and the upper bound of the rectangular obstacle in each iteration 
        double x_lb = obs.x;
        double x_ub = obs.x + obs.width;
        double y_lb = obs.y;
        double y_ub = obs.y + obs.height;

        // if the point lies anywhere between the bounds of the rectangle it means it collides with the object and hence returns a false value  
        if (x >= x_lb && x<=x_ub && y>=y_lb && y<= y_ub){
            return false;
        }
    }

    return true;
}

bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles,double hbound, double lbound)
{
    //Fillout 

    double half_side = sideLength/2;

    // creating a list of pairs of the corners of the robot with center at the origin, this is the local coordinate frame which will be used for rotation 
    std::vector<std::pair<double,double>> loc = {
        {half_side,half_side},
        {-half_side,half_side},
        {-half_side,-half_side},
        {half_side,-half_side}
    };

    // rotate the robot and translate it with the robots actual center i.e. x,y to get the actual corners with respect to the global coordinate system
    
    std::vector<std::pair<double,double>> glob_corners;

    for (auto& i : loc){
        double x_r = i.first * std::cos(theta) + i.second * std::sin(theta);
        double y_r = i.first * std::sin(theta) - i.second * std::cos(theta);


        //this give the rotated coordinate of the robot now to we just have to translate it to get the actual coordinates of the robot.
        double x_f = x+x_r;
        double y_f = y+y_r; 

        glob_corners.push_back({x_f,y_f});
    }

    // if the robot rotates it may not align with the axis therefore we use the Axis-aligned bounding box which will contain the positions of the robot within it even when it is rotated

    double x_min = glob_corners[0].first , x_max = glob_corners[0].first;
    double y_min = glob_corners[0].second , y_max = glob_corners[0].second;

    for(auto j : glob_corners){
        x_min = std::min(x_min,j.first);
        x_max = std::max(x_max,j.first);
        y_min = std::min(y_min,j.second);
        y_max = std::max(y_max,j.second);
    }

    // condition to check if the robot is within the bounds of the environment 

    if(x_min < lbound || x_max > hbound || y_min < lbound || y_max > hbound){
        return false;
    }

    // now to check for the collision 
    for(const auto& obs: obstacles){
        // to calculate the boundaries of the obstacle
        double x_lb = obs.x;
        double x_ub = obs.x + obs.width;
        double y_lb = obs.y;
        double y_ub = obs.y + obs.height;

        // condition to check if the bounding box collides with the obstacle
        if (x_max >= x_lb && x_min <= x_ub && y_max >= y_lb && y_min <= y_ub){
            return false;
        }

    }
    return true;
}

