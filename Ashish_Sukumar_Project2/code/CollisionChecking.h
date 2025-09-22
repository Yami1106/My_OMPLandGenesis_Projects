#ifndef COLLISION_CHECKING_H_
#define COLLISION_CHECKING_H_

#include <vector>
#include <cmath>
#include <iostream>

struct Rectangle
{
    // Coordinate of the lower left corner of the rectangle
    double x, y;
    // The width (x-axis extent) of the rectangle
    double width;
    // The height (y-axis extent) of the rectangle
    double height;
};

// Definition of our robot.
struct Robot
{
    // Type = {s,p}.  square, point robot
    char type;
    // The location of the robot in the environment
    double x, y;
    // The orientation of the square robot.  Undefined for point or circle robot
    double theta;
    // The length of a side of the square robot or the radius of the circle robot
    // Undefined for the point robot.
    double length;
};

// Intersect the point (x,y) with the set of rectangles. If the point lies
// outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles);

// Intersect a square with center at (x,y), orientation theta, and the given
// side length with the set of rectangles. If the square lies outside of all
// obstacles, return true. The hbound, and lbound are the dimentions of the environment. 
// You have to ensure that the square does not go outside the bounds as well. 
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles, double hbound , double lbound);
#endif
