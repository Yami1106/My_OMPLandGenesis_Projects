/* Author: Ali Golestaneh and Constantinos Chamzas,Ashish Sukumar */


#include <cmath>
#include "DiskSampler.h"
using namespace std;

bool isStateValid(const ob::State *state) {

    // cast the abstract state type to the type we expect
    const auto *r2state = state->as<ob::RealVectorStateSpace::StateType>();
    double x = r2state->values[0];
    double y = r2state->values[1];
    // A square obstacle with and edge of size 2*sqrt(2) is located in location [-3,-2,] and rotated pi/4 degrees around its center.
    // Fill out this function that returns False when the state is inside/or the obstacle and True otherwise// 
    // ******* START OF YOUR CODE HERE *******//
    
    double sx = -3;
    double sy = -2;
    
    double theta = M_PI/4;
    double s = 2*sqrt(2);
    
    double x_t,y_t;
    
    x_t = x-sx;
    y_t = y-sy;
    
    double xr,yr;
    
    xr = x_t*cos(theta) + y_t*sin(theta);
    yr = x_t*sin(theta) - y_t*cos(theta);
    double half_s = s/2;
    
    if (abs(xr) <= half_s && abs(yr)<= half_s){
    return false;
    }
    
    return true;

    // ******* END OF YOUR CODE HERE *******//
}


bool DiskSampler::sampleNaive(ob::State *state) 
{
    // ******* START OF YOUR CODE HERE *******//
    const auto *r2state = state->as<ob::RealVectorStateSpace::StateType>();
    double R=10;
    double r = rng_.uniformReal(0,R);
    double theta = rng_.uniformReal(0,2*M_PI);
    
    
    double car_x = r*cos(theta);
    double car_y = r*sin(theta);
    
    r2state->values[0] = car_x;
    r2state->values[1] = car_y;
    // ******* END OF YOUR CODE HERE *******//
    
    //The valid state sampler must return false if the state is in-collision
    return isStateValid(state);
}

bool DiskSampler::sampleCorrect(ob::State *state)
{ 
    // ******* START OF YOUR CODE HERE *******//
    const auto *r2state = state->as<ob::RealVectorStateSpace::StateType>();
    double R=10;
    double u= rng_.uniformReal(0,1);
    double r = R*sqrt(u);
    double theta = rng_.uniformReal(0,2*M_PI);
    
    double car_x = r*cos(theta);
    double car_y = r*sin(theta);
    
    r2state->values[0] = car_x;
    r2state->values[1] = car_y;
    // ******* END OF YOUR CODE HERE *******//
    //The valid state sampler must return false if the state is in-collision
    return isStateValid(state);
}
