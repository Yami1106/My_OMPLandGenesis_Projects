#include "CollisionChecking.h"
#include "RTP.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>

// helps to make the reference to ompl::base and ompl::geometric easier by using namespace shortcut
namespace ob = ompl::base;
namespace og = ompl::geometric;

void planPoint(const std::vector<Rectangle> & obstacles )
{
    // defining the bounds of the workspace 
    const double lbound = 0.0, hbound = 10.0;

    //creating a 2D space for (x,y) notation
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

    // the bounds are specified to that space as well
    ob::RealVectorBounds bounds(2);
    bounds.setLow(lbound); bounds.setHigh(hbound);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // simplesetup is called to simplify the state space initialization process
    og::SimpleSetup ss(space);


    // check if the state is within the specified bounds if not reject it 
    ss.setStateValidityChecker([&](const ob::State *state) -> bool {
        const auto *st = state->as<ob::RealVectorStateSpace::StateType>();
        double x = st->values[0], y = st->values[1];
        if (x < lbound || x > hbound || y < lbound || y > hbound) return false;

        // check if point collides with and obstacle, if it does reject it 
        return isValidPoint(x, y, obstacles);
    });

    // create and assign start and goal states 
    ob::ScopedState<> start(space), goal(space);
    start[0] = 1.0; start[1] = 1.0;
    goal[0]  = 9.0; goal[1]  = 9.0;

    // assigning a tolerance of 0.3 so that the planner can hit the goal 
    ss.setStartAndGoalStates(start, goal,0.3);

    //implement the RTP planner, assign range and goalbias values
    auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation(), false);
    planner->setRange(1.2);
    planner->setGoalBias(0.2); 
    ss.setPlanner(planner);

    ob::PlannerStatus solved = ss.solve(10.0); // run the planner for 10s maximum

    // if a solution is found by the planner, write it to a file
    if (ss.haveSolutionPath())
    {
        std::ofstream pathOut("path1.txt");
        // config space header needed for visualizing script
        pathOut << "R2\n";  
        auto &path = ss.getSolutionPath();
        // wirte all the x,y values to path file 
        for (std::size_t i = 0; i < path.getStateCount(); ++i)
        {
            const auto *st = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
            pathOut << st->values[0] << " " << st->values[1] << "\n";
        }
        std::cout << "Point robot: solution written to path1.txt\n";
    }
    // print no solution if no path is found 
    else
        std::cout << "Point robot: no solution found.\n";
    // TODO: Plan for a point robot in the plane. 
}

void planBox(const std::vector<Rectangle> &  obstacles)
{
    // assign the workspace bounds and the size of the robot
    const double lbound = 0.0, hbound = 10.0;
    const double side   = 0.3;  

    // create SE2 space (x,y,yaw(theta))
    ob::StateSpacePtr space(new ob::SE2StateSpace());
    ob::RealVectorBounds bounds(2);
    bounds.setLow(lbound); bounds.setHigh(hbound);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    og::SimpleSetup ss(space);

    ss.setStateValidityChecker([&](const ob::State *state) -> bool {
        const auto *s = state->as<ob::SE2StateSpace::StateType>();
        return isValidSquare(s->getX(), s->getY(), s->getYaw(), side, obstacles, hbound, lbound);
    });

    ob::ScopedState<ob::SE2StateSpace> start(space), goal(space);

    // the start and goal configuration for the robot
    start->setX(1.0); start->setY(1.0); start->setYaw(0.0);
    goal->setX( 5.0); goal->setY( 8.0); goal->setYaw(0.0);

    ss.setStartAndGoalStates(start, goal,0.1);

    // the RTP planner is called in the box mode 
    auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation(), true);
    planner->setRange(1.0);  
    planner->setGoalBias(0.25);
    ss.setPlanner(planner);

    ob::PlannerStatus solved = ss.solve(15.0); // time limit of 15seconds max

    if (ss.haveSolutionPath())
    {
        std::ofstream pathOut("path2.txt"); 
        pathOut << "SE2 " << side << "\n"; 
        auto &path = ss.getSolutionPath();
        for (std::size_t i = 0; i < path.getStateCount(); ++i)
        {
            const auto *st = path.getState(i)->as<ob::SE2StateSpace::StateType>();
            pathOut << st->getX() << " " << st->getY() << " " << st->getYaw() << "\n";
        }
        std::cout << "Box robot: solution written to path2.txt\n";
    }
    else
        std::cout << "Box robot: no solution found.\n";
    // TODO: Plan for a square_box that rotates and translates in the plane.
}

void makeEnvironment1(std::vector<Rectangle> &  obstacles )
{
    // ensure empty obstacle list 
    obstacles.clear();
    obstacles.push_back(Rectangle{0.0, 2.0, 4.0, 0.6});   
    obstacles.push_back(Rectangle{6.0, 2.0, 4.0, 0.6});  
    obstacles.push_back(Rectangle{0.0, 4.0, 7.0, 0.6});  
    obstacles.push_back(Rectangle{8.5, 4.0, 1.5, 0.6});  
    obstacles.push_back(Rectangle{0.0, 6.0, 1.5, 0.6});   
    obstacles.push_back(Rectangle{3.0, 6.0, 7.0, 0.6}); 
    obstacles.push_back(Rectangle{0.0, 8.0, 7.5, 0.6});   
    obstacles.push_back(Rectangle{9.0, 8.0, 1.0, 0.6});  
    obstacles.push_back(Rectangle{5.0, 0.5, 0.7, 1.5});

    // Write to obstacles1.txt file
    std::ofstream obsOut("obstacles1.txt");
    for (auto &r : obstacles)
        obsOut << r.x << " " << r.y << " " << r.width << " " << r.height << "\n";

    // TODO: Fill in the vector of rectangles with your second environment.
}

void makeEnvironment2(std::vector<Rectangle> &  obstacles )
{
    obstacles.clear();
    obstacles.push_back(Rectangle{0.0, 0.0, 6.5, 0.7}); 
    obstacles.push_back(Rectangle{0.0, 0.0, 0.7, 4.0}); 
    obstacles.push_back(Rectangle{5.8, 0.0, 0.7, 3.0}); 
    obstacles.push_back(Rectangle{3.5, 9.3, 6.5, 0.7}); 
    obstacles.push_back(Rectangle{9.3, 6.5, 0.7, 3.5}); 
    obstacles.push_back(Rectangle{8.6, 6.5, 0.7, 2.3}); 
    obstacles.push_back(Rectangle{3.2, 3.0, 3.6, 3.8});
    obstacles.push_back(Rectangle{6.8, 2.0, 0.6, 1.6});
    obstacles.push_back(Rectangle{7.6, 4.6, 0.6, 1.6});
    obstacles.push_back(Rectangle{1.2, 2.6, 0.6, 1.6});

    // Write to obstacles2.txt file
    std::ofstream obsOut("obstacles2.txt");
    for (auto &r : obstacles)
        obsOut << r.x << " " << r.y << " " << r.width << " " << r.height << "\n";
    // TODO: Fill in the vector of rectangles with your second environment.

}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) TODO" << std::endl;
        std::cout << " (2) TODO" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}
