/* Author: Ali Golestaneh and Constantinos Chamzas */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <fstream>
#include <iostream>

#include "DiskSampler.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

void draw_samples(int status) {
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    // set the bounds for the R^2
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);

    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    std::ofstream fgraph;
    switch (status) {
        // Use naive sampling
        case 0:
            ss.getSpaceInformation()->setValidStateSamplerAllocator(
                allocDiskSamplerNaive);
            fgraph = std::ofstream("naive_samples.graphml");
            break;
        // Use correct sampling
        case 1:
            ss.getSpaceInformation()->setValidStateSamplerAllocator(
                allocDiskSamplerCorrect);
            fgraph = std::ofstream("correct_samples.graphml");
            break;
    }

    // set state validity checking for this space
    ss.setStateValidityChecker(
        [](const ob::State *state) { return isStateValid(state); });

    // set the planner to PRM
    auto planner(std::make_shared<og::PRM>(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    ss.setup();

    // Grow the roamap for 1000 iterations (This could result in fewwer than
    // 1000 vertices)
    planner->as<og::PRM>()->growRoadmap(
        ob::IterationTerminationCondition(5000));
    
    ob::PlannerData samples(ss.getSpaceInformation());
    planner->getPlannerData(samples);
    std::cout << "Number of vertices in the final roadmap: "
              << samples.numVertices() << std::endl;

    samples.printGraphML(fgraph);
    fgraph.close();
}

int main(int /*argc*/, char ** /*argv*/) {
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    draw_samples(0);
    draw_samples(1);
}
