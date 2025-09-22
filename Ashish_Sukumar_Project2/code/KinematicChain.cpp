/*********************************************************************
 * Software License Agreement (BSD License)
 *********************************************************************/
#include "KinematicChain.h"
#include <ompl/geometric/SimpleSetup.h>
#include "RTP.h"
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage:\n" << argv[0] << " <num_links> [once]\n";
        return 0;
    }

    unsigned int numLinks = std::stoul(argv[1]);

    // pick the horn environment (more interesting than empty)
    Environment env = createHornEnvironment(numLinks, std::log((double)numLinks)/(double)numLinks);

    auto chain(std::make_shared<KinematicChainSpace>(numLinks, 1.0/(double)numLinks, &env));
    ompl::geometric::SimpleSetup ss(chain);
    ss.setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(ss.getSpaceInformation()));

    ompl::base::ScopedState<> start(chain), goal(chain);
    std::vector<double> startVec(numLinks, boost::math::constants::pi<double>()/(double)numLinks);
    std::vector<double> goalVec (numLinks, 0.0);
    startVec[0] = 0.0;
    goalVec[0]  = boost::math::constants::pi<double>() - 0.001;

    chain->setup();
    chain->copyFromReals(start.get(), startVec);
    chain->copyFromReals(goal.get() , goalVec);
    ss.setStartAndGoalStates(start, goal);

    // If a 2nd arg is given, just solve once with STRIDE and dump a path .dat
    if (argc > 2)
    {
        ss.setPlanner(std::make_shared<ompl::geometric::STRIDE>(ss.getSpaceInformation()));
        ss.setup();
        ss.solve(3600.0);
        ss.simplifySolution();
        std::ofstream pathfile(boost::str(boost::format("kinematic_path_%i.dat") % numLinks));
        ss.getSolutionPath().printAsMatrix(pathfile);
        return 0;
    }

    // Otherwise run a full benchmark with multiple planners
    double runtime_limit = 60.0, memory_limit = 1024.0;
    int    run_count     = 20;
    ompl::tools::Benchmark::Request req(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark bench(ss, "KinematicChain");
    bench.addExperimentParameter("num_links", "INTEGER", std::to_string(numLinks));

    bench.addPlanner(std::make_shared<ompl::geometric::STRIDE>(ss.getSpaceInformation()));
    bench.addPlanner(std::make_shared<ompl::geometric::EST>(ss.getSpaceInformation()));
    bench.addPlanner(std::make_shared<ompl::geometric::KPIECE1>(ss.getSpaceInformation()));
    bench.addPlanner(std::make_shared<ompl::geometric::RRT>(ss.getSpaceInformation()));
    bench.addPlanner(std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation()));
    bench.addPlanner(std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation(), false));

    bench.benchmark(req);
    bench.saveResultsToFile(boost::str(boost::format("kinematic_%i.log") % numLinks).c_str());
    return 0;
}
