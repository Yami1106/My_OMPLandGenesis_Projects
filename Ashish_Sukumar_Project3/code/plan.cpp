#include "KinematicChain.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h> 
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>


#include <array>
#include <utility>
#include <limits>
#include <algorithm>
#include <cmath>
#include <fstream>   


namespace ob = ompl::base;
namespace og = ompl::geometric;


// creating a validity checker for the chainbox robot, it inherits the KinematicChain Vlidity checker so that we can use some of its functions
class ChainBoxValidityChecker : public KinematicChainValidityChecker {
    public:
        ChainBoxValidityChecker(const ob::SpaceInformationPtr &si, const Environment &env) : KinematicChainValidityChecker(si), env_(env){}
        const Environment& obstacles() const { return env_; }

        // Public wrapper so helpers can reuse the robust intersection predicate
        bool segmentsIntersect(const Segment& s0, const Segment& s1) const {
            return this->intersectionTest(s0, s1);
        }

        bool isValid(const ob::State* state) const override{
            // the vector will store x,y,theta,q0,q1,q2,q3 values from ompl state
            std::vector<double> r;

            si_->getStateSpace()->copyToReals(r,state);
            if (r.size()<7) return false;

            // cx and cy are the center of the box robot's center w.r.t. world coordinates
            const double cx = r[0] , cy = r[1] , theta = r[2];

            //check if state is within the bounds
            if(cx<-5.0 || cx>5.0 || cy<-5.0 ||cy>5.0) return false;

            // lambda funciton to check if the robot to lies in bounds
            auto inBounds = [&](double x, double y)->bool {
                return (x >= -5.0 && x <= 5.0 && y >= -5.0 && y <= 5.0);
            };

            // a lambda function is used to do the forward kinematics for the robot and convert it from local coordinate system to a global one
            auto rota = [&](double dx, double dy)-> std::pair<double,double>{
                double c = std::cos(theta) , s = std::sin(theta);
                return std::pair<double,double>(cx + c*dx - s*dy,cy + s*dx + c*dy); // rotation + translation
            };

            // a lambda function is used to call the intersectionTest function from the KinematicChain.h file to check if inputted segments intersect
            auto segmIsect = [&](const Segment& a,const Segment& b)-> bool{
                return this->intersectionTest(a,b);
            };

            // the length of square is 1 so the corners are calculated according to the local frame 
            const double h = 0.5;

            // this will convert the local frame to a global one and accounts for both translation+rotation
            std::pair<double,double> p0 = rota(-h,-h);
            std::pair<double,double> p1 = rota( h,-h);
            std::pair<double,double> p2 = rota( h, h);
            std::pair<double,double> p3 = rota(-h, h);

            double x0 = p0.first, y0 = p0.second;
            double x1 = p1.first, y1 = p1.second;
            double x2 = p2.first, y2 = p2.second;
            double x3 = p3.first, y3 = p3.second;

            // checks if all four base corners are inside bounds
            if (!inBounds(x0,y0) || !inBounds(x1,y1) || !inBounds(x2,y2) || !inBounds(x3,y3))
                return false;

            // the corners are used to create edges of the robot
            std::array<Segment,4> boxEdges = {
                Segment(x0,y0,x1,y1),
                Segment(x1,y1,x2,y2),
                Segment(x2,y2,x3,y3),
                Segment(x3,y3,x0,y0)
            };

            std::array<double,4> q{r[3],r[4],r[5],r[6]};
            std::vector<Segment> chain;
            chain.reserve(4);
            // the angle for the first link will be angle the box robot is rotated by + the angle the link itself has shifted by
            double angle = theta + q[0];
            // px and py store the current joint positions
            double px = cx , py = cy;
            for (int i=0;i<4;++i) {
                // the angle for each joint will then be summation of each of the previous angles
                if (i>0) angle += q[i];
                // cos contributed to the x direction and sin contributed to the y direction generally so the next joint position will the current joint position + the vector in that direction
                // since link length is 1 nx = px+ cos(ang) if the link length was l it would be nx = px+l*cos(ang), similar for ny
                double nx = px + std::cos(angle);
                double ny = py + std::sin(angle);

                // --- NEW: enforce each joint endpoint stays inside bounds ---
                // (Square is convex, so if endpoints are inside, the straight link segment is inside too.)
                if (!inBounds(nx,ny)) return false;

                chain.emplace_back(px, py, nx, ny);
                px = nx; py = ny;
            }

            // check for self intersection of non adjacent joints because adjacent joints are connected by a joint so they touch and we want to ignore that 
            for(int i=0;i<chain.size();++i)
                for(int j=i+2;j<chain.size();++j)
                    if(segmIsect(chain[i],chain[j])) return false;

            // link 0 is allowed to intersect with bot but others are not so the loop starts from 1 
            for (size_t i=1;i<chain.size();++i)
                for (const auto& be : boxEdges)
                    if (segmIsect(chain[i], be)) return false;
            
            // check if edges of the bot or chain links collide with the obstacles in the environment
            for (const auto& e : env_) {
                // this checks if any box edge intersects with e and if it does returns false
                for (const auto& be : boxEdges)
                    if (segmIsect(be, e)) return false;
                //each link is tested for collision with environment obstacles
                for (const auto& l : chain)
                    if (segmIsect(l, e)) return false;
        }
        return true;
        }
    
    private:
        const Environment env_;
};


void makeScenario1(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{

    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -3;
    start[1] = -3;
    goal[0] = 2 ; 
    goal[1] = 2 ; 
    goal[2] = 0; 
    goal[4] = -1.57079;

    //Obstacle 1
    env.emplace_back(2, -1, 2.8, -1);
    env.emplace_back(2.8, -1, 2.8, 0.5);
    env.emplace_back(2.8, 0.5, 2, 0.5);
    env.emplace_back(2, 0.5, 2, -1);

    //Obstacle 2
    env.emplace_back(3.2, -1, 4, -1);
    env.emplace_back(4, -1, 4, 0.5);
    env.emplace_back(4, 0.5, 3.2, 0.5);
    env.emplace_back(3.2, 0.5, 3.2, -1);

}

void makeScenario2(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{
    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -4;
    start[1] = -4;
    start[2] = 0;
    goal[0] = 3; 
    goal[1] = 3; 
    goal[2] = 0; 
    goal[4] = -1.57079;

    //Obstacle 1
    env.emplace_back(-1, -1, 1, -1);
    env.emplace_back(1, -1, 1, 1);
    env.emplace_back(1, 1, -1, 1);
    env.emplace_back(-1, 1, -1, -1);
}


void planScenario1(ompl::geometric::SimpleSetup &ss)
{
     // TODO: Plan for chain_box in the plane, and store the path in narrow.txt. 
    // implementing the RRT connect algorithm
    auto planner = std::make_shared<og::RRTConnect>(ss.getSpaceInformation());
    // set range is used to give the distance of each step short step makes sure t goes through tight gaps as well
    // this value can be more optimized by trial and error 
    planner->setRange(0.9728);           
    ss.setPlanner(planner);

     
    ss.setup();
    // run the planner for 20s return true if a solution is found
    if (ss.solve(20.0))             
    {
        //store the found solution
        og::PathGeometric &pg = ss.getSolutionPath();
        // pathsimplifier has functions that tries to simplify geometric path
        og::PathSimplifier ps(ss.getSpaceInformation());
        //simplifies the apth and creates 200 states
        ps.simplifyMax(pg);
        pg.interpolate(200);

        // write the final path to narrow.txt file 
        std::ofstream ofs("narrow.txt");
        for (std::size_t i = 0; i < pg.getStateCount(); ++i) {
            std::vector<double> r;
            ss.getStateSpace()->copyToReals(r, pg.getState(i));
            for (std::size_t k = 0; k < r.size(); ++k)
                ofs << r[k] << (k + 1 == r.size() ? '\n' : ' ');
        }
        ofs.close();
    }
    else {
        std::cerr << "[Scenario1] No solution within time.\n";
    } 
}

void benchScenario1(ompl::geometric::SimpleSetup &ss)
{
    //TODO: Benchmark PRM with uniform, bridge, gaussian, and obstacle-based Sampling. Do 20 trials with 20 seconds each 
    double runtime_limit = 20, memory_limit = 1024;
    int run_count = 20;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "ChainBox_Narrow");

    // create a lambda function which creates a PRM names it and adds it to the benchmark
    auto addPRM = [&](const std::string& name, const ob::ValidStateSamplerAllocator& alloc) {
        ss.getSpaceInformation()->setValidStateSamplerAllocator(alloc);
        auto prm = std::make_shared<og::PRM>(ss.getSpaceInformation());
        prm->setName(name);
        b.addPlanner(prm);
    };

    // uses the default valid sampler uniformly samples, then rejects invalid states
    auto makeUniform  = [](const ob::SpaceInformation *si){ return si->allocValidStateSampler(); };
    // sample using gaussian with a standard deviation of 0.5
    auto makeGaussian = [](const ob::SpaceInformation *si){
        auto s = std::make_shared<ob::GaussianValidStateSampler>(si);
        s->setStdDev(0.5);
        return s;
    };
    // biases samples near objects , helps with narrow passages
    auto makeObstacle = [](const ob::SpaceInformation *si){
        return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
    };
    // sample 2 invalid states take their midpoint, if valid it lies in a narrow gap 
    auto makeBridge   = [](const ob::SpaceInformation *si){
        return std::make_shared<ob::BridgeTestValidStateSampler>(si);
    };

    addPRM("PRM_Uniform",  makeUniform);
    addPRM("PRM_Gaussian", makeGaussian);
    addPRM("PRM_Obstacle", makeObstacle);
    addPRM("PRM_Bridge",   makeBridge);

    b.benchmark(request);
    b.saveResultsToFile("narrow_benchmark.log");
}

// function to return the squared Euclidean distance from point P = (px,py) to a segment s.
static double pointSegDist2(double px, double py, const Segment& s) {
    const double ax = s.x0, ay = s.y0, bx = s.x1, by = s.y1;
    const double vx = bx - ax, vy = by - ay;
    const double wx = px - ax, wy = py - ay;
    const double vv = vx*vx + vy*vy;
    // how far along AB is the closest point
    double t = vv > 0.0 ? (wx*vx + wy*vy) / vv : 0.0;
    if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
    //Coordinates of the closest point C on the segment.
    const double cx = ax + t*vx, cy = ay + t*vy;
    // Return squared distance
    const double dx = px - cx, dy = py - cy;
    return dx*dx + dy*dy;
}

// distance between two segments
static double segSegDist2(const Segment& a, const Segment& b,
                          const ChainBoxValidityChecker* vc) {
    //If they intersect, distance is exactly zero
    if (vc && vc->segmentsIntersect(a, b)) return 0.0;
    double d2 = std::numeric_limits<double>::infinity();
    d2 = std::min(d2, pointSegDist2(a.x0, a.y0, b));
    d2 = std::min(d2, pointSegDist2(a.x1, a.y1, b));
    d2 = std::min(d2, pointSegDist2(b.x0, b.y0, a));
    d2 = std::min(d2, pointSegDist2(b.x1, b.y1, a));
    return d2;
}

// build robot geometry from a state vector r = [x,y,theta,q0,q1,q2,q3]
static void buildRobotSegments(const std::vector<double>& r,
                               std::array<Segment,4>& boxEdges,
                               std::vector<Segment>& chain) {

    // Box center (cx,cy) and its rotation th
    const double cx = r[0], cy = r[1], th = r[2];

    // forward kinematics
    auto rota = [&](double dx, double dy)->std::pair<double,double>{
        double c = std::cos(th), s = std::sin(th);
        return std::make_pair(cx + c*dx - s*dy, cy + s*dx + c*dy);
    };
    // robot length = 1, if center is at origin corners are 0.5 on either side of the origin 
    const double h = 0.5; 
    std::pair<double,double> p0 = rota(-h,-h);
    std::pair<double,double> p1 = rota( h,-h);
    std::pair<double,double> p2 = rota( h, h);
    std::pair<double,double> p3 = rota(-h, h);

    // create the edges of the square
    boxEdges = {
        Segment(p0.first,p0.second,p1.first,p1.second),
        Segment(p1.first,p1.second,p2.first,p2.second),
        Segment(p2.first,p2.second,p3.first,p3.second),
        Segment(p3.first,p3.second,p0.first,p0.second)
    };

    std::array<double,4> q{ r[3], r[4], r[5], r[6] };
    chain.clear(); chain.reserve(4);
    double angle = th + q[0], px = cx, py = cy;
    for (int i=0;i<4;++i) {
        if (i>0) angle += q[i];
        double nx = px + std::cos(angle), ny = py + std::sin(angle);
        chain.emplace_back(px, py, nx, ny);
        px = nx; py = ny;
    }
}

// true workspace clearance finds minimum distance of robot to any obstacle edge, the objective is to find the distance 2 segments the robot and the obstacle
static double trueWorkspaceClearance(const std::vector<double>& r,
                                     const Environment& env,
                                     const ChainBoxValidityChecker* vc) {
    // initialize because Segment has no default constructor
    std::array<Segment,4> boxEdges = {
        Segment(0,0,0,0), Segment(0,0,0,0),
        Segment(0,0,0,0), Segment(0,0,0,0)
    };
    // give the box and chain segments
    std::vector<Segment> chain;
    buildRobotSegments(r, boxEdges, chain);

    double d2 = std::numeric_limits<double>::infinity();

    // Compare every base edge and every obstacle edge, keep the smallest squared distance.
    for (const auto& be : boxEdges)
        for (const auto& e : env)
            d2 = std::min(d2, segSegDist2(be, e, vc));

    // do the previous step for every robot link and every obstacle edge
    for (const auto& l : chain)
        for (const auto& e : env)
            d2 = std::min(d2, segSegDist2(l, e, vc));

    return std::sqrt(d2);
}

void planScenario2(ompl::geometric::SimpleSetup &ss)
{
    // the custom state validity checker is called to extract the obstacles in it 
    auto *svc = dynamic_cast<ChainBoxValidityChecker*>(ss.getStateValidityChecker().get());
    // if the function isint ChainBoxValidityChecker the code is stopped
    if (!svc) {
        std::cerr << "[Scenario2] StateValidityChecker is not ChainBoxValidityChecker.\n";
        return;
    }
    // the obstacles are extracted and stored in environment
    const Environment &env = svc->obstacles();

    // create a custom cost function for planner to keep robot as far away from object as possible, OptimizationObjective is inherited because it is a library which has useful functions to minimize costs and maximize clearance
    struct IntegralClearanceObjective : public ob::StateCostIntegralObjective {
        // declare the variables for obstacles, validity checker and eps is used so that no division by 0 is done
        const Environment& env_;
        const ChainBoxValidityChecker* vc_;
        double eps_;
        // class constructor is called
        IntegralClearanceObjective(const ob::SpaceInformationPtr &si,
                                   const Environment& env,
                                   const ChainBoxValidityChecker* vc,
                                   double eps=1e-6)
            : ob::StateCostIntegralObjective(si, true), env_(env), vc_(vc), eps_(eps) {}

        // conver the ompl state to a 7D vector[x,y,z,q0,q1,q2,q3]
        ob::Cost stateCost(const ob::State *s) const override {
            std::vector<double> r; si_->getStateSpace()->copyToReals(r, s);
            // calls the trueWorkspaceClearance computes the actual Euclidean clearance from the robot (box + links) to all obstacle edges
            double d = trueWorkspaceClearance(r, env_, vc_);
            return ob::Cost(1.0 / (eps_ + d));  // same unit the tool prints
        }
    };

    // Asymptotically optimal planners try to optimize 
    auto si = ss.getSpaceInformation();

    {
        auto lengthObj = std::make_shared<ob::PathLengthOptimizationObjective>(si);
        ss.getProblemDefinition()->setOptimizationObjective(lengthObj);

        auto prm_len = std::make_shared<og::PRMstar>(si);
        ss.setPlanner(prm_len);

        ss.setup();
        ss.getPlanner()->clear();
        ss.getProblemDefinition()->clearSolutionPaths();

        if (ss.solve(60.0))  // run for 60 seconds
        {
            og::PathGeometric &pg = ss.getSolutionPath();
            pg.interpolate(400);

            // Write the path in the file clear.txt 
            std::ofstream ofs("clear_path.txt");
            for (std::size_t i = 0; i < pg.getStateCount(); ++i) {
                std::vector<double> r;
                ss.getStateSpace()->copyToReals(r, pg.getState(i));
                for (std::size_t k = 0; k < r.size(); ++k)
                    ofs << r[k] << (k + 1 == r.size() ? '\n' : ' ');
            }
        }
        else {
            std::cerr << "[Scenario2] No solution within time (PRM* length).\n";
        }
    }
    // PRM* is executed with the true workspace clearance 
    {
        auto objective = std::make_shared<IntegralClearanceObjective>(si, env, svc);
        ss.getProblemDefinition()->setOptimizationObjective(objective);

        auto prm_clear = std::make_shared<og::PRMstar>(si);
        ss.setPlanner(prm_clear);

        ss.setup();
        ss.getPlanner()->clear();
        ss.getProblemDefinition()->clearSolutionPaths();

        if (ss.solve(300.0))  // run for 300seconds to allow the planner to improve
        {
            og::PathGeometric &pg = ss.getSolutionPath();
            pg.interpolate(800);

            // Write the path in the file optimal_clear_path.txt 
            std::ofstream ofs("optimal_clear_path.txt");
            for (std::size_t i = 0; i < pg.getStateCount(); ++i) {
                std::vector<double> r;
                ss.getStateSpace()->copyToReals(r, pg.getState(i));
                for (std::size_t k = 0; k < r.size(); ++k)
                    ofs << r[k] << (k + 1 == r.size() ? '\n' : ' ');
            }
        }
        else {
            std::cerr << "[Scenario2] No solution within time (PRM* clearance).\n";
        }
    }
}

void benchScenario2(ompl::geometric::SimpleSetup &ss)
{
    //TODO: Benchmark RRT*, PRM*, RRT# for 10 trials with 60 secounds timeout.
    double runtime_limit = 60, memory_limit = 1024;
    int run_count = 10;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "ChainBox_Clearance");

    auto *svc = dynamic_cast<ChainBoxValidityChecker*>(ss.getStateValidityChecker().get());
    std::vector<std::pair<double,double>> corners;
    if (svc) {
        const Environment &env = svc->obstacles();
        for (std::size_t i = 0; i + 3 < env.size(); i += 4) {
            double xmin = std::numeric_limits<double>::infinity();
            double ymin = std::numeric_limits<double>::infinity();
            double xmax = -xmin, ymax = -ymin;
            for (int k = 0; k < 4; ++k) {
                const auto &e = env[i + k];
                xmin = std::min({xmin, e.x0, e.x1});
                ymin = std::min({ymin, e.y0, e.y1});
                xmax = std::max({xmax, e.x0, e.x1});
                ymax = std::max({ymax, e.y0, e.y1});
            }
            corners.emplace_back(xmin, ymin);
            corners.emplace_back(xmax, ymin);
            corners.emplace_back(xmax, ymax);
            corners.emplace_back(xmin, ymax);
        }
    }

    // custom function to try to increase optimality, path cost is the sum of costs along the path 
    struct ApproxClearanceObjective : public ob::StateCostIntegralObjective {
        // if a path is very close to obstacles, total cost becomes large too, cs_ stores obstacle corners
        const std::vector<std::pair<double,double>> cs_;
        ApproxClearanceObjective(const ob::SpaceInformationPtr &si,
                                 std::vector<std::pair<double,double>> corners)
            : ob::StateCostIntegralObjective(si, true), cs_(std::move(corners)) {}

        // calculate minimum distance from (x,y) to obstacle corner
        ob::Cost stateCost(const ob::State *s) const override {
            std::vector<double> r; si_->getStateSpace()->copyToReals(r, s);
            const double x = r[0], y = r[1];
            double dmin = std::numeric_limits<double>::infinity();
            for (const auto &c : cs_) {
                double dx = x - c.first, dy = y - c.second;
                dmin = std::min(dmin, std::sqrt(dx*dx + dy*dy));
            }
            // large dmin => smaller cost, eps is added so that the division never goes to undefined
            const double eps = 1e-3;
            return ob::Cost(1.0 / (eps + dmin));
        }
    };

    auto objective = std::make_shared<ApproxClearanceObjective>(ss.getSpaceInformation(), corners);
    ss.getProblemDefinition()->setOptimizationObjective(objective);

    b.addPlanner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<og::PRMstar>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<og::RRTXstatic>(ss.getSpaceInformation())); // RRT#

    b.benchmark(request);
    b.saveResultsToFile("clear_benchmark.log");
}

std::shared_ptr<ompl::base::CompoundStateSpace> createChainBoxSpace()
{   //TODO Create the Chainbox ConfigurationSpace
    auto space = std::make_shared<ompl::base::CompoundStateSpace>();

    // the chain box robot has a square base which means the parameters used to describe it are (x,y,theta) in the 2D space
    // R^2 + SO^2
    auto xy_plane = std::make_shared<ob::RealVectorStateSpace>(2);

    ob::RealVectorBounds b(2);
    b.setLow(-5.0);
    b.setHigh(5.0);
    xy_plane->setBounds(b);
    space->addSubspace(xy_plane,1.0);

    space->addSubspace(std::make_shared<ob::SO2StateSpace>(),1.0);

    // the 4 links have revolute joints so they all have only (theta_i) as their parameters
    for(int i=0; i<4; ++i){
        space->addSubspace(std::make_shared<ob::SO2StateSpace>(),1.0);
    }

    //using the lock function so that the structure of state space cannot be altered further
    space->lock();
    return space;
}
void setupCollisionChecker(ompl::geometric::SimpleSetup &ss, Environment &env)
{   //TODO Setup the stateValidity Checker
    ss.setStateValidityChecker(std::make_shared<ChainBoxValidityChecker>(ss.getSpaceInformation(), env));
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.002);

}

    
int main(int argc, char **argv)
{

    int scenario; 
    Environment env;
    std::vector<double> startVec;
    std::vector<double> goalVec;
    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) Robot Reaching Task" << std::endl;
        std::cout << " (2) Robot Avoiding Task" << std::endl;

        std::cin >> scenario;
    } while (scenario < 1 || scenario > 3);

    switch (scenario)
    {
        case 1:
            makeScenario1(env, startVec, goalVec);
            break;
        case 2:
            makeScenario2(env, startVec, goalVec);
            break;
        default:
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }

    auto space = createChainBoxSpace();
    ompl::geometric::SimpleSetup ss(space);

    setupCollisionChecker(ss, env);

    //setup Start and Goal
    ompl::base::ScopedState<> start(space), goal(space);
    space->setup();
    space->copyFromReals(start.get(), startVec);
    space->copyFromReals(goal.get(), goalVec);
    ss.setStartAndGoalStates(start, goal);

    switch (scenario)
    {
        case 1:
            planScenario1(ss);
            benchScenario1(ss);
            break;
        case 2:
            planScenario2(ss);
            benchScenario2(ss);
            break;
        default:
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }

}
