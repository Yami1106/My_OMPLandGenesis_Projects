#include "RTP.h"
#include <limits>
#include <algorithm>                       
#include <vector>                         
#include "ompl/geometric/PathGeometric.h"  
#include "ompl/base/goals/GoalSampleableRegion.h"  
#include "ompl/tools/config/SelfConfig.h"                 

//constructor declaration 
ompl::geometric::RTP::RTP(const base::SpaceInformationPtr &si, bool addIntermediateStates)
    : base::Planner(si, addIntermediateStates ? "RTPintermediate" : "RTP")
{
    // allow planner to return approximate solutions, in case exact is not found
    specs_.approximateSolutions = true;  

    // the edges of trees have direction 
    specs_.directed = true;              

    // declares the range,goal bias and intermediate states so that it can be tuned as ompl parameters
    Planner::declareParam<double>("range", this, &RTP::setRange, &RTP::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RTP::setIntermediateStates, &RTP::getIntermediateStates, "0,1");

    addIntermediateStates_ = addIntermediateStates;
}

//destructor declaration to clean up memory 
ompl::geometric::RTP::~RTP(){
    freeMemory();
}

void ompl::geometric::RTP::setup(){
    Planner::setup();

    // extension range is automatically set if the user does not provide it 
    tools::SelfConfig sc(si_,getName());
    sc.configurePlannerRange(maxDistance_);
}

// fucntion to reset the planner state so that it can be used for a new run
void ompl::geometric::RTP::clear(){

    //resets the base class Planner
    Planner::clear();

    //release sampler
    sampler_.reset();
    freeMemory();
    lastGoalMotion_ = nullptr;
}

// function to clear the tree's memory
void ompl::geometric::RTP::freeMemory(){
    for (Motion *m : motions_)
    {
        if (m && m->state)
            si_->freeState(m->state);
        delete m;
    }
    motions_.clear();
    lastGoalMotion_ = nullptr;

}

// makes planner parameters and samplers ready for use when the solve() function is called
ompl::base::PlannerStatus ompl::geometric::RTP::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    // check if the planner has been properly setup
    checkValidity();

    ompl::base::Goal *goal = pdef_->getGoal().get();

    auto *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal);

    // create a sampler if it dosent exist
    if (!sampler_){
        sampler_ = si_->allocStateSampler();
    }

    while (const ompl::base::State *st = pis_.nextStart()){
        //motion object is created with a fresh ompl state
        Motion *m = new Motion(si_);

        // the start state is copied into the motion state
        si_->copyState(m->state, st);

        //root nodes do not have a parent
        m->parent = nullptr;
        motions_.push_back(m);
    }

    if (motions_.empty()){
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_START;
    }

    //will hold randomly sampled state
    ompl::base::State *xrand = si_->allocState();

    //the new added state
    ompl::base::State *xnew = si_->allocState();

    Motion *exactSol   = nullptr;                       
    Motion *approxSol  = nullptr;                        
    double  approxDist = std::numeric_limits<double>::infinity();

    //motion planning loop, runs till the terminating condition is satisfied
    while(!ptc){

        // sample target stae with goal biasing
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(xrand);       
        else
            sampler_->sampleUniform(xrand); 

        // pick an existing node as parent
        Motion *parent = motions_[rng_.uniformInt(0, static_cast<int>(motions_.size()) - 1)];

        //connect the node to new node if distance is less than maxdistance
        double dist = si_->distance(parent->state, xrand);
        if (dist > maxDistance_)
        {
            // Create xnew = interpolate(parent, xrand, step)
            si_->getStateSpace()->interpolate(parent->state, xrand, maxDistance_ / dist, xnew);
        }
        else
        {
            // copy the sample as xnew
            si_->copyState(xnew, xrand);
        }

        //collision checking 
        bool valid = false;

        // First just check the motion validity with the 2-argument overload
        valid = si_->checkMotion(parent->state, xnew);

        // if the state is invalid free the state
        if(!valid){
            continue;
        }

        //inserting the valid states
        Motion *tail = nullptr;

        // If adding intermediate states, fetch them via getMotionStates AFTER we know the motion is valid
        if (addIntermediateStates_)
        {
            std::vector<ompl::base::State *> intermediates;
            const unsigned int count = si_->getStateSpace()->validSegmentCount(parent->state, xnew);
            if (si_->getMotionStates(parent->state, xnew, intermediates, count, true, true))
            {
                // OMPL returns states including the first state; free it to avoid duplicate of parent->state
                if (!intermediates.empty())
                {
                    si_->freeState(intermediates.front());
                    intermediates.erase(intermediates.begin());
                }
            }

            for (std::size_t i = 0; i < intermediates.size(); ++i)
            {
                Motion *mi  = new Motion;                               
                mi->state   = intermediates[i];               
                // first links to parent, then chain           
                mi->parent  = (i == 0 ? parent : tail);         
                //add to tree         
                motions_.push_back(mi);                                  
                tail = mi;                                               
            }
        }

        //creates a new tree node with its own copy of the configuration, connects it to the parent
        {
            Motion *m  = new Motion;
            m->state   = si_->cloneState(xnew);                         
            m->parent  = (tail ? tail : parent);                        
            motions_.push_back(m);
            tail = m;                                                   
        }

        //check if reached goal 
        if (goal)
        {
            double gdist = 0.0;
            bool   sat   = goal->isSatisfied(tail->state, &gdist);  // check goal satisfaction & distance

            // closest-to-goal node
            if (gdist < approxDist)
            {
                approxDist = gdist;
                approxSol  = tail;
            }

            // we found an exact solution—stop
            if (sat)
            {
                exactSol = tail;
                lastGoalMotion_ = tail;
                break;
            }
        }
    }

    // Pick exact solution if its there, else best approximate is taken
    Motion *sol   = (exactSol ? exactSol : approxSol);
    bool    solved = (exactSol != nullptr);
    bool    approximate = (exactSol == nullptr && sol != nullptr);

    if (sol)
    {
        // Reconstruct path from solution to start
        std::vector<Motion*> mpath;
        for (Motion *m = sol; m != nullptr; m = m->parent)
            mpath.push_back(m);

        std::reverse(mpath.begin(), mpath.end());

        auto path = std::make_shared<ompl::geometric::PathGeometric>(si_);
        for (Motion *m : mpath)
            path->append(m->state);

        pdef_->addSolutionPath(path, approximate, approxDist, getName());
    }

    
    si_->freeState(xrand);
    si_->freeState(xnew);

    return {solved, approximate};
}

void ompl::geometric::RTP::getPlannerData(ompl::base::PlannerData &data) const
{
    ompl::base::Planner::getPlannerData(data);

    // Add all vertices and edges
    for (Motion *m : motions_)
    {
        // Add vertex for each motion’s state
        data.addVertex(ompl::base::PlannerDataVertex(m->state));

        // If it has a parent, add a directed edge parent -> m
        if (m->parent)
            data.addEdge(
                ompl::base::PlannerDataVertex(m->parent->state),
                ompl::base::PlannerDataVertex(m->state));
    }

    // Mark goal node if it is saved
    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(ompl::base::PlannerDataVertex(lastGoalMotion_->state));
}
