#ifndef RANDOM_TREE_H
#define RANDOM_TREE_H

#include "ompl/geometric/planners/PlannerIncludes.h"
#include <vector>

namespace ompl
{
    namespace geometric
    {
        // TODO: Implement RTP as described
        // class RTP : public base::Planner
        // {
        // };

        class RTP : public base::Planner{
            public:
            //defining the class constructor, it initializes the spacial information variable pointer si and the addintermediatestates variable
                RTP(const base::SpaceInformationPtr &si, bool addIntermediateStates = false);

                // destructor is called to release the memory efficiently 
                ~RTP() override;

                // this funciton is used to export the tree for visualization
                void getPlannerData(base::PlannerData &data) const override;
                // this is the function that tries to find a path, keeps running till it finds a path or times out due to the termination condiditon
                base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

                //this function resets the planner state
                void clear() override;

                // (added) ensure out-of-line definition matches a declaration
                void setup() override;

                // a goal bias is set to increase the possibility to reach the goal 
                void setGoalBias(double goalBias){
                    goalBias_ = goalBias;
                }

                double getGoalBias() const{
                    return goalBias_;
                }

                // function to return true if any intermediate states are generated 
                bool getIntermediateStates() const{
                    return addIntermediateStates_;
                }

                // specifies if the generated intermediate states are to be added 
                void setIntermediateStates(bool addIntermediateStates)
                {
                     addIntermediateStates_ = addIntermediateStates;
                }

                // sets the distance that the tree can extend to in one step 
                void setRange(double distance)
                {
                    maxDistance_ = distance;
                }

                double getRange() const
                {
                    return maxDistance_;
                }

                protected:

                    // this node is used to store the state and a pointer to its parent
                    class Motion
                    {
                    public:
                        Motion() = default;
                        Motion(const base::SpaceInformationPtr &si) : state(si->allocState()){}

                        ~Motion() = default;
                        base::State *state{nullptr};
                        Motion *parent{nullptr};
                    };
                    
                    // memory allocated by planner is freed
                    void freeMemory();

                    double distanceFunction(const Motion *a, const Motion *b) const
                    {
                        return si_->distance(a->state, b->state);
                    }

                    base:: StateSamplerPtr sampler_;

                    // datastructure to contain the tree of motions
                    std::vector<Motion *> motions_;

                    double goalBias_{.05};
                    double maxDistance_{0.};
                    bool addIntermediateStates_;
                    RNG rng_;

                    Motion *lastGoalMotion_{nullptr};

        };
    }  // namespace geometric
}  // namespace ompl

#endif
