/* Author: Ali Golestaneh and Constantinos Chamzas */

#ifndef DISK_SAMPLER
#define DISK_SAMPLER

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <cmath>

namespace ob = ompl::base;
using ValidStateSamplerPtr = std::shared_ptr<ob::ValidStateSampler>;

bool isStateValid(const ob::State *state);

class DiskSampler : public ob::ValidStateSampler {
   public:
    DiskSampler(const ob::SpaceInformation *si, const std::string name)
        : ValidStateSampler(si) {
        name_ = name;
    }

    bool sampleNaive(ob::State *state);
    bool sampleCorrect(ob::State *state);

    bool virtual sample(ob::State *state) override {
        if (name_ == "Naive") return sampleNaive(state);

        if (name_ == "Correct") return sampleCorrect(state);
        return true;
    }

    bool sampleNear(ob::State * /*state*/, const ob::State * /*near*/,
                    const double /*distance*/) override {
        throw ompl::Exception("MyValidStateSampler::sampleNear",
                              "not implemented");
        return false;
    }

   private:
    ompl::RNG rng_;
};

inline ob::ValidStateSamplerPtr allocDiskSamplerNaive(
    const ob::SpaceInformation *si) {
    return std::make_shared<DiskSampler>(si, "Naive");
}

inline ob::ValidStateSamplerPtr allocDiskSamplerCorrect(
    const ob::SpaceInformation *si) {
    return std::make_shared<DiskSampler>(si, "Correct");
}

#endif
