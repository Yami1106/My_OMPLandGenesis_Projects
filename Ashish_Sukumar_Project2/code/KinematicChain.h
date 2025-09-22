/*********************************************************************
 * Software License Agreement (BSD License)
 * (c) 2013 Rice University
 *********************************************************************/
#ifndef OMPL_DEMO_KINEMATIC_CHAIN_
#define OMPL_DEMO_KINEMATIC_CHAIN_

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>

// a 2D line segment
struct Segment { Segment(double p0_x, double p0_y, double p1_x, double p1_y)
: x0(p0_x), y0(p0_y), x1(p1_x), y1(p1_y) {} double x0, y0, x1, y1; };

using Environment = std::vector<Segment>;

class KinematicChainProjector : public ompl::base::ProjectionEvaluator {
public:
    KinematicChainProjector(const ompl::base::StateSpace *space) : ompl::base::ProjectionEvaluator(space) {
        int dimension = std::max(2, (int)std::ceil(std::log((double)space->getDimension())));
        projectionMatrix_.computeRandom(space->getDimension(), dimension);
    }
    unsigned int getDimension() const override { return projectionMatrix_.mat.rows(); }
    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override {
        std::vector<double> v(space_->getDimension()); space_->copyToReals(v, state);
        projectionMatrix_.project(&v[0], projection);
    }
protected: ompl::base::ProjectionMatrix projectionMatrix_;
};

class KinematicChainSpace : public ompl::base::RealVectorStateSpace {
public:
    KinematicChainSpace(unsigned int numLinks, double linkLength, Environment *env = nullptr)
      : ompl::base::RealVectorStateSpace(numLinks), linkLength_(linkLength), environment_(env) {
        ompl::base::RealVectorBounds b(numLinks);
        b.setLow(-boost::math::constants::pi<double>());
        b.setHigh(boost::math::constants::pi<double>());
        setBounds(b);
    }
    void registerProjections() override {
        registerDefaultProjection(std::make_shared<KinematicChainProjector>(this));
    }
    double distance(const ompl::base::State *a, const ompl::base::State *b) const override {
        const auto *sa = a->as<StateType>(); const auto *sb = b->as<StateType>();
        double t1=0., t2=0., dx=0., dy=0., d=0.; 
        for (unsigned int i=0;i<dimension_;++i){ t1+=sa->values[i]; t2+=sb->values[i];
            dx += std::cos(t1)-std::cos(t2); dy += std::sin(t1)-std::sin(t2);
            d  += std::sqrt(dx*dx+dy*dy);
        } return d*linkLength_;
    }
    void enforceBounds(ompl::base::State *s) const override {
        auto *st = s->as<StateType>();
        for (unsigned int i=0;i<dimension_;++i){
            double v = std::fmod(st->values[i], 2.0*boost::math::constants::pi<double>());
            if (v < -boost::math::constants::pi<double>()) v += 2.0*boost::math::constants::pi<double>();
            else if (v >= boost::math::constants::pi<double>()) v -= 2.0*boost::math::constants::pi<double>();
            st->values[i] = v;
        }
    }
    bool equalStates(const ompl::base::State *a, const ompl::base::State *b) const override {
        const auto *sa=a->as<StateType>(), *sb=b->as<StateType>();
        for (unsigned int i=0;i<dimension_;++i)
            if (std::fabs(sa->values[i]-sb->values[i]) >= std::numeric_limits<double>::epsilon()*2.0) return false;
        return true;
    }
    void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t,
                     ompl::base::State *state) const override {
        const auto *f=from->as<StateType>(); const auto *tt=to->as<StateType>();
        auto *st=state->as<StateType>();
        for (unsigned int i=0;i<dimension_;++i){
            double diff = tt->values[i]-f->values[i];
            if (std::fabs(diff) <= boost::math::constants::pi<double>()) st->values[i]=f->values[i]+diff*t;
            else{
                if (diff>0.) diff = 2.0*boost::math::constants::pi<double>()-diff;
                else diff = -2.0*boost::math::constants::pi<double>()-diff;
                st->values[i]=f->values[i]-diff*t;
                if (st->values[i] >  boost::math::constants::pi<double>()) st->values[i]-=2.0*boost::math::constants::pi<double>();
                else if (st->values[i] < -boost::math::constants::pi<double>()) st->values[i]+=2.0*boost::math::constants::pi<double>();
            }
        }
    }
    double linkLength() const { return linkLength_; }
    const Environment *environment() const { return environment_; }
protected:
    double linkLength_; Environment *environment_;
};

class KinematicChainValidityChecker : public ompl::base::StateValidityChecker {
public:
    KinematicChainValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si) {}
    bool isValid(const ompl::base::State *state) const override {
        const auto *space = si_->getStateSpace()->as<KinematicChainSpace>();
        const auto *s = state->as<KinematicChainSpace::StateType>(); return isValidImpl(space, s);
    }
protected:
    bool isValidImpl(const KinematicChainSpace *space, const KinematicChainSpace::StateType *s) const {
        unsigned int n = si_->getStateDimension(); Environment segs;
        double L = space->linkLength(); double th=0., x=0., y=0., xN, yN;
        segs.reserve(n+1);
        for (unsigned int i=0;i<n;++i){ th+=s->values[i]; xN=x+std::cos(th)*L; yN=y+std::sin(th)*L;
            segs.emplace_back(x,y,xN,yN); x=xN; y=yN; }
        xN = x + std::cos(th)*0.001; yN = y + std::sin(th)*0.001; segs.emplace_back(x,y,xN,yN);
        return selfIntersectionTest(segs) && environmentIntersectionTest(segs, *space->environment());
    }
    bool selfIntersectionTest(const Environment &env) const {
        for (unsigned int i=0;i<env.size();++i) for (unsigned int j=i+1;j<env.size();++j)
            if (intersectionTest(env[i], env[j])) return false; return true;
    }
    bool environmentIntersectionTest(const Environment &a, const Environment &b) const {
        for (const auto &i:a) for (const auto &j:b) if (intersectionTest(i,j)) return false; return true;
    }
    bool intersectionTest(const Segment &s0, const Segment &s1) const {
        double s10x=s0.x1-s0.x0, s10y=s0.y1-s0.y0, s32x=s1.x1-s1.x0, s32y=s1.y1-s1.y0;
        double denom = s10x*s32y - s32x*s10y; if (std::fabs(denom) < std::numeric_limits<double>::epsilon()) return false;
        bool denomPos = denom>0; double s02x=s0.x0-s1.x0, s02y=s0.y0-s1.y0;
        double s_num = s10x*s02y - s10y*s02x; if ((s_num < std::numeric_limits<float>::epsilon()) == denomPos) return false;
        double t_num = s32x*s02y - s32y*s02x; if ((t_num < std::numeric_limits<float>::epsilon()) == denomPos) return false;
        if (((s_num-denom > -std::numeric_limits<float>::epsilon()) == denomPos) ||
            ((t_num-denom >  std::numeric_limits<float>::epsilon())  == denomPos)) return false;
        return true;
    }
};

inline Environment createHornEnvironment(unsigned int d, double eps) {
    std::ofstream envFile(boost::str(boost::format("environment_%i.dat") % d));
    std::vector<Segment> env; double w=1./(double)d, x=w, y=-eps, xN, yN, th=0.,
           scale = w*(1.+boost::math::constants::pi<double>()*eps);
    envFile << x << " " << y << "\n";
    for (unsigned int i=0;i<d-1;++i){ th += boost::math::constants::pi<double>()/(double)d;
        xN=x+std::cos(th)*scale; yN=y+std::sin(th)*scale; env.emplace_back(x,y,xN,yN); x=xN; y=yN; envFile<<x<<" "<<y<<"\n"; }
    th=0.; x=w; y=eps; envFile<<x<<" "<<y<<"\n"; scale=w*(1.0 - boost::math::constants::pi<double>()*eps);
    for (unsigned int i=0;i<d-1;++i){ th += boost::math::constants::pi<double>()/d;
        xN=x+std::cos(th)*scale; yN=y+std::sin(th)*scale; env.emplace_back(x,y,xN,yN); x=xN; y=yN; envFile<<x<<" "<<y<<"\n"; }
    envFile.close(); return env;
}
inline Environment createEmptyEnvironment(unsigned int d) {
    std::ofstream envFile(boost::str(boost::format("environment_%i.dat") % d)); envFile.close(); return {};
}
#endif
