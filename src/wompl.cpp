/*
DATE
  19.07.25

DESCRIPTION
  The `wompl` file is a wrapper over the famous Open MPL library for aleOS.
*/

#include <ompl/config.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/GeneticSearch.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "include/wompl.h"

  // std::shared_ptr<ob::RealVectorStateSpace> space;
  // Wompl();
  // void set_start(double x, double y, double z);
  // void set_goal(double x, double y, double z);
  // double** compute_waypoints(float sampling_dist, float minimum_free_volume);

Wompl::Wompl(std::shared_ptr<Woctomap> swp) {
  this->wp = swp;
  this->space = std::make_shared<ob::RealVectorStateSpace>(3);
  this->ss = std::make_shared<og::SimpleSetup>(space);
  space->setBounds(-200, 200); // @TODO: allow user to configure this later on!

  // @TODO: allow the user to set their own state validity checkers
  // later on.
  ss->setStateValidityChecker([&](const ob::State *state) {
    auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
    float x,y,z;
    x = pos->values[0];
    y = pos->values[1];
    z = pos->values[2]; 

    bool occ =    wp->free(x,y,z) 
                && wp->free(x+0.05,y,z) 
                && wp->free(x,y+0.05,z)
                && wp->free(x,y,z+0.05)
                && wp->free(x-0.05,y,z)
                && wp->free(x,y-0.05,z)
                && wp->free(x,y,z-0.05);
    return occ;
  });

  this->rrt = std::make_shared<og::RRTstar>(ss->getSpaceInformation());
  rrt->setRange(0.05);
  ss->setPlanner(rrt);

}

void
Wompl::set_start(double x, double y, double z) {
  ob::ScopedState<> start(space);
  start->as<ob::RealVectorStateSpace::StateType>()->values[0] = x;
  start->as<ob::RealVectorStateSpace::StateType>()->values[1] = y;
  start->as<ob::RealVectorStateSpace::StateType>()->values[2] = z;
  ss->setStartState(start);
}

void
Wompl::set_goal(double x, double y, double z) {
  ob::ScopedState<> goal(space);
  goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = x;
  goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = y;
  goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = z;
  ss->setGoalState(goal);
}