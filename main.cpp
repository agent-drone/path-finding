/*
DATE
  19.07.25

DESCRIPTION
  The main driver file for the Octomap and OMPL.
*/

#include "include/common.h"
#include "include/woctomap.h"

#include <ompl/config.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/GeneticSearch.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <stdio.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main() {
  Woctomap wp(0.1);
  wp.read_from_bt("data/dubdub.bt");
  /* do OMPL shit here. */

  std::shared_ptr<ob::RealVectorStateSpace> space = std::make_shared<ob::RealVectorStateSpace>(3);
  space->setBounds(-200, 200);

  ompl::geometric::SimpleSetup ss(space);
  ss.setStateValidityChecker([&](const ob::State *state) {
    auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
    return wp.free(pos->values[0], pos->values[1], pos->values[2]);
  });

  double sx, sy, sz;
  double gx, gy, gz;
  wp.otree.getMetricMin(sx, sy, sz);
  wp.otree.getMetricMax(gx, gy, gz);

  printf("[ %f, %f, %f ]\n", sx, sy, sz);

  ob::ScopedState<> start(space), goal(space);
  start->as<ob::RealVectorStateSpace::StateType>()->values[0] = -50;
  start->as<ob::RealVectorStateSpace::StateType>()->values[1] = -14;
  start->as<ob::RealVectorStateSpace::StateType>()->values[2] = 5;

  goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 50;
  goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0;
  goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = 5;

  ss.setPlanner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
  ss.setStartAndGoalStates(start, goal);
  ss.solve(5.0);
}