/*
DATE
  19.07.25

DESCRIPTION
  The `wompl` file is a wrapper over the famous Open MPL library for 
  the `pathfinder` lib.
  
  Usage of keywords like POINT, OCTREE, etc.; are typedef/bindings over
  Octomap data structures; and can be found under `include/common.h`.
*/

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/GeneticSearch.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <algorithm>
#include <array>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include "../include/common.h"
#include "../include/wompl.h"

Wompl::Wompl(std::shared_ptr<Woctomap> swp) {
  this->wp = swp;
  this->space = std::make_shared<ob::RealVectorStateSpace>(3);
  this->ss = std::make_shared<og::SimpleSetup>(space);
  space->setBounds(-200, 200); // @TODO: allow user to configure this later on!

  this->rrt = std::make_shared<og::RRTstar>(ss->getSpaceInformation());
  rrt->setRange(0.05);
  ss->setPlanner(rrt);
}

void Wompl::criterion(double sampling_dist, double clearance_radius) {
  /* Check for a valid path every `SAMPLING_DIST`. Verify there are
  no obstacle within `CLEARANCE_RADIUS` at that point for 
  it to be considered a valid point. */

  // double garbage = 0;
  // double& mx = garbage;
  // double& my = garbage;
  // double& mz = garbage;

  // wp->otree.getMetricMax(mx, my, mz);
  // if (
  //   clearance_radius > mx
  // || clearance_radius > my
  // || clearance_radius > mz
  // ) {
  //   std::ostringstream oss;
  //   oss << "Search radius [" << clearance_radius << "] is outside of world bounds [" << mx << " " << my << " " << mz << "]!\n";
  //   throw std::runtime_error(oss.str());
  // }

  ss->setStateValidityChecker([&](const ob::State *state) {
    auto *pos = state->as<ob::RealVectorStateSpace::StateType>();

    POINT center(pos->values[0], pos->values[1], pos->values[2]);

    POINT min_bound = center - POINT(
      clearance_radius, clearance_radius, clearance_radius);
    POINT max_bound = center + POINT(
      clearance_radius, clearance_radius, clearance_radius);

    for (
      auto it = wp->otree.begin_leafs_bbx(min_bound, max_bound), end = wp->otree.end_leafs_bbx(); it != end; it++) {
      POINT p = it.getCoordinate();
      if (wp->otree.isNodeOccupied(*it)) {
        /* verify that the occupied octree node is 
        * *actually* in our avoidance radius. */
        double voxel_size = it.getSize();
        if ((p - center).norm() - voxel_size < clearance_radius) return false;
      }
    }
    return true;
  });
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

bool
Wompl::solve(float limit) {
    return ss->solve(limit);
}

std::vector<std::array<double, 3>>
Wompl::get_computed_waypoints() 
{
    std::vector<std::array<double, 3>> ret = {};
    auto pts = ss->getSolutionPath();
    for (int i = 0; i < pts.getStateCount(); i++) {
      float x, y, z;
      auto *cs = pts.getState(i)->as<ob::RealVectorStateSpace::StateType>();
      x = cs->values[0];
      y = cs->values[1];
      z = cs->values[2];
      ret.push_back(std::array<double, 3>{x, y, z});
    }
    return ret;
}
