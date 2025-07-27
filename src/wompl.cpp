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

#include <vector>
#include <array>

#include "../include/wompl.h"

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

  this->rrt = std::make_shared<og::RRTstar>(ss->getSpaceInformation());
  rrt->setRange(0.05);
  ss->setPlanner(rrt);
}

void
Wompl::criterion(double sampling_dist, double minimum_volume)
{
  /* Check for a valid path every `SAMPLING_DIST`. Verify that
  `MINIMUM VOLUME` is free at that point for it to be considered 
  a valid point. */
  ss->setStateValidityChecker([&](const ob::State *state) {
    auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
    float x,y,z;
    x = pos->values[0];
    y = pos->values[1];
    z = pos->values[2]; 
    octomap::point3d center(x, y, z);
    octomap::point3d min_bound = center - octomap::point3d(minimum_volume, minimum_volume, minimum_volume);
    octomap::point3d max_bound = center + octomap::point3d(minimum_volume, minimum_volume, minimum_volume);

    for (auto it = wp->otree.begin_leafs_bbx(min_bound, max_bound), end = wp->otree.end_leafs_bbx(); it != end; ++it)
    {
      if (wp->otree.isNodeOccupied(*it)) return false;

      // Optionally: if you want to count only known free space, skip unknowns
      // if (!it->isOccupied() && it->getOccupancy() > 0.0) free_voxels++;

      // Make sure voxel center is inside sphere if you want spherical volume (optional)
      // if ((it.getCoordinate() - center).norm() <= sampling_dist)
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
