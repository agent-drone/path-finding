#pragma once

#include "common.h"
#include "woctomap.h"

#include <ompl/config.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/GeneticSearch.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Wompl 
{
public:
  std::shared_ptr<ob::RealVectorStateSpace> space;
  std::shared_ptr<og::SimpleSetup> ss;
  std::shared_ptr<og::RRTstar> rrt;
  std::shared_ptr<Woctomap> wp;
  Wompl(std::shared_ptr<Woctomap> swp);
  void set_start(double x, double y, double z);
  void set_goal(double x, double y, double z);
  double** compute_waypoints(float sampling_dist, float minimum_free_volume);
};
