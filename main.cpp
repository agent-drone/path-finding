/*
DATE
  19.07.25

DESCRIPTION
  The main driver file for the Octomap and OMPL.
*/

#include "include/common.h"
#include "include/woctomap.h"
#include "include/wompl.h"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

#include <stdio.h>

int main() {
  Woctomap wp("./data/dubdub.bt");

  Wompl mp(std::make_shared<Woctomap>(wp));
  mp.set_start(-50, 0, 6.5);
  mp.set_goal(60, 0, 5);
  mp.criterion(0.05, 5);
  mp.solve(20.0);
}