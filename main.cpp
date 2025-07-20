/*
DATE
  19.07.25

DESCRIPTION
  The main driver file for the Octomap and OMPL.
*/

#include "include/common.h"
#include "include/woctomap.h"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

#include <stdio.h>

int main() {
  Woctomap wp(0.1);
  wp.read_from_bt("data/dubdub.bt");
  /* do OMPL shit here. */

  double sx, sy, sz;
  double gx, gy, gz;
  wp.otree.getMetricMin(sx, sy, sz);
  wp.otree.getMetricMax(gx, gy, gz);

  printf("[ %f, %f, %f ]\n", sx, sy, sz);

  // bool path = ss.solve(20);
  // if (!path) return 0;

  // // ss.simplifySolution();
  // auto pts = ss.getSolutionPath();

  // // @TODO: iterate  and see all the generated points
  // // !path
  // for (int i = 0; i < pts.getStateCount(); i++) {
  //   float x, y, z;
  //   auto *cstate = pts.getState(i)->as<ob::RealVectorStateSpace::StateType>();
  //   x = cstate->values[0];
  //   y = cstate->values[1];
  //   z = cstate->values[2];
  //   wp.otree.updateNode(POINT(x,y,z), false);
  //   printf("[ %f %f %f ]\n", x,y,z);
    
  // }
  
  // wp.otree.writeBinary("path.bt");
}