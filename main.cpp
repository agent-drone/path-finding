/*
DATE
  19.07.25

DESCRIPTION
  The main driver file for the Octomap and OMPL.
*/

#include "include/common.h"
#include "include/woctomap.h"

#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include <stdio.h>

FILE *fptr;

int main() {
  Woctomap wp(0.1);
  wp.read_from_bt("dubdub.bt");
  printf("::> %d\n", wp.otree.calcNumNodes());
  printf("nPTS: [ %d ]\nThe point at: [1,2,3] is occupied: %d", wp.otree.calcNumNodes(), 0);
}