/*
DATE
  19.07.25

DESCRIPTION
  This source file makes a wrapper over the `Octomap`
  library source code, needed for the aleOS pre-flight(and in-flight)
  modules.
*/

#include "../include/woctomap.h"
#include "../include/common.h"

#include <stdio.h>

bool read_ascii_file(const char *file, OCTREE *tree) {
  FILE *fptr = fopen(file, "r");
  if (!fptr)
    return false;
  float x, y, z;
  char line[256];

  while (fgets(line, sizeof(line), fptr)) {
    if (sscanf(line, "%f %f %f", &x, &y, &z)) {
      tree->updateNode(POINT(x, y, z), true);
    } else {
      continue;
    }
  }
  fclose(fptr);
  return true;
}

OCTREE
read_bt_file(const char *file) { return OCTREE(file); }

Woctomap::Woctomap(float size) : otree(size) {}
Woctomap::Woctomap(const char *file) : otree(0.1) { otree.readBinary(file); }

void Woctomap::read_from_text(const char *file) {
  read_ascii_file(file, &otree);
}

void Woctomap::read_from_bt(const char *file) { otree.readBinary(file); }

void Woctomap::add_point(float x, float y, float z, uint8 occupied) {
  otree.updateNode(POINT(x, y, z), !!occupied);
}

bool Woctomap::free(float x, float y, float z) {
  octomap::OcTreeNode *on = otree.search(POINT(x, y, z));
  if (on == NULL)
    return true;

  bool occupied = otree.isNodeOccupied(on);
  return !occupied;
}

void Woctomap::save_to_bt(const char *file) { otree.writeBinary(file); }
