#pragma once

#include "common.h"

/**
 * @brief A function for reading point cloud data from a text file, in ASCII
 * format.
 * @param file: A string pointing to the filepath; example: `./point_data.txt`.
 */
bool read_ascii_file (const char *file, OCTREE * tree);

/**
 * @brief: A function for reading octree data from a given octomap BT file.
 */
OCTREE read_bt_file (const char *file);

class Woctomap
{
public:
  OCTREE otree;
  Woctomap (float size = 0.1);
  Woctomap (const char* file);
  void read_from_text (const char *file);
  void read_from_bt (const char *file);
  void add_point (float x, float y, float z, uint8 occupied);
  void save_to_bt(const char *file);
  bool free (float x, float y, float z);
};
