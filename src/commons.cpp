/*
 * Commons.cpp
 *
 *  Created on: 23 Jun, 2014
 *      Author: chenqian
 */

#include "commons.h"

#include "absl/strings/numbers.h"

namespace voxelizer {

int Random(const int l, const int r) {
  return (int)((1.0 * random()) / RAND_MAX * (r - l)) + l;
}

inline void Fill(const Vec3f& vc, float ft[3]) {
  ft[0] = vc[0];
  ft[1] = vc[1];
  ft[2] = vc[2];
}

bool Collide(const Vec3f& half_unit, const Vec3f& box_aa, const TriangleP& tri) {
  Vec3f v_box_center = box_aa + half_unit;
  float half_size[3];
  Fill(half_unit, half_size);
  float box_center[3];
  Fill(v_box_center, box_center);
  float tricerts[3][3];
  Fill(tri.a, tricerts[0]);
  Fill(tri.b, tricerts[1]);
  Fill(tri.c, tricerts[2]);
  return TriBoxOverlap(box_center, half_size, tricerts);
}

bool ToVector3Int(const std::vector<std::string>& vs, std::vector<int>& vi) {
  vi.clear();
  for (const auto& v : vs) {
    int i;
    if (!absl::SimpleAtoi(v, &i)) {
      return false;
    }
    vi.push_back(i);
  }
  return true;
}
bool ToVector3Float(const std::vector<std::string>& vs, std::vector<float>& vf) {
  vf.clear();
  for (const auto& v : vs) {
    float f;
    if (!absl::SimpleAtof(v, &f)) {
      return false;
    }
    vf.push_back(f);
  }
  return true;
}

}  // namespace voxelizer
