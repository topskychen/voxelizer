/*
 * Commons.cpp
 *
 *  Created on: 23 Jun, 2014
 *      Author: chenqian
 */

#include "Commons.h"

int random(const int l, const int r) {
	return (int)((1.0*random())/RAND_MAX*(r-l))+l;
}

inline void fill(const Vec3f& vc, float ft[3]) {
	ft[0] = vc[0];
	ft[1] = vc[1];
	ft[2] = vc[2];
}

bool collide(const v3_p& halfUnit, const v3_p& boxAA, const tri_p& tri) {
	Vec3f vBoxCenter = (*boxAA)+*halfUnit;
	float halfSize[3];
	fill(*halfUnit, halfSize);
	float boxCenter[3];
	fill(vBoxCenter, boxCenter);
	float tricerts[3][3];
	fill(tri->a, tricerts[0]);
	fill(tri->b, tricerts[1]);
	fill(tri->c, tricerts[2]);
	return triBoxOverlap(boxCenter, halfSize, tricerts);
}


