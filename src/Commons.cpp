/*
 * Commons.cpp
 *
 *  Created on: 23 Jun, 2014
 *      Author: chenqian
 */

#include "commons.h"

int Random(const int l, const int r) {
	return (int)((1.0*random())/RAND_MAX*(r-l))+l;
}

inline void Fill(const Vec3f& vc, float ft[3]) {
	ft[0] = vc[0];
	ft[1] = vc[1];
	ft[2] = vc[2];
}

bool Collide(const v3_p& halfUnit, const v3_p& boxAA, const tri_p& tri) {
	Vec3f vBoxCenter = (*boxAA)+*halfUnit;
	float halfSize[3];
	Fill(*halfUnit, halfSize);
	float boxCenter[3];
	Fill(vBoxCenter, boxCenter);
	float tricerts[3][3];
	Fill(tri->a, tricerts[0]);
	Fill(tri->b, tricerts[1]);
	Fill(tri->c, tricerts[2]);
	return TriBoxOverlap(boxCenter, halfSize, tricerts);
}


