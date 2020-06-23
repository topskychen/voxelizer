/*
 * TriBox.h
 *
 *  Created on: 23 Jun, 2014
 *      Author: chenqian
 */

#ifndef TRIBOX_H_
#define TRIBOX_H_

namespace voxelizer {

int TriBoxOverlap(float boxcenter[3], float boxhalfsize[3],
                  float triverts[3][3]);

}

#endif /* TRIBOX_H_ */
