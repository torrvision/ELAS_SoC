#ifndef ARMCODE_H_
#define ARMCODE_H_

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <algorithm>
#include <math.h>

#include <ap_int.h>
#include <vector>

#include "triangle.h"
#include "matrix.h"
#include "Topmain.h"


struct support_pt {
	int32_t u;
	int32_t v;
	int32_t d;
	support_pt(int32_t u,int32_t v,int32_t d):u(u),v(v),d(d){}
};

struct triangle {
	int32_t c1,c2,c3;
	float   t1a,t1b,t1c;
	float   t2a,t2b,t2c;
	triangle(int32_t c1,int32_t c2,int32_t c3):c1(c1),c2(c2),c3(c3){}
};

void armcode (int16_t *TriagedResult, unsigned long long *ValidDispsTop, unsigned long long *ValidDispsBot, uint8_t *PlaneCentres, int DS);
void extractSupport (int16_t *Supp_Pnts, std::vector<support_pt> &p_support, unsigned long long *ValidDispsTop, unsigned long long *ValidDispsBot, int DS);

std::vector<triangle> computeDelaunayTriangulation (std::vector<support_pt> p_support);
void computeDisparityPlanes (std::vector<support_pt> p_support,std::vector<triangle> &tri);
void findPlaneCentres(std::vector<support_pt> p_support, std::vector<triangle> tri, int32_t *grid_dims, uint8_t *d_plane, int width, int height);


#endif /* SRC_ARMCODE_H_ */
