#include "ARMcode.h"

using namespace std;

// Look up table used for creating expanded disparity candidates (including immediate neighbors)
static unsigned long long LUTGrid[64] = {3LL,7LL,14LL,28LL,56LL,112LL,224LL,448LL,896LL,1792LL,3584LL,7168LL,14336LL,28672LL,57344LL,114688LL,229376LL,458752LL,
	917504LL,1835008LL,3670016LL,7340032LL,14680064LL,29360128LL,58720256LL,117440512LL,234881024LL,469762048LL,939524096LL,1879048192LL,3758096384LL,
	7516192768LL,15032385536LL,30064771072LL,60129542144LL,120259084288LL,240518168576LL,481036337152LL,962072674304LL,1924145348608LL,3848290697216LL,
	7696581394432LL,15393162788864LL,30786325577728LL,61572651155456LL,123145302310912LL,246290604621824LL,492581209243648LL,985162418487296LL,
	1970324836974592LL,3940649673949184LL,7881299347898368LL,15762598695796736LL,31525197391593472LL,63050394783186944LL,126100789566373888LL,
	252201579132747776LL,504403158265495552LL,1008806316530991104LL,2017612633061982208LL,4035225266123964416LL,8070450532247928832LL,16140901064495857664LL,
	13835058055282163712LL}; //Last value and first value are special case

void armcode (int16_t *Supp_Pnts, unsigned long long *ValidDispsTop, unsigned long long *ValidDispsBot, uint8_t *PlaneCentres, int DS){

	// Clear the GridVectors
	for (int32_t y = 0; y < grid_height_var; y++){
		for(int32_t x = 0; x < grid_width_var; x++){
			ValidDispsTop[y*grid_width_var + x] = (unsigned long long)(0);
			ValidDispsBot[y*grid_width_var + x] = (unsigned long long)(0);
		}
	}

	// Transform support points from image representation into a vector representation
	vector<support_pt> p_support;
	extractSupport(Supp_Pnts, p_support, ValidDispsTop, ValidDispsBot, DS);

	// if not enough support points for triangulation
	if (p_support.size()<3) {
		cout << "ERROR: Need at least 3 support points!" << endl;
		return;
	}

	// Compute Delaunay Triangulation
	vector<triangle> tri_1 = computeDelaunayTriangulation(p_support);

	// Compute Disparity Planes
	computeDisparityPlanes(p_support,tri_1);

	// allocate memory for disparity grid
	int32_t grid_width   = (int32_t)ceil((float)img_width/(float)grid_size_var);
	int32_t grid_height  = (int32_t)ceil((float)img_height/(float)grid_size_var);
	int32_t grid_dims[3] = {Max_Disp+2,grid_width,grid_height};

	// Find the values that make up PlaneValues - argument to the hardware function
	// Makes use of the Delaunay Triangulation for the planes.
	findPlaneCentres(p_support,tri_1,grid_dims,PlaneCentres, img_width, img_height);

	return;
}


// Creating the "Grid Vectors" used to provide legal disparity evaluations in Dense Matching Stage
void extractSupport (int16_t *Supp_Pnts, vector<support_pt> &p_support, unsigned long long *ValidDispsTop, unsigned long long *ValidDispsBot, int DS){
	int downsmp = DS;
	int k = downsmp;

	int index = 0;
	int32_t currentval;
	for (int i = 0; i < rows; i++){
		for (int j = 0; j < cols; j++){
			currentval = int32_t(*(Supp_Pnts + index++));
			if(currentval >=0) {


				int32_t currentDispGridx = j/grid_size_var;
				int32_t currentDispGridy = i/grid_size_var;

				// Because you can only, at max, send 64 bit data through the DMA port, the 128 bit disparity grid vector
				// needs to be broken apart into two separate 64 bit vectors. This clashes with LUT method for values at boundaries of vectors
				if (currentval > 64) ValidDispsTop[currentDispGridy*grid_width_var + currentDispGridx] |= LUTGrid[currentval-64];
				else if (currentval == 64) {
					ValidDispsTop[currentDispGridy*grid_width_var + currentDispGridx] |= (unsigned long long)(2LL);
					ValidDispsBot[currentDispGridy*grid_width_var + currentDispGridx] |= (unsigned long long)(9223372036854775808LL);
				}
				else if (currentval == 63) {
					ValidDispsTop[currentDispGridy*grid_width_var + currentDispGridx] |= (unsigned long long)(1LL);
					ValidDispsBot[currentDispGridy*grid_width_var + currentDispGridx] |= (unsigned long long)(13835058055282163712LL);
				}
				else ValidDispsBot[currentDispGridy*grid_width_var + currentDispGridx] |= LUTGrid[currentval];

				if (k == downsmp){
					p_support.push_back(support_pt(j,i,currentval));	//Inverted compared to original
					k = 0;
				} else k++;
			}
		}
	}
}


vector<triangle> computeDelaunayTriangulation (vector<support_pt> p_support) {

	// input/output structure for triangulation
    struct triangulateio in, out;
    int32_t k;

    // inputs
    in.numberofpoints = p_support.size();
    in.pointlist = (float*)malloc(in.numberofpoints*2*sizeof(float));
    k=0;

    for (int32_t i=0; i<p_support.size(); i++) {
    	in.pointlist[k++] = p_support[i].u;
    	in.pointlist[k++] = p_support[i].v;
    }

    in.numberofpointattributes = 0;
    in.pointattributelist      = NULL;
    in.pointmarkerlist         = NULL;
    in.numberofsegments        = 0;
    in.numberofholes           = 0;
    in.numberofregions         = 0;
    in.regionlist              = NULL;

    // outputs
    out.pointlist              = NULL;
    out.pointattributelist     = NULL;
    out.pointmarkerlist        = NULL;
    out.trianglelist           = NULL;
    out.triangleattributelist  = NULL;
    out.neighborlist           = NULL;
    out.segmentlist            = NULL;
    out.segmentmarkerlist      = NULL;
    out.edgelist               = NULL;
    out.edgemarkerlist         = NULL;

    // do triangulation (z=zero-based, n=neighbors, Q=quiet, B=no boundary markers)
    char parameters[] = "zQB";
    triangulate(parameters, &in, &out, NULL);

    // put resulting triangles into vector tri
    vector<triangle> tri;
    k=0;
    for (int32_t i=0; i<out.numberoftriangles; i++) {
      tri.push_back(triangle(out.trianglelist[k],out.trianglelist[k+1],out.trianglelist[k+2]));
      k+=3;
    }

    // free memory used for triangulation
    free(in.pointlist);
    free(out.pointlist);
    free(out.trianglelist);

    // return triangles
    return tri;
  }

void computeDisparityPlanes (vector<support_pt> p_support,vector<triangle> &tri) {

	// init matrices
	Matrix A(3,3);
	Matrix b(3,1);

	// for all triangles do
	for (int32_t i=0; i<tri.size(); i++) {

		// get triangle corner indices
		int32_t c1 = tri[i].c1;
		int32_t c2 = tri[i].c2;
		int32_t c3 = tri[i].c3;

		// compute matrix A for linear system of left triangle
		A.val[0][0] = p_support[c1].u;
		A.val[1][0] = p_support[c2].u;
		A.val[2][0] = p_support[c3].u;
		A.val[0][1] = p_support[c1].v; A.val[0][2] = 1;
		A.val[1][1] = p_support[c2].v; A.val[1][2] = 1;
		A.val[2][1] = p_support[c3].v; A.val[2][2] = 1;

		// compute vector b for linear system (containing the disparities)
		b.val[0][0] = p_support[c1].d;
		b.val[1][0] = p_support[c2].d;
		b.val[2][0] = p_support[c3].d;

		// on success of gauss jordan elimination
		if (b.solve(A)) {

		// grab results from b
		tri[i].t1a = b.val[0][0];
		tri[i].t1b = b.val[1][0];
		tri[i].t1c = b.val[2][0];

		// otherwise: invalid
		} else {
		tri[i].t1a = 0;
		tri[i].t1b = 0;
		tri[i].t1c = 0;
		}

		// compute matrix A for linear system of right triangle
		A.val[0][0] = p_support[c1].u-p_support[c1].d;
		A.val[1][0] = p_support[c2].u-p_support[c2].d;
		A.val[2][0] = p_support[c3].u-p_support[c3].d;
		A.val[0][1] = p_support[c1].v; A.val[0][2] = 1;
		A.val[1][1] = p_support[c2].v; A.val[1][2] = 1;
		A.val[2][1] = p_support[c3].v; A.val[2][2] = 1;

		// compute vector b for linear system (containing the disparities)
		b.val[0][0] = p_support[c1].d;
		b.val[1][0] = p_support[c2].d;
		b.val[2][0] = p_support[c3].d;

		// on success of gauss jordan elimination
		if (b.solve(A)) {

		// grab results from b
		tri[i].t2a = b.val[0][0];
		tri[i].t2b = b.val[1][0];
		tri[i].t2c = b.val[2][0];

		// otherwise: invalid
		} else {
		tri[i].t2a = 0;
		tri[i].t2b = 0;
		tri[i].t2c = 0;
		}
	}

	return;
}



void findPlaneCentres(vector<support_pt> p_support, vector<triangle> tri, int32_t *grid_dims, uint8_t *d_plane, int width, int height) {

  // number of disparities
  const int32_t disp_num  = grid_dims[0]-1;

  // loop variables
  int32_t c1, c2, c3;
  float plane_a,plane_b,plane_c,plane_d;

  // for all triangles do
  for (uint32_t i=0; i<tri.size(); i++) {

    // get plane parameters
    plane_a = tri[i].t1a;
    plane_b = tri[i].t1b;
    plane_c = tri[i].t1c;
    plane_d = tri[i].t2a;

    // triangle corners
    c1 = tri[i].c1;
    c2 = tri[i].c2;
    c3 = tri[i].c3;

    // sort triangle corners wrt. u (ascending)
    float tri_u[3];
    tri_u[0] = p_support[c1].u;
    tri_u[1] = p_support[c2].u;
    tri_u[2] = p_support[c3].u;
    float tri_v[3];
    tri_v[0] = p_support[c1].v;
    tri_v[1] = p_support[c2].v;
    tri_v[2] = p_support[c3].v;

    for (uint32_t j=0; j<3; j++) {
      for (uint32_t k=0; k<j; k++) {
        if (tri_u[k]>tri_u[j]) {
          float tri_u_temp = tri_u[j]; tri_u[j] = tri_u[k]; tri_u[k] = tri_u_temp;
          float tri_v_temp = tri_v[j]; tri_v[j] = tri_v[k]; tri_v[k] = tri_v_temp;
        }
      }
    }

    // rename corners
    float A_u = tri_u[0]; float A_v = tri_v[0];
    float B_u = tri_u[1]; float B_v = tri_v[1];
    float C_u = tri_u[2]; float C_v = tri_v[2];

    // compute straight lines connecting triangle corners
    float AB_a = 0; float AC_a = 0; float BC_a = 0;
    if ((int32_t)(A_u)!=(int32_t)(B_u)) AB_a = (A_v-B_v)/(A_u-B_u);
    if ((int32_t)(A_u)!=(int32_t)(C_u)) AC_a = (A_v-C_v)/(A_u-C_u);
    if ((int32_t)(B_u)!=(int32_t)(C_u)) BC_a = (B_v-C_v)/(B_u-C_u);
    float AB_b = A_v-AB_a*A_u;
    float AC_b = A_v-AC_a*A_u;
    float BC_b = B_v-BC_a*B_u;

    // a plane is only valid if itself and its projection
    // into the other image is not too much slanted
    bool valid = fabs(plane_a)<0.7 && fabs(plane_d)<0.7;

    int32_t tempStore;

    // first part (triangle corner A->B)
    if ((int32_t)(A_u)!=(int32_t)(B_u)) {
    	for (int32_t u=max((int32_t)A_u,0); u<min((int32_t)B_u,width); u++){
    		int32_t v_1 = (uint32_t)(AC_a*(float)u+AC_b);
    		int32_t v_2 = (uint32_t)(AB_a*(float)u+AB_b);
    		for (int32_t v=min(v_1,v_2); v<max(v_1,v_2); v++){
				// compute disparity, min disparity and max disparity of plane prior (if valid)
    			tempStore = (int32_t)(plane_a*(float)u+plane_b*(float)v+plane_c);
//    			if (valid && tempStore >= 0) d_plane[v*width+u]     = (uint8_t)tempStore;
    			if (valid && tempStore >= 0) d_plane[v*width+u]     = (uint8_t)(Max_Disp - tempStore);		//Convert from disparity into a location along the pixel window array
    			else d_plane[v*width+u] = (uint8_t)(disp_num + 10); // 10 is chosen arbitrarily, just to get out of the range
    		}
    	}
    }

    // second part (triangle corner B->C)
    if ((int32_t)(B_u)!=(int32_t)(C_u)) {
    	for (int32_t u=max((int32_t)B_u,0); u<min((int32_t)C_u,width); u++){
    		int32_t v_1 = (uint32_t)(AC_a*(float)u+AC_b);
    		int32_t v_2 = (uint32_t)(BC_a*(float)u+BC_b);
    		for (int32_t v=min(v_1,v_2); v<max(v_1,v_2); v++){
    			// compute disparity, min disparity and max disparity of plane prior (if valid)
				tempStore = (int32_t)(plane_a*(float)u+plane_b*(float)v+plane_c);
//				if (valid && tempStore >= 0) d_plane[v*width+u]     = (uint8_t)tempStore;
				if (valid && tempStore >= 0) d_plane[v*width+u]     = (uint8_t)(Max_Disp - tempStore);
				else d_plane[v*width+u] = (uint8_t)(disp_num + 10); // 10 is chosen arbitrarily, just to get out of the range
    		}
    	}
    }
  }

  return;
}
