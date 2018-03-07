#ifndef TOPMAIN_H
#define TOPMAIN_H

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>

#include <ap_int.h>
#include <assert.h>

// Common
#define 	cols			1242   // Image resolution only limited by the constraints on sds_alloc which is < 4,190,208 bytes
#define		rows			375    // Image resolution only limited by the constraints on sds_alloc which is < 4,190,208 bytes
#define		Max_Disp	127    // Possible value range: 64 - 127


// Support Point Extractor
#define		W_First			  4 // Corresponds to one arm length of a Window, full window is (2*(W_First)+1). Therefore 9 x 9 Windows with value 4
#define		desc_half		  2*W_First*(W_First+1)
#define 	DescLength		2*desc_half+1  // Length of corresponding Census Transform Descriptor


// Dense Depth Estimation
#define		W_Second		  2 // Corresponds to one arm length of a Window, full window is (2*(W_Second)+1). Therefore 5 x 5 Windows with value 2
#define		desc_half_2		2*W_Second*(W_Second+1)
#define 	DescLength_2	2*desc_half_2+1  // Length of corresponding Census Transform Descriptor


//Support Point Filtering Parameters
#define 	in_d			    Max_Disp
#define 	win				    4        // Used for both redundancy (win back in row & column) and support (in a window of 2*win+1)
#define		redun_thresh	1        // Number of repetitions before deemed redundant
#define		incon_thresh	5        // Threhold to be deemed similar enough to add to support criteria
#define		min_support		10       // Minimum required number of points to be found to qualify as "supported" value

// ARM Code
#define		opx_type		ap_int<16>
#define		mpx_type		ap_uint<64>
#define		img_height	rows
#define		img_width		cols

  // Size for Grid Construction (aggregating possible disparity indexes)
#define		grid_size_var	  20
#define 	grid_width_var	(cols/grid_size_var + ((cols%grid_size_var>0)?1:0))
#define 	grid_height_var	(rows/grid_size_var + ((rows%grid_size_var>0)?1:0))

  // LUT Values
static unsigned long long LUTDisps[64] = {1LL,2LL,4LL,8LL,16LL,32LL,64LL,128LL,256LL,512LL,1024LL,2048LL,4096LL,8192LL,16384LL,32768LL,65536LL,
		131072LL,262144LL,524288LL,1048576LL,2097152LL,4194304LL,8388608LL,16777216LL,33554432LL,67108864LL,134217728LL,268435456LL,536870912LL,
		1073741824LL,2147483648LL,4294967296LL,8589934592LL,17179869184LL,34359738368LL,68719476736LL,137438953472LL,274877906944LL,549755813888LL,
		1099511627776LL,2199023255552LL,4398046511104LL,8796093022208LL,17592186044416LL,35184372088832LL,70368744177664LL,140737488355328LL,281474976710656LL,
		562949953421312LL,1125899906842624LL,2251799813685248LL,4503599627370496LL,9007199254740992LL,18014398509481984LL,36028797018963968LL,72057594037927936LL,
		144115188075855872LL,288230376151711744LL,576460752303423488LL,1152921504606846976LL,2305843009213693952LL,4611686018427387904LL,9223372036854775808LL};  //, 18446744073709551616};

#endif /* SRC_TOPMAIN_H_ */
