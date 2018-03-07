/*
 * ExtMatch.hpp
 *
 *  Created on: Jan 24, 2017
 *      Author: orah
 */

#ifndef EXTMATCHDENSE_H
#define EXTMATCHDENSE_H

#include "Topmain.h"
#include "Tools.hpp"

#pragma SDS data copy(ImageLeft, ImageRight, Result, DisparityMSB, DisparityLSB, PlaneVal)
#pragma SDS data access_pattern(ImageLeft:SEQUENTIAL)
#pragma SDS data access_pattern(ImageRight:SEQUENTIAL)
#pragma SDS data access_pattern(Result:SEQUENTIAL)
#pragma SDS data access_pattern(DisparityMSB:SEQUENTIAL)
#pragma SDS data access_pattern(DisparityLSB:SEQUENTIAL)
#pragma SDS data access_pattern(PlaneVal:SEQUENTIAL)
void ExtMatchDense (ap_uint<8> ImageLeft[rows*cols],     ap_uint<8> ImageRight[rows*cols],
                   ap_uint<64> DisparityMSB[grid_height_var*grid_width_var], ap_uint<64> DisparityLSB[grid_height_var*grid_width_var],
                   ap_uint<8>  PlaneVal[rows*cols]	,   ap_int<16>  Result[rows*cols],
                   ap_uint<8>  PlaneRadius);


#endif /* EXTMATCH_H */
