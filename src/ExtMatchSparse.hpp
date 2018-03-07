/*
 * ExtMatch.hpp
 *
 *  Created on: Jan 24, 2017
 *      Author: orah
 */

#ifndef EXTMATCHSPARSE_H
#define EXTMATCHSPARSE_H

#include "Topmain.h"
#include "Tools.hpp"

#pragma SDS data copy(ImageLeft, ImageRight, Result)
#pragma SDS data access_pattern(ImageLeft:SEQUENTIAL)
#pragma SDS data access_pattern(ImageRight:SEQUENTIAL)
#pragma SDS data access_pattern(Result:SEQUENTIAL)
void ExtMatchSparse(ap_uint<8> ImageLeft[rows*cols], ap_uint<8> ImageRight[rows*cols], ap_int<16>  Result[rows*cols]);

#endif /* EXTMATCH_H */
