/*
 * ExtMatch.cpp
 *
 *  Created on: Jan 24, 2017
 *      Author: orah
 */
#include "ExtMatchSparse.hpp"

void ExtMatchSparse(ap_uint<8> ImageLeft[rows*cols], ap_uint<8> ImageRight[rows*cols], ap_int<16>  Result[rows*cols]) {

    ap_uint<8> new_col_Left[2*W_First+1];
    ap_uint<8> new_col_Right[2*W_First+1];
    int y, x;
    int i, j;
    ap_uint<DescLength> TempBitResLeft;
    ap_uint<DescLength> TempBitResRight;

    int currentPixel = 0;

    ap_uint<8>           disp[Max_Disp+1], val[2];
#pragma HLS ARRAY_PARTITION variable=disp dim=1 complete
    ap_uint<16>          totes;
    ap_uint<16>          loc[2];

    ap_uint<DescLength>  Match_Win_Left, XorVal;
    ap_uint<DescLength>	 Match_Win_Right[1+Max_Disp];
#pragma HLS ARRAY_PARTITION variable=Match_Win_Right dim=1 complete
    ap_uint<DescLength>  RightVal;

    int                  currentOut = 0;

    LineBuffer <W_First*2+1,	cols,		    ap_uint<8> >	Buff_Left;
    Window	   <W_First*2+1,	W_First*2+1,	ap_uint<8> >	Win_Left;

    LineBuffer <W_First*2+1,	cols,		    ap_uint<8> >	Buff_Right;
    Window     <W_First*2+1,	W_First*2+1,	ap_uint<8> >	Win_Right;

    // Inside the working area of the calculation
    for(y = 0; y < rows; y++){
        for(x = 0; x < cols; x++){
#pragma HLS PIPELINE

#ifndef __SDSVHLS__
            // Debug info
            printf("row: %d , col: %d \n\r", int(y), int(x));
#endif

            // Collecting pixels
            fillBuffer(Buff_Left,ImageLeft[currentPixel],x); 	  // Fill Left Buffer
            Buff_Left.get_col(new_col_Left,x);					        // Line buffer now has latest pixel value
            fillWindow(Win_Left,new_col_Left);					        // Fill Left Reference Window

            fillBuffer(Buff_Right,ImageRight[currentPixel],x);  // Fill Right Buffer
            Buff_Right.get_col(new_col_Right,x);			          // Line buffer now has latest pixel value
            fillWindow(Win_Right,new_col_Right);				        // Fill Right Reference Window

            currentPixel++;

#ifndef __SDSVHLS__
            // Debug info
            printf("currentPixel: %d \n\r", int(currentPixel));
#endif

            for(i = 0; i < 2*W_First+1; i++){
                for (j = 0; j < 2*W_First+1; j++){
                    TempBitResLeft.set( i*(2*W_First+1) + j, Win_Left.getval(i,j) > Win_Left.getval(W_First,W_First));
                    TempBitResRight.set(i*(2*W_First+1) + j, Win_Right.getval(i,j) > Win_Right.getval(W_First,W_First));
                }
            }

            RightVal = TempBitResRight;
            shiftInsert(Match_Win_Right,RightVal,Max_Disp+1);
            Match_Win_Left = TempBitResLeft;

            for(i = 0; i < Max_Disp+1; i++) {
                XorVal = Match_Win_Left^Match_Win_Right[i];
                disp[i] = my_struct<DescLength>::pop_count(XorVal);
                if (Max_Disp+1 - i > x) disp[i] = 200;
            }
            TwoArrayMin<Max_Disp+1>::getmin(disp,loc,val);

#ifndef __SDSVHLS__
            // Debug info
            printf("min: %d \n\r", int(val[0]));
#endif

            totes = ap_uint<16>((val[1] >> 1) + (val[1] >> 2) + (val[1] >> 3) + (val[1] >> 5));


            // Controls when to output zeroes because calculation outside calculable
            // area and just "scrap" and when to output value of SAD.
            if(y > W_First-1 && !(y == W_First && x < W_First)){
                // Check that the first three pixels don't output 0
                if ( y < 2*W_First || x < Max_Disp + 2*W_First) Result[currentOut++] = ap_int<16>(-Max_Disp);
                //if ( y < 2*W_First || x < 40 + 2*W_First) Result[currentOut++] = ap_int<16>(-Max_Disp);  // This expands the left edge to include more pixels (that don't have full dmax range however)
                else {
                    if (totes  > (ap_uint<16>)val[0])           Result[currentOut++] = ap_int<16> (Max_Disp - loc[0]);
                    else                                        Result[currentOut++] = ap_int<16> (-Max_Disp);
                }
            }
        }
    }

    // Add output for bottom w rows of 0
    // to complete disparity image.
    for(i = 0; i < W_First*cols+W_First; i++){
#pragma HLS PIPELINE
        Result[currentOut++] = ap_int<16> (-Max_Disp);

#ifndef __SDSVHLS__
        // Debug Info
        printf("Here: %d  --- ", int(i));
#endif
    }
}



