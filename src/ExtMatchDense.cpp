#include "ExtMatchDense.hpp"

void ExtMatchDense (ap_uint<8> ImageLeft[rows*cols],     ap_uint<8> ImageRight[rows*cols],
                   ap_uint<64> DisparityMSB[grid_height_var*grid_width_var], ap_uint<64> DisparityLSB[grid_height_var*grid_width_var],
                   ap_uint<8>  PlaneVal[rows*cols]	,   ap_int<16>  Result[rows*cols],
                   ap_uint<8>  PlaneRadius){

	  int y, x, i, j;
    ap_uint<8> new_col_Left[2*W_Second+1];
    ap_uint<8> new_col_Right[2*W_Second+1];
    ap_uint<DescLength_2> TempBitResLeft;
    ap_uint<DescLength_2> TempBitResRight;
    int currentPixel = 0;

    ap_uint<8>           disp[Max_Disp+1], val;
    ap_uint<16>          loc;
#pragma HLS ARRAY_PARTITION variable=disp dim=1 complete


    ap_uint<DescLength_2>  Match_Win_Left, XorVal;
    ap_uint<DescLength_2>  Match_Win_Right[1+Max_Disp];
#pragma HLS ARRAY_PARTITION variable=Match_Win_Right dim=1 complete

    int                  currentOut = 0;

    ap_uint<Max_Disp-63> TopDisps;
    ap_uint<64>          BotDisps;
    ap_uint<Max_Disp+1>  ConcatDisps;
    ap_uint<8>           PlaneValCurr;
    int                  currentIndex = 0;

    // Reward for lying on pre-calculated plane prior
    static const ap_uint<8>			P[] = {    1  *(2*W_Second+1)*(2*W_Second+1)/25,
                                           3  *(2*W_Second+1)*(2*W_Second+1)/25,
                                           7  *(2*W_Second+1)*(2*W_Second+1)/25,
                                           11 *(2*W_Second+1)*(2*W_Second+1)/25,
                                           16 *(2*W_Second+1)*(2*W_Second+1)/25,
                                           11 *(2*W_Second+1)*(2*W_Second+1)/25,
                                           7  *(2*W_Second+1)*(2*W_Second+1)/25,
                                           3  *(2*W_Second+1)*(2*W_Second+1)/25,
                                           1  *(2*W_Second+1)*(2*W_Second+1)/25};

    int                  PIndex = 0;


    // Linebuffer and window buffers for both L/R images.
    LineBuffer <W_Second*2+1,	cols,		    ap_uint<8> >	Buff_Left;
    Window	   <W_Second*2+1,	W_Second*2+1,	ap_uint<8> >	Win_Left;

    LineBuffer <W_Second*2+1,	cols,		    ap_uint<8> >	Buff_Right;
    Window     <W_Second*2+1,	W_Second*2+1,	ap_uint<8> >	Win_Right;

    ap_uint<Max_Disp+1> DispsStore[(int)grid_width_var];
#pragma HLS ARRAY_PARTITION variable=DispsStore dim=1 complete
    int currentDispGrid = 0;
    ap_uint<Max_Disp+1>  currentDisp = 0;
    int                  currentDispIdx = 0;


    int x_tip = 0; int y_tip = 0;
    int flag = 0;

// Inside the working area of the calculation
    for(y = 0; y < rows; y++){
        for(x = 0; x < cols; x++){
#pragma HLS PIPELINE

#ifndef __SDSVHLS__
        	// Debug info
        	printf("row: %d , col: %d \n\r", int(y), int(x));
#endif

//          Collecting pixels
            fillBuffer(Buff_Left,ImageLeft[currentPixel],x); 	// Fill Left Buffer
            Buff_Left.get_col(new_col_Left,x);					// Line buffer now has latest pixel value
            fillWindow(Win_Left,new_col_Left);					// Fill Left Reference Window

            fillBuffer(Buff_Right,ImageRight[currentPixel],x);  // Fill Right Buffer
            Buff_Right.get_col(new_col_Right,x);				// Line buffer now has latest pixel value
            fillWindow(Win_Right,new_col_Right);				// Fill Right Reference Window

            if(y > W_Second-1 && !(y == W_Second && x < W_Second)){
                PlaneValCurr = PlaneVal[currentIndex++];

                if (x_tip == cols) { x_tip = 0; y_tip++; }

                // Making sure to read in and get the right "Grid Vector"
                currentDispGrid = x_tip/(int)grid_size_var;
                if (x_tip % (int)grid_size_var == 0){
                  if (y_tip % (int)grid_size_var == 0){
                    TopDisps =     ap_uint<(Max_Disp+1)-64>(((DisparityMSB[currentDispIdx])(Max_Disp-64,0)));
                    BotDisps =     DisparityLSB[currentDispIdx];
                    ConcatDisps     =  TopDisps.concat(BotDisps);
                    DispsStore[currentDispGrid] = ConcatDisps;
                    currentDispIdx++;
                  }
                };
                currentDisp = DispsStore[currentDispGrid]; // This may need to be W higher and W before-in-row - Neglibible effect regardless
                x_tip++;
            }
            currentPixel++;


#ifndef __SDSVHLS__
            // Debug info
        	printf("currentPixel: %d \n\r currentIndex: %d \n\r", int(currentPixel), int(currentIndex));
#endif
        	// Creating the Census Descriptor
            for(i = 0; i < 2*W_Second+1; i++){
                for (j = 0; j < 2*W_Second+1; j++){
                    TempBitResLeft.set( i*(2*W_Second+1) + j, Win_Left.getval(i,j) > Win_Left.getval(W_Second,W_Second));
                    TempBitResRight.set(i*(2*W_Second+1) + j, Win_Right.getval(i,j) > Win_Right.getval(W_Second,W_Second));
                }
            }

            // Storing the Census Descriptor
            shiftInsert(Match_Win_Right,TempBitResRight,Max_Disp+1);
            Match_Win_Left = TempBitResLeft;

            for(i = 0; i < Max_Disp+1; i++) {
                XorVal = Match_Win_Left^Match_Win_Right[i];
                disp[i] = my_struct<DescLength_2>::pop_count(XorVal);

                // Takes care of settling values that are within the radius of the plane
                if (PlaneValCurr != 0 && i >= (PlaneValCurr - PlaneRadius) && (i <= (PlaneValCurr + PlaneRadius))){
                	PIndex = i - PlaneValCurr + PlaneRadius;
                    if (disp[i] < P[PIndex]){
                        disp[i] = (ap_uint<8>)0;
                    } else {
                    	disp[i] = disp[i] - P[PIndex];
                    }
                } else {
                	if (!currentDisp.test(Max_Disp - i) || (Max_Disp - i > x)) {
                		disp[i] = (ap_uint<8>)200;
                	}
                }
            }

            ArrayMin<Max_Disp+1>::getmin(disp,loc,val);

#ifndef __SDSVHLS__
        	printf("min: %d \n\r", int(val));
#endif

            // Controls when to output zeroes because calculation outside calculable
            // area and just "scrap" and when to output value of SAD.
            if(y > W_Second-1 && !(y == W_Second && x < W_Second)){
            // Check that the first three pixels don't ouput 0
                if ( y < 2*W_Second || x < Max_Disp + 2*W_Second) {
//            	if ( y < 2*W_Second || x < 2*W_Second + 40) {    // This + 40 expands vs Max_Disp decreases the amount that is ignored on left edge of final result
                	   Result[currentOut++] = ap_int<16>(0);
                } else Result[currentOut++] = ap_int<16> (Max_Disp - loc);
            }
        }
    }

// Add output for bottom w rows of 0
// to complete disparity image.
    for(i = 0; i < W_Second*cols+W_Second; i++){
#pragma HLS PIPELINE

        Result[currentOut++] = ap_int<16> (0);
        PlaneValCurr = PlaneVal[currentIndex++];
        if (currentDispIdx < grid_height_var*grid_width_var){
          TopDisps =     ap_uint<(Max_Disp+1)-64>(((DisparityMSB[currentDispIdx])(Max_Disp-64,0)));
          BotDisps =     DisparityLSB[currentDispIdx];
          DispsStore[currentDispGrid] = TopDisps.concat(BotDisps);
        	currentDispIdx++;
        }

#ifndef __SDSVHLS__
        	// Debug info
        	printf("Here: %d  --- ", int(i));
        	printf("currentIndex: %d \n\r", int(currentIndex));
#endif


    }
}



