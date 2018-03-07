#include "Topmain.h"
#include "Triage.hpp"

///   MODIFICATION TO ELAS
// After considering the pruning technique used in ELAS, we have come to the conclusion that looking forward
// into future values as well as previous ones then it will continuously propagate redundancy without occasionally
// inserting values to maintain some of the information in the image.
// 				Example:
//								Pixels	:	1 1 1 1 1 1 1 2 2 2 2 2 2 2 1 1 1
//Elas with 5 Window [2 back / 2 front] :	1 1 1 1 1 1 0 1 1 1 1 1 1 0 1 1 0  	-----> Only 2-3 points at the end of a list of similar values
//Mine with 5 Window [4 back / 0 front] :	0 1 1 1 1 0 1 0 1 1 1 1 0 1 0 1 1	-----> 4-5 points and you get occasional reminder of value

// There is two ways to tackle this. The first is to have the redundancy calculated ahead of the support and kept
// separately in a bit buffer that is accessed at the end of the support calculation to decided whether to output it
// or not

// The second method is to have both redundancy and support determined at the same time. The input pixel is still at the
// bottom right of the window, but you have a larger buffer and window to support the redundancy. For this you would need
// buffer of 2*win+1, cols that is fed in by middle pixel of support window. An additional 2*win+1 by cols uint<1> buffer
// to store redundancy. -----> Stopping description here as this method clearly occupies more than the first so we will
// stick with that.


// One issue with my own implementation is that it becomes diagonal lines, that are indeed satisfying the constraints, but may still
// to abundant in the worst case. i.e.
/*
 *             *    *    *    *    *
 *              *    *    *    *    *
 *               *    *    *    *    *
 *                *    *    *    *    *
 *                 *    *    *    *    *
 *                  *    *    *    *    *
 *                   *    *    *    *    *
 *                    *    *    *    *    *
 *                     *    *    *    *    *
 *                      *    *    *    *    *
 *                       *    *    *    *    *
 */


void triage (ap_int<16> InputDisps[rows*cols], ap_int<16> TriageResult[rows*cols]) {
    
    int y, x;
    int i, j;
    int redun[2*win],redun2[2*win], supported;
    int redunsum, redunsum2;


    ap_int<16> tempstore,tempstore2,tempstore3;     // tempvariables used in redundancy/support check
    int        tempstore4, tempstore5;              // int used vs ap_int<16> only to see if difference in end-implementation

    ap_int<16> input_px, support_px, window_R_Col_value, window_Bot_Row_value, window_val;


    LineBuffer <2*win+1, cols, ap_int<16> >	Buff;              // Pixel Value Storage (part-accessible)

    ap_int<16>                              new_col[2*win+1];  // Used to get values from Buff and fill Window with
    Window <2*win+1, win*2+1, ap_int<16> >  Window;            // Stores instantaneously accessible pixel values

    LineBuffer <2*win, cols, ap_uint<1> >   RedunBuff;			   // Retains longer term storage of calculated redundancy (for column redundancy)
    ap_uint<1>                              Redun_Col[2*win];
    ap_uint<1>                              PrevRowRedun[2*win];// Retains whether previous value was marked row-redundant
    ap_uint<1>                              SupportRedun[win+1];

    int                                     currentIn = 0;
    int                                     currentOut= 0;

 Rows:for(y = 0; y < rows; y++){
    Columns:for(x = 0; x < cols; x++){
#pragma HLS PIPELINE

            // Collecting Left (reference window) pixels
            input_px = InputDisps[currentIn++];	// Use this for redundancy
            fillBuffer(Buff,input_px,x); 			  // Fill Buffer
            Buff.get_col(new_col,x);				    // Line buffer now has latest pixel value
            fillWindow(Window,new_col);				  // Fill Left Reference Window


            RedunBuff.get_col(Redun_Col,x);
            // Checking rightmost column and bottom-most row for redundancy
            // Backwards looking ONLY.
            for(i = 0; i < 2*win; i++){
                window_R_Col_value 		= Window.getval(i,2*win);
                window_Bot_Row_value 	= Window.getval(2*win,i);
                tempstore 	= absdiff_cus(window_R_Col_value, input_px);
                tempstore5 	= absdiff_cus(window_Bot_Row_value.to_int(), input_px.to_int());  // int-cast vs no int-cast to see if speed differences exist
                if (tempstore < 1)  redun[i]  = !Redun_Col[i];    else redun[i]  = 0;         // If value is too close, set redundant unless said value is
                if (tempstore5 < 1) redun2[i] = !PrevRowRedun[i]; else redun2[i] = 0;         // itself already previously set to redundant
            }


            // Count up the number of similar values in both the row and column that are being
            // filtered for redundancy
            redunsum = redunsum2 = 0;
            for (i = 0; i < 2*win; i++){
                redunsum+=redun[i];
                redunsum2+=redun2[i];
            }
            // Store the result of whether a value was deemed redudant or not
            // for output and calculations of subsequent pixels' redundancies
            if (redunsum >= redun_thresh || redunsum2 >= redun_thresh ) {
                shiftInsert(PrevRowRedun,ap_uint<1>(1),2*win);
                fillBuffer(RedunBuff, ap_uint<1>(1), x);
            } else {
                shiftInsert(PrevRowRedun,ap_uint<1>(0),2*win);
                fillBuffer(RedunBuff,ap_uint<1>(0),x);
            }

            support_px = Window.getval(win,win);		        // Use this value for support check
            shiftInsert(SupportRedun,Redun_Col[win],win+1); // Load in the redundancy value of the support point.
            supported = 0;
            // Iterate over the window region and check that enough similar values exist
            for (i = 0; i < 2*win+1; i++){
                for (j = 0; j < 2*win+1; j++){
                    window_val = Window.getval(i,j);
                    tempstore2 = absdiff_cus(window_val, support_px);
                    tempstore4 = absdiff_cus(window_val.to_int(), support_px.to_int()); // Unused, should be trimmed - can replace to compare implementation differences.
                    if ( tempstore2 <= incon_thresh) supported++;
                }
            }


            // Controls when to output zeroes because calculation outside calculable
            // area and just "scrap" and when to output value of SAD.
            if(y > win-1 && !(y == win && x < win)){
                // Check that the first three pixels don't output 0
                if ( x < 2*win || y < 2*win) {
                    // There exists a minor bug here for the first 5 inside calculabe region pixels of every row
                    // as the prevredundbuffer extends into the end of the previous row.
                    TriageResult[currentOut++] = ap_int<16>(-in_d);
                }
                else {
                    if (SupportRedun[0] == 1 || supported < min_support){
                        TriageResult[currentOut++] = ap_int<16>(-in_d);
                        // printf("This is supported, but redundant -- NEGATIVE \n\r");
                    }
                    else {
                        TriageResult[currentOut++] = ap_int<16> (support_px);
                        // printf("This is supported and not redundant -- POSITIVE \n\r");
                    }
                }
            }
        }
    }

// Add output for bottom w rows of 0
// to complete disparity image.
Zeroes:for(i = 0; i < win*cols+win; i++){
#pragma HLS PIPELINE
        TriageResult[currentOut++] = ap_int<16>(-in_d);
    }
}
