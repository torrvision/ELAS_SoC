#include "Topmain.h"

using namespace std;

#include <stdlib.h>
#include <sds_lib.h>
#include <unistd.h>

#include "ExtMatchSparse.hpp"
#include "ExtMatchDense.hpp"
#include "Triage.hpp"
#include "ARMcode.h"


#define			ipx_type		ap_uint<8>

int main () {

    int i, j;
    int retval = 0;

    printf("Starting... \n\r");

    // Open and create necessary files
    FILE *result_file;
    FILE *sparse_result;
    FILE *LeftIm_file;
    FILE *RightIm_file;


    // Allocate space in memory for Image files to be stored
    ipx_type *ImageL;      ImageL 		= (ipx_type *)(sds_alloc_non_cacheable(rows*cols*sizeof(ipx_type)));
    ipx_type *ImageR;      ImageR 		= (ipx_type *)(sds_alloc_non_cacheable(rows*cols*sizeof(ipx_type)));
    opx_type *Result;      Result 		= (opx_type *)(sds_alloc(rows*cols*sizeof(opx_type)));
    opx_type *FinalResult; FinalResult  = (opx_type *)(sds_alloc_non_cacheable(rows*cols*sizeof(opx_type)));

    for (i = 0; i < rows*cols; i++){
        ImageL[i] 	  	= (ipx_type)0;
        ImageR[i] 	  	= (ipx_type)0;
        Result[i] 	  	= (opx_type)0;
        FinalResult[i] 	= (opx_type)0;
    }

    printf("Start Allocating Untriaged Result Array! \r\n");
    opx_type	*untriagedResult; untriagedResult = (opx_type *)(sds_alloc(rows*cols*sizeof(opx_type)));

    printf("Start Allocating Disparity Array! \r\n"); //For the Sparse Feature Matching iteration of the block
    unsigned long long	*DisparityMSB;  DisparityMSB = (unsigned long long *)(sds_alloc(grid_height_var*grid_width_var*sizeof(mpx_type)));
    unsigned long long	*DisparityLSB;  DisparityLSB = (unsigned long long *)(sds_alloc(grid_height_var*grid_width_var*sizeof(mpx_type)));
    ap_uint<8> 	*PlaneVal;     			    PlaneVal     = (ap_uint<8> *)(sds_alloc(rows*cols*sizeof(ap_uint<8>)));

#define PathLengthInput 100
    char ImageLPath[PathLengthInput];
    char ImageRPath[PathLengthInput];
    char SparsePath[PathLengthInput];
    char FinalsPath[PathLengthInput];
    char FinalsDir[PathLengthInput];



#define avg_cpu_cycles()(total_run_time / num_calls)
#define NUM_TESTS 155//1024
    unsigned long long total_run_time = 0;
    unsigned int num_calls = 0;
    unsigned long long count_val = 0;
    unsigned long long tmp = 0;

    unsigned long long part_full_RT = 0;
    unsigned long long part_ARM_RT = 0;


    unsigned long long prearm = 0;
    unsigned long long postarm = 0;
    unsigned long long totalarm = 0;

    double divided = 0;
    double divided2 = 0;

    int DS [] = {0,1,2,4,8,16,32,64,128,256,512}; // The different quantities of numbers of DownSampling used

    printf("Start! \r\n");
    for (int taco = 0; taco < sizeof(DS)/sizeof(DS[0]); taco++){
        for (int i = 0; i < NUM_TESTS; i++) {
            for(int j = 0; j < 2; j++){


                sprintf(ImageRPath, "/mnt/training/image_3/000%03d_1%d.pgm", i, j);
                sprintf(SparsePath, "/mnt/SparseResults/%d/000%03d_1%d.pgm", DS[taco], i, j);
                sprintf(FinalsPath, "/mnt/FinalResults/%d/000%03d_1%d.pgm", DS[taco], i, j);
                sprintf(ImageLPath, "/mnt/training/image_2/000%03d_1%d.pgm", i, j);

                sprintf(FinalsDir, "mkdir -p /mnt/FinalResults/%d", DS[taco]);
                system(FinalsDir);

                LeftIm_file 	= fopen(ImageLPath,"rb");   if (LeftIm_file == NULL) printf("Error Left \r\n");
                RightIm_file	= fopen(ImageRPath,"rb");   if (RightIm_file == NULL) printf("Error Right \r\n");

                // Loading both input images manually, without need for OpenCV
                char header[18];
                fread(header, sizeof(char), 16, LeftIm_file);
                fread(&ImageL[0], sizeof(ipx_type), rows*cols, LeftIm_file);	// Loading Image Left
                fread(header, sizeof(char), 16, RightIm_file);
                fread(&ImageR[0], sizeof(ipx_type), rows*cols, RightIm_file);	// Loading Image Right
                fclose(LeftIm_file); fclose(RightIm_file);

                printf("DS(%02d) - Image %03d_%d0 \r\n", DS[taco], i, j);

                count_val = sds_clock_counter();  //Timer
                ExtMatchSparse(ImageL, ImageR, untriagedResult);
                triage(untriagedResult, Result);
                prearm = sds_clock_counter();     //Timer
                armcode((int16_t*)Result,DisparityMSB, DisparityLSB,(uint8_t*)(PlaneVal), DS[taco]);
                postarm = sds_clock_counter();    //Timer
                ExtMatchDense(ImageL, ImageR, (mpx_type*)DisparityMSB, (mpx_type*)DisparityLSB, PlaneVal, FinalResult, 4);
                tmp = sds_clock_counter();        //Timer

                if(i == NUM_TESTS-1 && j == 1) printf("Done! \r\n");

                total_run_time += (tmp - count_val);
                totalarm += (postarm - prearm);

                part_full_RT += (tmp - count_val);
                part_ARM_RT  += (postarm - prearm);

                // Saving final image without OpenCV in PGM format.
                char buffer[18];
                result_file 	= fopen(FinalsPath,"w");    if (result_file == NULL) printf("Error Final Image \r\n");
                sprintf(buffer,"P5 %d %d %d\r", cols, rows, 65535);
                fwrite(buffer, sizeof(char), 18, result_file);
                buffer[0] = (char)0;
                fwrite(buffer, sizeof(char), 1, result_file);
                for (int t = 0; t < rows; t++) {
                    fwrite(&FinalResult[t*cols], sizeof(opx_type),cols,result_file);
                }
                fclose(result_file);
            }
        }

        divided = ((double)part_full_RT)/((double)NUM_TESTS*2); // Average number of clock cycles per image pair
        double ms_of_total = divided*1000/800000000;            // Converting to time. This depends on the frequency of the onboard ARM CPU
        divided2 = double(part_ARM_RT)/double(2*NUM_TESTS);     // Isolating the clocks/time of the ARM CPU
        divided2 = divided2/800000;                             // 800000 is characteristic of the ARM processor on the board
        printf("DownSampling of %d:    %f ms Overhead    %f FPS AVG    %f ms    %f ms (ARM)    %f ms (FPGA)    %dx%d    %d    %d    %d\n\r", DS[taco], (divided - 1242*375*4)*1000/800000000, (double)800000000/divided, ms_of_total, divided2, ms_of_total-divided2, cols, rows, W_First, W_Second, Max_Disp);

        part_full_RT = 0;
        part_ARM_RT  = 0;


    }

    sds_free(ImageL); sds_free(ImageR); sds_free(Result); sds_free(FinalResult);
    sds_free(untriagedResult);
    sds_free(DisparityLSB); sds_free(DisparityMSB); sds_free(PlaneVal);

    // This code below (commented out) is somewhat meaningless - It just counts
    // the entire time with aggregating the whole time spent processing for the
    // different amounts of downsampling

    // printf("Total time for %d images:  %llu\n\r",(int)NUM_TESTS, total_run_time);
    // divided = ((double)total_run_time)/((double)NUM_TESTS*2)/9;
    // printf("Average processing time per image pair  %f, which means %f FPS at this resolution \n\r", divided, (double)800000000/divided);
    // printf("Time spent total  %llu, of which %llu time spent in arm --- this is %f percent \n\r", total_run_time, totalarm, 100*(double(totalarm)/double(total_run_time)));
    // printf("Info of Run \n\r Resolution: %d x %d \n\r Window Size: %d x %d \n\r Disparity Range: %d \n\r", cols, rows, W_First, W_Second, Max_Disp);


    if (retval == 0) printf("\n\r Test PASSED! \n\r");
    return retval;
}
