# ELAS_SoC
Source Code for "Real-Time Dense Stereo Matching with ELAS on FPGA Accelerated Embedded Devices"

The code has no dependancy on OpenCV for reading/writing images - nothing needs to be cross-compiled and linked for the ARM CPU. The system expects 8-bit binary PGM images for the L/R inputs and outputs 16-bit binary PGM disparity map.

**Building through SDx/SDSoC GUI:**
1) Open a new project in Xilinx SDx/SDSoC with correct board/chip selected
2) Add source files to project
3) Modify the Topmain.h file to vary the parameters used in the build of the different accelerators.
4) In project settings, set functions "ExtMatchDense", "ExtMatchSparse" and "triage" for hardware acceration
5) Set Clock Frequency of HW functions to desired frequency, set the same frequency value for the "Data Motion Network". Whether a build at a desired clock frequency succeeds may depend on the capabilities & speedgrade of the SoC used.
6) Ensure that "Generate bitsream" & "Generate SD card image" options are selected.

**Testing System**
1) Following the succeful build, navigate to the build directory and copy contents located in the "sd_card" directory onto external {micro, standard} SD card.
2) Similarly, copy input images onto SD card in the fashion expected in Topmain.cpp
3) Insert SD card into development board and boot board.
4) Navigate to /mnt/ directoy and run ./*Project_Name*.elf
