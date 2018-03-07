#ifndef TOOLS_H
#define TOOLS_H

#include "cust_smem.h"

// Absolute Difference
template<typename T>
T absdiff_cus(T a, T b) {
#pragma HLS inline
    long long x = a-b;
    long long y = b-a;
    T r;
    if(x < 0) { // signbit check.
        r = y; assert(r == (T)y);
    } else {
        r = x; assert(r == (T)x);
    }
    return r;
}

// Function to fill appropriate line buffer value by value
template <typename T, int ROWS, int COLS>
void fillBuffer(LineBuffer<ROWS,COLS,T> &Buffer, T px, int idx ){
#pragma HLS inline
    Buffer.shift_pixels_up(idx);
    Buffer.insert_bottom_row(px,idx);
}

// Function to fill appropriate window buffer column by column
template <typename T, int ROWS, int COLS>
void fillWindow(Window<ROWS,COLS,T> &Window, T new_col[ROWS]){
#pragma HLS inline
    Window.shift_pixels_left();
    Window.insert_col(new_col,COLS-1);
}

template <typename T>
void shiftInsert(T arr[], T newvalue, int SIZE){
#pragma HLS inline
    for (int i = 0; i < SIZE-1; i++){
#pragma HLS unroll
        arr[i] = arr[i+1];
    }
    arr[SIZE-1] = newvalue;
}


#define CUTOFF 7

template<int BITWIDTH, bool small=BITWIDTH<=CUTOFF> struct my_struct {
    static ap_uint<8> pop_count(ap_uint<BITWIDTH> x);
};

template<int BITWIDTH> struct my_struct<BITWIDTH,true> {
    static inline ap_uint<3> pop_count(ap_uint<BITWIDTH> x) {
    	assert(CUTOFF<=7 && "need CUTOFF <= 7 or need to make the LUT larger");
   static ap_uint<3> table[128] = {
		0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
		1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
		1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
		2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7};
      return table[x];
    }
};

template<int BITWIDTH> struct my_struct<BITWIDTH,false> {
    static ap_uint<8> pop_count(ap_uint<BITWIDTH> x) {
        // trying improve the above by cutting in the middle to get a binary tree adder shape
        const int middle = BITWIDTH > 2*CUTOFF ? (BITWIDTH/2/CUTOFF)*CUTOFF : CUTOFF;
        return
            my_struct<middle>::pop_count(x(middle-1,0)) +
            my_struct<BITWIDTH-middle>::pop_count(x(BITWIDTH-1,middle));
    }
};



// Find First and Second minimum in tree like fashion
template<int SIZE>
class TwoArrayMin {
public:
    template <typename T, typename T_idx>
    static void getmin(T a[SIZE], T_idx loc[2], T val[2]) {
#pragma HLS inline
#pragma HLS array_partition variable=a complete dim=0
        T a1[SIZE/2];
        T a2[SIZE-SIZE/2];
        for(int i = 0; i < SIZE/2; i++) {
            a1[i] = a[i];
        }
        for(int i = 0; i < SIZE-SIZE/2; i++) {
            a2[i] = a[i+SIZE/2];
        }
        T_idx l1[2];
        T v1[2];
        T_idx l2[2];
        T v2[2];
        TwoArrayMin<SIZE/2>::getmin(a1,l1,v1);
        TwoArrayMin<SIZE-SIZE/2>::getmin(a2,l2,v2);
        if(v1[0] >= v2[0]) {
            val[0] = v2[0];
            loc[0] = l2[0]+SIZE/2;
            if (v1[0] >= v2[1]) {
                val[1] = v2[1];
                loc[1] = l2[1]+SIZE/2;
            } else {
            	val[1] = v1[0];
            	loc[1] = l1[0];
            }
        } else {
            val[0] = v1[0];
            loc[0] = l1[0];
            if (v1[1] >= v2[0]){
            	val[1] = v2[0];
            	loc[1] = l2[0]+SIZE/2;
            } else{
            	val[1] = v1[1];
            	loc[1] = l1[1];
            }
        }
    }
};

template<>
class TwoArrayMin<2> {
public:
    template <typename T, typename T_idx>
    static void getmin(T a[2], T_idx loc[2], T val[2]) {
#pragma HLS inline
#pragma HLS array_partition variable=a complete dim=0
        T_idx l1=0, l2=1;
        T v1=a[0], v2=a[1];
        if(v1 >= v2) {
            val[0] = v2;
            loc[0] = l2;
            val[1] = v1;
            loc[1] = l1;
        } else {
            val[0] = v1;
            loc[0] = l1;
            val[1] = v2;
			loc[1] = l2;
        }
    }
};

template<>
class TwoArrayMin<1> {
public:
    template <typename T, typename T_idx>
    static void getmin(T a[1], T_idx loc[2], T val[2]) {
#pragma HLS inline
#pragma HLS array_partition variable=a complete dim=0
        loc[0] = 0;
        val[0] = a[0];
        loc[1] = (T_idx)600;
        val[1] = (T)31;
    }
};



template<int SIZE>
class ArrayMin {
public:
    template <typename T, typename T_idx>
    static void getmin(T a[SIZE], T_idx &loc, T &val) {
#pragma HLS inline
#pragma HLS array_partition variable=a complete dim=0
        T a1[SIZE/2];
        T a2[SIZE-SIZE/2];
        for(int i = 0; i < SIZE/2; i++) {
            a1[i] = a[i];
        }
        for(int i = 0; i < SIZE-SIZE/2; i++) {
            a2[i] = a[i+SIZE/2];
        }
        T_idx l1,l2;
        T v1,v2;
        ArrayMin<SIZE/2>::getmin(a1,l1,v1);
        ArrayMin<SIZE-SIZE/2>::getmin(a2,l2,v2);
        if(v1 >= v2) {
            val = v2;
            loc = l2+SIZE/2;
        } else {
            val = v1;
            loc = l1;
        }
//        printf("Climbing Loc %d \n\r", (int)loc);
    }
};

template<>
class ArrayMin<2> {
public:
    template <typename T, typename T_idx>
    static void getmin(T a[2], T_idx &loc, T &val) {
#pragma HLS inline
#pragma HLS array_partition variable=a complete dim=0
        T_idx l1=0, l2=1;
        T v1=a[0], v2=a[1];
        if(v1 >= v2) {
            val = v2;
            loc = l2;
        } else {
            val = v1;
            loc = l1;
        }
    }
};

template<>
class ArrayMin<1> {
public:
    template <typename T, typename T_idx>
    static void getmin(T a[1], T_idx &loc, T &val) {
#pragma HLS inline
#pragma HLS array_partition variable=a complete dim=0
        loc = 0;
        val = a[0];
    }
};

#endif
