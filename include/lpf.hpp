#include <stdint.h>
#include <math.h>

// Thanks Cornell

// ========================================
// === dds setup 
// =======================================
//
// == bit fixed point 2.14 format ===========
// resolution 2^-14 =  6.1035e-5
// dynamic range is +1.9999/-2.0
typedef signed short s1x14 ;
#define muls1x14(a,b) ((s1x14)((((int32_t)(a))*((int32_t)(b)))>>14)) //multiply two fixed s1x14
#define float_to_s1x14(a) ((s1x14)((a)*16384.0)) // 2^14
#define s1x14_to_float(a) ((float)(a)/16384.0)
#define absfix14(a) abs(a)

class lpf {
    public:
        lpf(float Fs, float F0, float Q);

        //======================================================== 
        // Second order lowpass
        // xx is the current input signal sample
        // returns the current filtered output sample
        // Simplification: b(1)*x(n)+b(2)*x(n-1)+b(3)*x(n-2) = 
        // b(1)*x(n)+2*b(1)*x(n-1)+b(1)*x(n-2) =
        // b(1)* (x(n)+(x(n-1)<<1)+x(n-2))
        s1x14 update(s1x14 xx);
    private:
        // filter design inputs
        float Fs, F0, Q, alpha, w0, BW;
        // parameters for  filter design
        s1x14  b0, b1, b2, a1, a2 ;

        s1x14 yy; 
        //IIR state variables
        s1x14  xn, xn_1, xn_2 ; 
        s1x14  yn_1, yn_2  ;
};
