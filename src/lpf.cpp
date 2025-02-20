#include "lpf.hpp"


lpf::lpf(float Fs, float F0, float Q) {
    if(Q < 0.5) Q = 0.5 ;
    if(Q > 2) Q = 2 ;

    w0 = 2*M_PI*F0/Fs ;
    alpha = sin(w0)/(2*Q) ;
    b0 = float_to_s1x14( (1 - cos(w0))/2/(1 + alpha));
    b1 = float_to_s1x14( (1 - cos(w0))/(1 + alpha));
    b2 = float_to_s1x14( (1 - cos(w0))/2/(1 + alpha));
    a1 = float_to_s1x14( (-2*cos(w0))/(1 + alpha));
    a2 = float_to_s1x14( (1 - alpha)/(1 + alpha));

    this->F0 = F0;
    this->Fs = Fs;
    this->Q = Q;
}

s1x14 lpf::update(s1x14 xx) {
    // sum the 5 terms: yy += xx*coeff 
    // and update the state variables
    // as soon as possible

    this->yy = muls1x14((xx+(this->xn_1<<1)+this->xn_2), this->b0);
    xn_2 = this->xn_1;
    xn_1 = xx;
    yy = this->yy - muls1x14(this->yn_2, this->a2); 
    yn_2 = yn_1;
    yy = this->yy - muls1x14(this->yn_1, this->a1); 
    yn_1 = this->yy;
    return this->yy;
}
