#include "LowPass.h"
//Low pass butterworth filter order=1 alpha1=0.1 
LowPass::LowPass(int s) {
    v[0]=0.0;
    switchpoint = s;
}
int LowPass::step(int x){ //class II 
    v[0] = v[1];
    if( x > switchpoint) {
        v[1] = (1.546629140310340489e-2 * x)
			 + (0.96906741719379319022 * v[0]);
    } else {
        v[1] = (1.367287359973195227e-1 * x)
		     + (0.72654252800536101020 * v[0]);    
    }
    return (int)(v[0] + v[1]);
}
