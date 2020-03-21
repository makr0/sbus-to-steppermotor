#include "LowPass.h"
//Low pass butterworth filter order=1 alpha1=0.1 
LowPass::LowPass() {
    v[0]=0.0;
}

int LowPass::step(int x){ //class II 
    v[0] = v[1];
    v[1] = (1.3e-1 * x)
            + (0.72 * v[0]);
    return (int)(v[0] + v[1]);
}
