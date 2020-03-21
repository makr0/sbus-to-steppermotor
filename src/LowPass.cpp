#include "LowPass.h"
//Low pass butterworth filter order=1 alpha1=0.1 
LowPass::LowPass(int s) {
    v[0]=0.0;
    switchpoint = s;
    fastResponse = true;
}
void LowPass::setResponse(bool fast){
    fastResponse = fast;
} //class II 
int LowPass::step(int x){ //class II 
    v[0] = v[1];
    if(fastResponse) {
        v[1] = (1.3e-1 * x)
		     + (0.72 * v[0]);
    } else {
        v[1] = (6.244035046342855111e-3 * x)
			 + (0.98751192990731428978 * v[0]);
    }
    return (int)(v[0] + v[1]);
}
