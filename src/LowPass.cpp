#include "LowPass.h"
//Low pass butterworth filter order=1 alpha1=0.1 
LowPass::LowPass() {
    v[0]=0.0;
}
int LowPass::step(int x){ //class II 
    v[0] = v[1];
    v[1] = (3.046874709125380054e-2 * x)
		 + (0.93906250581749239892 * v[0]);
    return (int)(v[0] + v[1]);
}
