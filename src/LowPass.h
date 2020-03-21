#ifndef LOWPASS_h
#define LOWPASS_h

//Low pass butterworth filter order=1 alpha1=0.1 
class  LowPass
{
	private:
		float v[2];
		int switchpoint;
	public:
		LowPass(int s);
		int step(int x);
};

#endif