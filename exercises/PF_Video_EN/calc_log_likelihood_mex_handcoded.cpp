#include <cmath>
#include "mex.h"

//
// Calculation of Log Likelihood
//

int round(double x){
    
    if (x > 0){
        
        return int(x + 0.5);
    }
    else{
        
        return 0;
    }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){

	// Input Array

	if (nrhs != 4)		mexErrMsgTxt("Four inputs required.");

	double		Xstd_rgb	= mxGetScalar(prhs[0]);

	double*		C			= mxGetPr(prhs[1]);
	int			C_m			= mxGetM(prhs[1]);
	int			C_n			= mxGetN(prhs[1]);

	double*		X			= mxGetPr(prhs[2]);
	int			X_m			= mxGetM(prhs[2]);
	int			X_n			= mxGetN(prhs[2]);

	unsigned char*	Y		= (unsigned char*)mxGetData(prhs[3]);
	int			Y_m			= mxGetM(prhs[3]);
	int			Y_n			= mxGetN(prhs[3]);

	if (C_m * C_n != 3)	mexErrMsgTxt("2nd input should be 3 x 1 matrix");
	if (X_m != 2)		mexErrMsgTxt("3rd input should be 2 x N matrix");

	
	// Output Array

	plhs[0] = mxCreateDoubleMatrix(1, X_n, mxREAL);

	double*		L			= mxGetPr(plhs[0]);

	int			k, j1, j2, j3, m, n;

	double		dR, dG, dB, D2;

    double      pi = 3.141592653589793;
	double		A = -log(sqrt(2.0 * pi) * Xstd_rgb);
	double		B = -0.5 / (Xstd_rgb * Xstd_rgb);

	Y_n	= Y_n / 3;
    
	for(k = 0; k < X_n; k++) {

		m	= round(X[2*k]);
		n	= round(X[2*k+1]);

		if (m >= 1 && n >= 1 && m <= Y_m && n <= Y_n) {

			j1 = (m - 1) + (n - 1) * Y_m;
            j2 = j1 + (Y_m * Y_n);
            j3 = j2 + (Y_m * Y_n);

			dR	= double(Y[j1]) - C[0];
			dG	= double(Y[j2]) - C[1];
			dB	= double(Y[j3]) - C[2];

			D2	= dR * dR + dG * dG + dB * dB;

			L[k]	= A + B * D2;
		}
		else {

			L[k]	= -mxGetInf();
		}
	}
}