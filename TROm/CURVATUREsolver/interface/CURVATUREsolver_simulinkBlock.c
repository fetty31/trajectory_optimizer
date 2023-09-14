/*
CURVATUREsolver : A fast customized optimization solver.

Copyright (C) 2013-2021 EMBOTECH AG [info@embotech.com]. All rights reserved.


This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/


#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME CURVATUREsolver_simulinkBlock

#include "simstruc.h"



/* include FORCESPRO functions and defs */
#include "../include/CURVATUREsolver.h" 

/* SYSTEM INCLUDES FOR TIMING ------------------------------------------ */


#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

typedef CURVATUREsolverinterface_float CURVATUREsolvernmpc_float;

extern void CURVATUREsolver_casadi2forces(double *x, double *y, double *l, double *p, double *f, double *nabla_f, double *c, double *nabla_c, double *h, double *nabla_h, double *hess, solver_int32_default stage, solver_int32_default iteration, solver_int32_default threadID);
CURVATUREsolver_extfunc pt2function_CURVATUREsolver = &CURVATUREsolver_casadi2forces;




/*====================*
 * S-function methods *
 *====================*/
/* Function: mdlInitializeSizes =========================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{

    DECL_AND_INIT_DIMSINFO(inputDimsInfo);
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
	{
		return; /* Parameter mismatch will be reported by Simulink */
    }

	/* initialize size of continuous and discrete states to zero */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

	/* initialize input ports - there are 6 in total */
    if (!ssSetNumInputPorts(S, 6)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 2952, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 2952, 1);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/
	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 656, 1);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/
	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 656, 1);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/
	
	/* Input Port 4 */
    ssSetInputPortMatrixDimensions(S,  4, 2952, 1);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 4, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/
	
	/* Input Port 5 */
    ssSetInputPortMatrixDimensions(S,  5, 7872, 1);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 5, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 5, 1); /*direct input signal access*/
 


	/* initialize output ports - there are 328 in total */
    if (!ssSetNumOutputPorts(S, 328)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 9, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 1 */
    ssSetOutputPortMatrixDimensions(S,  1, 9, 1);
    ssSetOutputPortDataType(S, 1, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 2 */
    ssSetOutputPortMatrixDimensions(S,  2, 9, 1);
    ssSetOutputPortDataType(S, 2, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 3 */
    ssSetOutputPortMatrixDimensions(S,  3, 9, 1);
    ssSetOutputPortDataType(S, 3, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 4 */
    ssSetOutputPortMatrixDimensions(S,  4, 9, 1);
    ssSetOutputPortDataType(S, 4, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 5 */
    ssSetOutputPortMatrixDimensions(S,  5, 9, 1);
    ssSetOutputPortDataType(S, 5, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 6 */
    ssSetOutputPortMatrixDimensions(S,  6, 9, 1);
    ssSetOutputPortDataType(S, 6, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 6, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 7 */
    ssSetOutputPortMatrixDimensions(S,  7, 9, 1);
    ssSetOutputPortDataType(S, 7, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 7, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 8 */
    ssSetOutputPortMatrixDimensions(S,  8, 9, 1);
    ssSetOutputPortDataType(S, 8, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 8, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 9 */
    ssSetOutputPortMatrixDimensions(S,  9, 9, 1);
    ssSetOutputPortDataType(S, 9, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 9, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 10 */
    ssSetOutputPortMatrixDimensions(S,  10, 9, 1);
    ssSetOutputPortDataType(S, 10, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 10, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 11 */
    ssSetOutputPortMatrixDimensions(S,  11, 9, 1);
    ssSetOutputPortDataType(S, 11, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 11, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 12 */
    ssSetOutputPortMatrixDimensions(S,  12, 9, 1);
    ssSetOutputPortDataType(S, 12, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 12, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 13 */
    ssSetOutputPortMatrixDimensions(S,  13, 9, 1);
    ssSetOutputPortDataType(S, 13, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 13, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 14 */
    ssSetOutputPortMatrixDimensions(S,  14, 9, 1);
    ssSetOutputPortDataType(S, 14, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 14, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 15 */
    ssSetOutputPortMatrixDimensions(S,  15, 9, 1);
    ssSetOutputPortDataType(S, 15, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 15, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 16 */
    ssSetOutputPortMatrixDimensions(S,  16, 9, 1);
    ssSetOutputPortDataType(S, 16, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 16, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 17 */
    ssSetOutputPortMatrixDimensions(S,  17, 9, 1);
    ssSetOutputPortDataType(S, 17, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 17, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 18 */
    ssSetOutputPortMatrixDimensions(S,  18, 9, 1);
    ssSetOutputPortDataType(S, 18, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 18, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 19 */
    ssSetOutputPortMatrixDimensions(S,  19, 9, 1);
    ssSetOutputPortDataType(S, 19, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 19, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 20 */
    ssSetOutputPortMatrixDimensions(S,  20, 9, 1);
    ssSetOutputPortDataType(S, 20, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 20, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 21 */
    ssSetOutputPortMatrixDimensions(S,  21, 9, 1);
    ssSetOutputPortDataType(S, 21, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 21, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 22 */
    ssSetOutputPortMatrixDimensions(S,  22, 9, 1);
    ssSetOutputPortDataType(S, 22, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 22, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 23 */
    ssSetOutputPortMatrixDimensions(S,  23, 9, 1);
    ssSetOutputPortDataType(S, 23, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 23, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 24 */
    ssSetOutputPortMatrixDimensions(S,  24, 9, 1);
    ssSetOutputPortDataType(S, 24, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 24, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 25 */
    ssSetOutputPortMatrixDimensions(S,  25, 9, 1);
    ssSetOutputPortDataType(S, 25, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 25, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 26 */
    ssSetOutputPortMatrixDimensions(S,  26, 9, 1);
    ssSetOutputPortDataType(S, 26, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 26, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 27 */
    ssSetOutputPortMatrixDimensions(S,  27, 9, 1);
    ssSetOutputPortDataType(S, 27, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 27, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 28 */
    ssSetOutputPortMatrixDimensions(S,  28, 9, 1);
    ssSetOutputPortDataType(S, 28, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 28, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 29 */
    ssSetOutputPortMatrixDimensions(S,  29, 9, 1);
    ssSetOutputPortDataType(S, 29, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 29, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 30 */
    ssSetOutputPortMatrixDimensions(S,  30, 9, 1);
    ssSetOutputPortDataType(S, 30, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 30, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 31 */
    ssSetOutputPortMatrixDimensions(S,  31, 9, 1);
    ssSetOutputPortDataType(S, 31, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 31, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 32 */
    ssSetOutputPortMatrixDimensions(S,  32, 9, 1);
    ssSetOutputPortDataType(S, 32, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 32, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 33 */
    ssSetOutputPortMatrixDimensions(S,  33, 9, 1);
    ssSetOutputPortDataType(S, 33, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 33, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 34 */
    ssSetOutputPortMatrixDimensions(S,  34, 9, 1);
    ssSetOutputPortDataType(S, 34, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 34, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 35 */
    ssSetOutputPortMatrixDimensions(S,  35, 9, 1);
    ssSetOutputPortDataType(S, 35, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 35, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 36 */
    ssSetOutputPortMatrixDimensions(S,  36, 9, 1);
    ssSetOutputPortDataType(S, 36, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 36, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 37 */
    ssSetOutputPortMatrixDimensions(S,  37, 9, 1);
    ssSetOutputPortDataType(S, 37, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 37, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 38 */
    ssSetOutputPortMatrixDimensions(S,  38, 9, 1);
    ssSetOutputPortDataType(S, 38, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 38, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 39 */
    ssSetOutputPortMatrixDimensions(S,  39, 9, 1);
    ssSetOutputPortDataType(S, 39, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 39, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 40 */
    ssSetOutputPortMatrixDimensions(S,  40, 9, 1);
    ssSetOutputPortDataType(S, 40, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 40, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 41 */
    ssSetOutputPortMatrixDimensions(S,  41, 9, 1);
    ssSetOutputPortDataType(S, 41, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 41, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 42 */
    ssSetOutputPortMatrixDimensions(S,  42, 9, 1);
    ssSetOutputPortDataType(S, 42, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 42, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 43 */
    ssSetOutputPortMatrixDimensions(S,  43, 9, 1);
    ssSetOutputPortDataType(S, 43, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 43, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 44 */
    ssSetOutputPortMatrixDimensions(S,  44, 9, 1);
    ssSetOutputPortDataType(S, 44, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 44, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 45 */
    ssSetOutputPortMatrixDimensions(S,  45, 9, 1);
    ssSetOutputPortDataType(S, 45, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 45, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 46 */
    ssSetOutputPortMatrixDimensions(S,  46, 9, 1);
    ssSetOutputPortDataType(S, 46, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 46, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 47 */
    ssSetOutputPortMatrixDimensions(S,  47, 9, 1);
    ssSetOutputPortDataType(S, 47, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 47, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 48 */
    ssSetOutputPortMatrixDimensions(S,  48, 9, 1);
    ssSetOutputPortDataType(S, 48, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 48, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 49 */
    ssSetOutputPortMatrixDimensions(S,  49, 9, 1);
    ssSetOutputPortDataType(S, 49, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 49, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 50 */
    ssSetOutputPortMatrixDimensions(S,  50, 9, 1);
    ssSetOutputPortDataType(S, 50, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 50, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 51 */
    ssSetOutputPortMatrixDimensions(S,  51, 9, 1);
    ssSetOutputPortDataType(S, 51, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 51, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 52 */
    ssSetOutputPortMatrixDimensions(S,  52, 9, 1);
    ssSetOutputPortDataType(S, 52, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 52, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 53 */
    ssSetOutputPortMatrixDimensions(S,  53, 9, 1);
    ssSetOutputPortDataType(S, 53, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 53, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 54 */
    ssSetOutputPortMatrixDimensions(S,  54, 9, 1);
    ssSetOutputPortDataType(S, 54, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 54, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 55 */
    ssSetOutputPortMatrixDimensions(S,  55, 9, 1);
    ssSetOutputPortDataType(S, 55, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 55, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 56 */
    ssSetOutputPortMatrixDimensions(S,  56, 9, 1);
    ssSetOutputPortDataType(S, 56, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 56, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 57 */
    ssSetOutputPortMatrixDimensions(S,  57, 9, 1);
    ssSetOutputPortDataType(S, 57, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 57, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 58 */
    ssSetOutputPortMatrixDimensions(S,  58, 9, 1);
    ssSetOutputPortDataType(S, 58, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 58, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 59 */
    ssSetOutputPortMatrixDimensions(S,  59, 9, 1);
    ssSetOutputPortDataType(S, 59, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 59, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 60 */
    ssSetOutputPortMatrixDimensions(S,  60, 9, 1);
    ssSetOutputPortDataType(S, 60, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 60, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 61 */
    ssSetOutputPortMatrixDimensions(S,  61, 9, 1);
    ssSetOutputPortDataType(S, 61, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 61, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 62 */
    ssSetOutputPortMatrixDimensions(S,  62, 9, 1);
    ssSetOutputPortDataType(S, 62, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 62, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 63 */
    ssSetOutputPortMatrixDimensions(S,  63, 9, 1);
    ssSetOutputPortDataType(S, 63, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 63, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 64 */
    ssSetOutputPortMatrixDimensions(S,  64, 9, 1);
    ssSetOutputPortDataType(S, 64, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 64, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 65 */
    ssSetOutputPortMatrixDimensions(S,  65, 9, 1);
    ssSetOutputPortDataType(S, 65, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 65, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 66 */
    ssSetOutputPortMatrixDimensions(S,  66, 9, 1);
    ssSetOutputPortDataType(S, 66, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 66, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 67 */
    ssSetOutputPortMatrixDimensions(S,  67, 9, 1);
    ssSetOutputPortDataType(S, 67, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 67, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 68 */
    ssSetOutputPortMatrixDimensions(S,  68, 9, 1);
    ssSetOutputPortDataType(S, 68, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 68, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 69 */
    ssSetOutputPortMatrixDimensions(S,  69, 9, 1);
    ssSetOutputPortDataType(S, 69, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 69, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 70 */
    ssSetOutputPortMatrixDimensions(S,  70, 9, 1);
    ssSetOutputPortDataType(S, 70, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 70, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 71 */
    ssSetOutputPortMatrixDimensions(S,  71, 9, 1);
    ssSetOutputPortDataType(S, 71, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 71, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 72 */
    ssSetOutputPortMatrixDimensions(S,  72, 9, 1);
    ssSetOutputPortDataType(S, 72, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 72, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 73 */
    ssSetOutputPortMatrixDimensions(S,  73, 9, 1);
    ssSetOutputPortDataType(S, 73, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 73, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 74 */
    ssSetOutputPortMatrixDimensions(S,  74, 9, 1);
    ssSetOutputPortDataType(S, 74, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 74, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 75 */
    ssSetOutputPortMatrixDimensions(S,  75, 9, 1);
    ssSetOutputPortDataType(S, 75, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 75, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 76 */
    ssSetOutputPortMatrixDimensions(S,  76, 9, 1);
    ssSetOutputPortDataType(S, 76, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 76, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 77 */
    ssSetOutputPortMatrixDimensions(S,  77, 9, 1);
    ssSetOutputPortDataType(S, 77, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 77, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 78 */
    ssSetOutputPortMatrixDimensions(S,  78, 9, 1);
    ssSetOutputPortDataType(S, 78, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 78, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 79 */
    ssSetOutputPortMatrixDimensions(S,  79, 9, 1);
    ssSetOutputPortDataType(S, 79, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 79, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 80 */
    ssSetOutputPortMatrixDimensions(S,  80, 9, 1);
    ssSetOutputPortDataType(S, 80, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 80, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 81 */
    ssSetOutputPortMatrixDimensions(S,  81, 9, 1);
    ssSetOutputPortDataType(S, 81, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 81, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 82 */
    ssSetOutputPortMatrixDimensions(S,  82, 9, 1);
    ssSetOutputPortDataType(S, 82, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 82, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 83 */
    ssSetOutputPortMatrixDimensions(S,  83, 9, 1);
    ssSetOutputPortDataType(S, 83, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 83, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 84 */
    ssSetOutputPortMatrixDimensions(S,  84, 9, 1);
    ssSetOutputPortDataType(S, 84, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 84, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 85 */
    ssSetOutputPortMatrixDimensions(S,  85, 9, 1);
    ssSetOutputPortDataType(S, 85, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 85, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 86 */
    ssSetOutputPortMatrixDimensions(S,  86, 9, 1);
    ssSetOutputPortDataType(S, 86, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 86, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 87 */
    ssSetOutputPortMatrixDimensions(S,  87, 9, 1);
    ssSetOutputPortDataType(S, 87, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 87, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 88 */
    ssSetOutputPortMatrixDimensions(S,  88, 9, 1);
    ssSetOutputPortDataType(S, 88, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 88, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 89 */
    ssSetOutputPortMatrixDimensions(S,  89, 9, 1);
    ssSetOutputPortDataType(S, 89, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 89, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 90 */
    ssSetOutputPortMatrixDimensions(S,  90, 9, 1);
    ssSetOutputPortDataType(S, 90, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 90, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 91 */
    ssSetOutputPortMatrixDimensions(S,  91, 9, 1);
    ssSetOutputPortDataType(S, 91, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 91, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 92 */
    ssSetOutputPortMatrixDimensions(S,  92, 9, 1);
    ssSetOutputPortDataType(S, 92, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 92, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 93 */
    ssSetOutputPortMatrixDimensions(S,  93, 9, 1);
    ssSetOutputPortDataType(S, 93, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 93, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 94 */
    ssSetOutputPortMatrixDimensions(S,  94, 9, 1);
    ssSetOutputPortDataType(S, 94, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 94, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 95 */
    ssSetOutputPortMatrixDimensions(S,  95, 9, 1);
    ssSetOutputPortDataType(S, 95, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 95, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 96 */
    ssSetOutputPortMatrixDimensions(S,  96, 9, 1);
    ssSetOutputPortDataType(S, 96, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 96, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 97 */
    ssSetOutputPortMatrixDimensions(S,  97, 9, 1);
    ssSetOutputPortDataType(S, 97, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 97, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 98 */
    ssSetOutputPortMatrixDimensions(S,  98, 9, 1);
    ssSetOutputPortDataType(S, 98, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 98, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 99 */
    ssSetOutputPortMatrixDimensions(S,  99, 9, 1);
    ssSetOutputPortDataType(S, 99, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 99, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 100 */
    ssSetOutputPortMatrixDimensions(S,  100, 9, 1);
    ssSetOutputPortDataType(S, 100, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 100, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 101 */
    ssSetOutputPortMatrixDimensions(S,  101, 9, 1);
    ssSetOutputPortDataType(S, 101, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 101, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 102 */
    ssSetOutputPortMatrixDimensions(S,  102, 9, 1);
    ssSetOutputPortDataType(S, 102, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 102, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 103 */
    ssSetOutputPortMatrixDimensions(S,  103, 9, 1);
    ssSetOutputPortDataType(S, 103, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 103, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 104 */
    ssSetOutputPortMatrixDimensions(S,  104, 9, 1);
    ssSetOutputPortDataType(S, 104, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 104, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 105 */
    ssSetOutputPortMatrixDimensions(S,  105, 9, 1);
    ssSetOutputPortDataType(S, 105, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 105, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 106 */
    ssSetOutputPortMatrixDimensions(S,  106, 9, 1);
    ssSetOutputPortDataType(S, 106, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 106, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 107 */
    ssSetOutputPortMatrixDimensions(S,  107, 9, 1);
    ssSetOutputPortDataType(S, 107, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 107, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 108 */
    ssSetOutputPortMatrixDimensions(S,  108, 9, 1);
    ssSetOutputPortDataType(S, 108, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 108, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 109 */
    ssSetOutputPortMatrixDimensions(S,  109, 9, 1);
    ssSetOutputPortDataType(S, 109, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 109, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 110 */
    ssSetOutputPortMatrixDimensions(S,  110, 9, 1);
    ssSetOutputPortDataType(S, 110, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 110, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 111 */
    ssSetOutputPortMatrixDimensions(S,  111, 9, 1);
    ssSetOutputPortDataType(S, 111, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 111, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 112 */
    ssSetOutputPortMatrixDimensions(S,  112, 9, 1);
    ssSetOutputPortDataType(S, 112, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 112, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 113 */
    ssSetOutputPortMatrixDimensions(S,  113, 9, 1);
    ssSetOutputPortDataType(S, 113, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 113, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 114 */
    ssSetOutputPortMatrixDimensions(S,  114, 9, 1);
    ssSetOutputPortDataType(S, 114, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 114, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 115 */
    ssSetOutputPortMatrixDimensions(S,  115, 9, 1);
    ssSetOutputPortDataType(S, 115, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 115, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 116 */
    ssSetOutputPortMatrixDimensions(S,  116, 9, 1);
    ssSetOutputPortDataType(S, 116, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 116, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 117 */
    ssSetOutputPortMatrixDimensions(S,  117, 9, 1);
    ssSetOutputPortDataType(S, 117, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 117, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 118 */
    ssSetOutputPortMatrixDimensions(S,  118, 9, 1);
    ssSetOutputPortDataType(S, 118, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 118, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 119 */
    ssSetOutputPortMatrixDimensions(S,  119, 9, 1);
    ssSetOutputPortDataType(S, 119, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 119, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 120 */
    ssSetOutputPortMatrixDimensions(S,  120, 9, 1);
    ssSetOutputPortDataType(S, 120, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 120, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 121 */
    ssSetOutputPortMatrixDimensions(S,  121, 9, 1);
    ssSetOutputPortDataType(S, 121, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 121, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 122 */
    ssSetOutputPortMatrixDimensions(S,  122, 9, 1);
    ssSetOutputPortDataType(S, 122, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 122, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 123 */
    ssSetOutputPortMatrixDimensions(S,  123, 9, 1);
    ssSetOutputPortDataType(S, 123, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 123, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 124 */
    ssSetOutputPortMatrixDimensions(S,  124, 9, 1);
    ssSetOutputPortDataType(S, 124, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 124, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 125 */
    ssSetOutputPortMatrixDimensions(S,  125, 9, 1);
    ssSetOutputPortDataType(S, 125, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 125, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 126 */
    ssSetOutputPortMatrixDimensions(S,  126, 9, 1);
    ssSetOutputPortDataType(S, 126, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 126, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 127 */
    ssSetOutputPortMatrixDimensions(S,  127, 9, 1);
    ssSetOutputPortDataType(S, 127, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 127, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 128 */
    ssSetOutputPortMatrixDimensions(S,  128, 9, 1);
    ssSetOutputPortDataType(S, 128, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 128, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 129 */
    ssSetOutputPortMatrixDimensions(S,  129, 9, 1);
    ssSetOutputPortDataType(S, 129, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 129, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 130 */
    ssSetOutputPortMatrixDimensions(S,  130, 9, 1);
    ssSetOutputPortDataType(S, 130, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 130, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 131 */
    ssSetOutputPortMatrixDimensions(S,  131, 9, 1);
    ssSetOutputPortDataType(S, 131, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 131, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 132 */
    ssSetOutputPortMatrixDimensions(S,  132, 9, 1);
    ssSetOutputPortDataType(S, 132, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 132, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 133 */
    ssSetOutputPortMatrixDimensions(S,  133, 9, 1);
    ssSetOutputPortDataType(S, 133, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 133, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 134 */
    ssSetOutputPortMatrixDimensions(S,  134, 9, 1);
    ssSetOutputPortDataType(S, 134, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 134, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 135 */
    ssSetOutputPortMatrixDimensions(S,  135, 9, 1);
    ssSetOutputPortDataType(S, 135, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 135, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 136 */
    ssSetOutputPortMatrixDimensions(S,  136, 9, 1);
    ssSetOutputPortDataType(S, 136, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 136, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 137 */
    ssSetOutputPortMatrixDimensions(S,  137, 9, 1);
    ssSetOutputPortDataType(S, 137, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 137, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 138 */
    ssSetOutputPortMatrixDimensions(S,  138, 9, 1);
    ssSetOutputPortDataType(S, 138, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 138, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 139 */
    ssSetOutputPortMatrixDimensions(S,  139, 9, 1);
    ssSetOutputPortDataType(S, 139, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 139, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 140 */
    ssSetOutputPortMatrixDimensions(S,  140, 9, 1);
    ssSetOutputPortDataType(S, 140, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 140, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 141 */
    ssSetOutputPortMatrixDimensions(S,  141, 9, 1);
    ssSetOutputPortDataType(S, 141, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 141, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 142 */
    ssSetOutputPortMatrixDimensions(S,  142, 9, 1);
    ssSetOutputPortDataType(S, 142, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 142, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 143 */
    ssSetOutputPortMatrixDimensions(S,  143, 9, 1);
    ssSetOutputPortDataType(S, 143, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 143, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 144 */
    ssSetOutputPortMatrixDimensions(S,  144, 9, 1);
    ssSetOutputPortDataType(S, 144, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 144, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 145 */
    ssSetOutputPortMatrixDimensions(S,  145, 9, 1);
    ssSetOutputPortDataType(S, 145, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 145, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 146 */
    ssSetOutputPortMatrixDimensions(S,  146, 9, 1);
    ssSetOutputPortDataType(S, 146, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 146, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 147 */
    ssSetOutputPortMatrixDimensions(S,  147, 9, 1);
    ssSetOutputPortDataType(S, 147, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 147, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 148 */
    ssSetOutputPortMatrixDimensions(S,  148, 9, 1);
    ssSetOutputPortDataType(S, 148, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 148, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 149 */
    ssSetOutputPortMatrixDimensions(S,  149, 9, 1);
    ssSetOutputPortDataType(S, 149, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 149, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 150 */
    ssSetOutputPortMatrixDimensions(S,  150, 9, 1);
    ssSetOutputPortDataType(S, 150, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 150, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 151 */
    ssSetOutputPortMatrixDimensions(S,  151, 9, 1);
    ssSetOutputPortDataType(S, 151, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 151, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 152 */
    ssSetOutputPortMatrixDimensions(S,  152, 9, 1);
    ssSetOutputPortDataType(S, 152, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 152, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 153 */
    ssSetOutputPortMatrixDimensions(S,  153, 9, 1);
    ssSetOutputPortDataType(S, 153, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 153, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 154 */
    ssSetOutputPortMatrixDimensions(S,  154, 9, 1);
    ssSetOutputPortDataType(S, 154, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 154, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 155 */
    ssSetOutputPortMatrixDimensions(S,  155, 9, 1);
    ssSetOutputPortDataType(S, 155, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 155, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 156 */
    ssSetOutputPortMatrixDimensions(S,  156, 9, 1);
    ssSetOutputPortDataType(S, 156, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 156, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 157 */
    ssSetOutputPortMatrixDimensions(S,  157, 9, 1);
    ssSetOutputPortDataType(S, 157, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 157, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 158 */
    ssSetOutputPortMatrixDimensions(S,  158, 9, 1);
    ssSetOutputPortDataType(S, 158, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 158, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 159 */
    ssSetOutputPortMatrixDimensions(S,  159, 9, 1);
    ssSetOutputPortDataType(S, 159, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 159, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 160 */
    ssSetOutputPortMatrixDimensions(S,  160, 9, 1);
    ssSetOutputPortDataType(S, 160, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 160, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 161 */
    ssSetOutputPortMatrixDimensions(S,  161, 9, 1);
    ssSetOutputPortDataType(S, 161, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 161, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 162 */
    ssSetOutputPortMatrixDimensions(S,  162, 9, 1);
    ssSetOutputPortDataType(S, 162, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 162, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 163 */
    ssSetOutputPortMatrixDimensions(S,  163, 9, 1);
    ssSetOutputPortDataType(S, 163, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 163, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 164 */
    ssSetOutputPortMatrixDimensions(S,  164, 9, 1);
    ssSetOutputPortDataType(S, 164, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 164, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 165 */
    ssSetOutputPortMatrixDimensions(S,  165, 9, 1);
    ssSetOutputPortDataType(S, 165, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 165, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 166 */
    ssSetOutputPortMatrixDimensions(S,  166, 9, 1);
    ssSetOutputPortDataType(S, 166, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 166, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 167 */
    ssSetOutputPortMatrixDimensions(S,  167, 9, 1);
    ssSetOutputPortDataType(S, 167, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 167, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 168 */
    ssSetOutputPortMatrixDimensions(S,  168, 9, 1);
    ssSetOutputPortDataType(S, 168, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 168, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 169 */
    ssSetOutputPortMatrixDimensions(S,  169, 9, 1);
    ssSetOutputPortDataType(S, 169, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 169, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 170 */
    ssSetOutputPortMatrixDimensions(S,  170, 9, 1);
    ssSetOutputPortDataType(S, 170, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 170, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 171 */
    ssSetOutputPortMatrixDimensions(S,  171, 9, 1);
    ssSetOutputPortDataType(S, 171, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 171, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 172 */
    ssSetOutputPortMatrixDimensions(S,  172, 9, 1);
    ssSetOutputPortDataType(S, 172, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 172, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 173 */
    ssSetOutputPortMatrixDimensions(S,  173, 9, 1);
    ssSetOutputPortDataType(S, 173, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 173, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 174 */
    ssSetOutputPortMatrixDimensions(S,  174, 9, 1);
    ssSetOutputPortDataType(S, 174, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 174, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 175 */
    ssSetOutputPortMatrixDimensions(S,  175, 9, 1);
    ssSetOutputPortDataType(S, 175, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 175, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 176 */
    ssSetOutputPortMatrixDimensions(S,  176, 9, 1);
    ssSetOutputPortDataType(S, 176, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 176, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 177 */
    ssSetOutputPortMatrixDimensions(S,  177, 9, 1);
    ssSetOutputPortDataType(S, 177, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 177, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 178 */
    ssSetOutputPortMatrixDimensions(S,  178, 9, 1);
    ssSetOutputPortDataType(S, 178, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 178, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 179 */
    ssSetOutputPortMatrixDimensions(S,  179, 9, 1);
    ssSetOutputPortDataType(S, 179, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 179, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 180 */
    ssSetOutputPortMatrixDimensions(S,  180, 9, 1);
    ssSetOutputPortDataType(S, 180, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 180, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 181 */
    ssSetOutputPortMatrixDimensions(S,  181, 9, 1);
    ssSetOutputPortDataType(S, 181, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 181, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 182 */
    ssSetOutputPortMatrixDimensions(S,  182, 9, 1);
    ssSetOutputPortDataType(S, 182, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 182, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 183 */
    ssSetOutputPortMatrixDimensions(S,  183, 9, 1);
    ssSetOutputPortDataType(S, 183, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 183, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 184 */
    ssSetOutputPortMatrixDimensions(S,  184, 9, 1);
    ssSetOutputPortDataType(S, 184, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 184, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 185 */
    ssSetOutputPortMatrixDimensions(S,  185, 9, 1);
    ssSetOutputPortDataType(S, 185, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 185, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 186 */
    ssSetOutputPortMatrixDimensions(S,  186, 9, 1);
    ssSetOutputPortDataType(S, 186, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 186, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 187 */
    ssSetOutputPortMatrixDimensions(S,  187, 9, 1);
    ssSetOutputPortDataType(S, 187, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 187, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 188 */
    ssSetOutputPortMatrixDimensions(S,  188, 9, 1);
    ssSetOutputPortDataType(S, 188, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 188, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 189 */
    ssSetOutputPortMatrixDimensions(S,  189, 9, 1);
    ssSetOutputPortDataType(S, 189, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 189, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 190 */
    ssSetOutputPortMatrixDimensions(S,  190, 9, 1);
    ssSetOutputPortDataType(S, 190, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 190, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 191 */
    ssSetOutputPortMatrixDimensions(S,  191, 9, 1);
    ssSetOutputPortDataType(S, 191, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 191, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 192 */
    ssSetOutputPortMatrixDimensions(S,  192, 9, 1);
    ssSetOutputPortDataType(S, 192, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 192, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 193 */
    ssSetOutputPortMatrixDimensions(S,  193, 9, 1);
    ssSetOutputPortDataType(S, 193, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 193, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 194 */
    ssSetOutputPortMatrixDimensions(S,  194, 9, 1);
    ssSetOutputPortDataType(S, 194, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 194, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 195 */
    ssSetOutputPortMatrixDimensions(S,  195, 9, 1);
    ssSetOutputPortDataType(S, 195, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 195, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 196 */
    ssSetOutputPortMatrixDimensions(S,  196, 9, 1);
    ssSetOutputPortDataType(S, 196, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 196, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 197 */
    ssSetOutputPortMatrixDimensions(S,  197, 9, 1);
    ssSetOutputPortDataType(S, 197, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 197, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 198 */
    ssSetOutputPortMatrixDimensions(S,  198, 9, 1);
    ssSetOutputPortDataType(S, 198, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 198, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 199 */
    ssSetOutputPortMatrixDimensions(S,  199, 9, 1);
    ssSetOutputPortDataType(S, 199, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 199, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 200 */
    ssSetOutputPortMatrixDimensions(S,  200, 9, 1);
    ssSetOutputPortDataType(S, 200, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 200, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 201 */
    ssSetOutputPortMatrixDimensions(S,  201, 9, 1);
    ssSetOutputPortDataType(S, 201, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 201, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 202 */
    ssSetOutputPortMatrixDimensions(S,  202, 9, 1);
    ssSetOutputPortDataType(S, 202, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 202, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 203 */
    ssSetOutputPortMatrixDimensions(S,  203, 9, 1);
    ssSetOutputPortDataType(S, 203, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 203, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 204 */
    ssSetOutputPortMatrixDimensions(S,  204, 9, 1);
    ssSetOutputPortDataType(S, 204, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 204, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 205 */
    ssSetOutputPortMatrixDimensions(S,  205, 9, 1);
    ssSetOutputPortDataType(S, 205, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 205, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 206 */
    ssSetOutputPortMatrixDimensions(S,  206, 9, 1);
    ssSetOutputPortDataType(S, 206, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 206, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 207 */
    ssSetOutputPortMatrixDimensions(S,  207, 9, 1);
    ssSetOutputPortDataType(S, 207, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 207, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 208 */
    ssSetOutputPortMatrixDimensions(S,  208, 9, 1);
    ssSetOutputPortDataType(S, 208, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 208, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 209 */
    ssSetOutputPortMatrixDimensions(S,  209, 9, 1);
    ssSetOutputPortDataType(S, 209, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 209, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 210 */
    ssSetOutputPortMatrixDimensions(S,  210, 9, 1);
    ssSetOutputPortDataType(S, 210, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 210, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 211 */
    ssSetOutputPortMatrixDimensions(S,  211, 9, 1);
    ssSetOutputPortDataType(S, 211, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 211, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 212 */
    ssSetOutputPortMatrixDimensions(S,  212, 9, 1);
    ssSetOutputPortDataType(S, 212, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 212, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 213 */
    ssSetOutputPortMatrixDimensions(S,  213, 9, 1);
    ssSetOutputPortDataType(S, 213, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 213, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 214 */
    ssSetOutputPortMatrixDimensions(S,  214, 9, 1);
    ssSetOutputPortDataType(S, 214, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 214, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 215 */
    ssSetOutputPortMatrixDimensions(S,  215, 9, 1);
    ssSetOutputPortDataType(S, 215, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 215, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 216 */
    ssSetOutputPortMatrixDimensions(S,  216, 9, 1);
    ssSetOutputPortDataType(S, 216, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 216, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 217 */
    ssSetOutputPortMatrixDimensions(S,  217, 9, 1);
    ssSetOutputPortDataType(S, 217, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 217, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 218 */
    ssSetOutputPortMatrixDimensions(S,  218, 9, 1);
    ssSetOutputPortDataType(S, 218, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 218, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 219 */
    ssSetOutputPortMatrixDimensions(S,  219, 9, 1);
    ssSetOutputPortDataType(S, 219, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 219, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 220 */
    ssSetOutputPortMatrixDimensions(S,  220, 9, 1);
    ssSetOutputPortDataType(S, 220, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 220, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 221 */
    ssSetOutputPortMatrixDimensions(S,  221, 9, 1);
    ssSetOutputPortDataType(S, 221, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 221, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 222 */
    ssSetOutputPortMatrixDimensions(S,  222, 9, 1);
    ssSetOutputPortDataType(S, 222, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 222, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 223 */
    ssSetOutputPortMatrixDimensions(S,  223, 9, 1);
    ssSetOutputPortDataType(S, 223, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 223, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 224 */
    ssSetOutputPortMatrixDimensions(S,  224, 9, 1);
    ssSetOutputPortDataType(S, 224, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 224, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 225 */
    ssSetOutputPortMatrixDimensions(S,  225, 9, 1);
    ssSetOutputPortDataType(S, 225, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 225, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 226 */
    ssSetOutputPortMatrixDimensions(S,  226, 9, 1);
    ssSetOutputPortDataType(S, 226, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 226, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 227 */
    ssSetOutputPortMatrixDimensions(S,  227, 9, 1);
    ssSetOutputPortDataType(S, 227, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 227, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 228 */
    ssSetOutputPortMatrixDimensions(S,  228, 9, 1);
    ssSetOutputPortDataType(S, 228, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 228, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 229 */
    ssSetOutputPortMatrixDimensions(S,  229, 9, 1);
    ssSetOutputPortDataType(S, 229, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 229, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 230 */
    ssSetOutputPortMatrixDimensions(S,  230, 9, 1);
    ssSetOutputPortDataType(S, 230, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 230, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 231 */
    ssSetOutputPortMatrixDimensions(S,  231, 9, 1);
    ssSetOutputPortDataType(S, 231, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 231, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 232 */
    ssSetOutputPortMatrixDimensions(S,  232, 9, 1);
    ssSetOutputPortDataType(S, 232, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 232, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 233 */
    ssSetOutputPortMatrixDimensions(S,  233, 9, 1);
    ssSetOutputPortDataType(S, 233, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 233, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 234 */
    ssSetOutputPortMatrixDimensions(S,  234, 9, 1);
    ssSetOutputPortDataType(S, 234, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 234, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 235 */
    ssSetOutputPortMatrixDimensions(S,  235, 9, 1);
    ssSetOutputPortDataType(S, 235, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 235, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 236 */
    ssSetOutputPortMatrixDimensions(S,  236, 9, 1);
    ssSetOutputPortDataType(S, 236, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 236, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 237 */
    ssSetOutputPortMatrixDimensions(S,  237, 9, 1);
    ssSetOutputPortDataType(S, 237, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 237, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 238 */
    ssSetOutputPortMatrixDimensions(S,  238, 9, 1);
    ssSetOutputPortDataType(S, 238, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 238, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 239 */
    ssSetOutputPortMatrixDimensions(S,  239, 9, 1);
    ssSetOutputPortDataType(S, 239, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 239, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 240 */
    ssSetOutputPortMatrixDimensions(S,  240, 9, 1);
    ssSetOutputPortDataType(S, 240, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 240, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 241 */
    ssSetOutputPortMatrixDimensions(S,  241, 9, 1);
    ssSetOutputPortDataType(S, 241, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 241, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 242 */
    ssSetOutputPortMatrixDimensions(S,  242, 9, 1);
    ssSetOutputPortDataType(S, 242, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 242, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 243 */
    ssSetOutputPortMatrixDimensions(S,  243, 9, 1);
    ssSetOutputPortDataType(S, 243, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 243, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 244 */
    ssSetOutputPortMatrixDimensions(S,  244, 9, 1);
    ssSetOutputPortDataType(S, 244, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 244, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 245 */
    ssSetOutputPortMatrixDimensions(S,  245, 9, 1);
    ssSetOutputPortDataType(S, 245, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 245, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 246 */
    ssSetOutputPortMatrixDimensions(S,  246, 9, 1);
    ssSetOutputPortDataType(S, 246, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 246, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 247 */
    ssSetOutputPortMatrixDimensions(S,  247, 9, 1);
    ssSetOutputPortDataType(S, 247, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 247, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 248 */
    ssSetOutputPortMatrixDimensions(S,  248, 9, 1);
    ssSetOutputPortDataType(S, 248, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 248, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 249 */
    ssSetOutputPortMatrixDimensions(S,  249, 9, 1);
    ssSetOutputPortDataType(S, 249, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 249, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 250 */
    ssSetOutputPortMatrixDimensions(S,  250, 9, 1);
    ssSetOutputPortDataType(S, 250, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 250, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 251 */
    ssSetOutputPortMatrixDimensions(S,  251, 9, 1);
    ssSetOutputPortDataType(S, 251, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 251, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 252 */
    ssSetOutputPortMatrixDimensions(S,  252, 9, 1);
    ssSetOutputPortDataType(S, 252, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 252, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 253 */
    ssSetOutputPortMatrixDimensions(S,  253, 9, 1);
    ssSetOutputPortDataType(S, 253, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 253, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 254 */
    ssSetOutputPortMatrixDimensions(S,  254, 9, 1);
    ssSetOutputPortDataType(S, 254, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 254, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 255 */
    ssSetOutputPortMatrixDimensions(S,  255, 9, 1);
    ssSetOutputPortDataType(S, 255, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 255, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 256 */
    ssSetOutputPortMatrixDimensions(S,  256, 9, 1);
    ssSetOutputPortDataType(S, 256, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 256, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 257 */
    ssSetOutputPortMatrixDimensions(S,  257, 9, 1);
    ssSetOutputPortDataType(S, 257, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 257, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 258 */
    ssSetOutputPortMatrixDimensions(S,  258, 9, 1);
    ssSetOutputPortDataType(S, 258, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 258, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 259 */
    ssSetOutputPortMatrixDimensions(S,  259, 9, 1);
    ssSetOutputPortDataType(S, 259, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 259, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 260 */
    ssSetOutputPortMatrixDimensions(S,  260, 9, 1);
    ssSetOutputPortDataType(S, 260, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 260, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 261 */
    ssSetOutputPortMatrixDimensions(S,  261, 9, 1);
    ssSetOutputPortDataType(S, 261, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 261, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 262 */
    ssSetOutputPortMatrixDimensions(S,  262, 9, 1);
    ssSetOutputPortDataType(S, 262, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 262, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 263 */
    ssSetOutputPortMatrixDimensions(S,  263, 9, 1);
    ssSetOutputPortDataType(S, 263, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 263, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 264 */
    ssSetOutputPortMatrixDimensions(S,  264, 9, 1);
    ssSetOutputPortDataType(S, 264, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 264, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 265 */
    ssSetOutputPortMatrixDimensions(S,  265, 9, 1);
    ssSetOutputPortDataType(S, 265, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 265, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 266 */
    ssSetOutputPortMatrixDimensions(S,  266, 9, 1);
    ssSetOutputPortDataType(S, 266, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 266, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 267 */
    ssSetOutputPortMatrixDimensions(S,  267, 9, 1);
    ssSetOutputPortDataType(S, 267, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 267, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 268 */
    ssSetOutputPortMatrixDimensions(S,  268, 9, 1);
    ssSetOutputPortDataType(S, 268, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 268, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 269 */
    ssSetOutputPortMatrixDimensions(S,  269, 9, 1);
    ssSetOutputPortDataType(S, 269, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 269, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 270 */
    ssSetOutputPortMatrixDimensions(S,  270, 9, 1);
    ssSetOutputPortDataType(S, 270, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 270, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 271 */
    ssSetOutputPortMatrixDimensions(S,  271, 9, 1);
    ssSetOutputPortDataType(S, 271, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 271, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 272 */
    ssSetOutputPortMatrixDimensions(S,  272, 9, 1);
    ssSetOutputPortDataType(S, 272, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 272, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 273 */
    ssSetOutputPortMatrixDimensions(S,  273, 9, 1);
    ssSetOutputPortDataType(S, 273, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 273, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 274 */
    ssSetOutputPortMatrixDimensions(S,  274, 9, 1);
    ssSetOutputPortDataType(S, 274, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 274, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 275 */
    ssSetOutputPortMatrixDimensions(S,  275, 9, 1);
    ssSetOutputPortDataType(S, 275, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 275, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 276 */
    ssSetOutputPortMatrixDimensions(S,  276, 9, 1);
    ssSetOutputPortDataType(S, 276, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 276, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 277 */
    ssSetOutputPortMatrixDimensions(S,  277, 9, 1);
    ssSetOutputPortDataType(S, 277, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 277, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 278 */
    ssSetOutputPortMatrixDimensions(S,  278, 9, 1);
    ssSetOutputPortDataType(S, 278, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 278, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 279 */
    ssSetOutputPortMatrixDimensions(S,  279, 9, 1);
    ssSetOutputPortDataType(S, 279, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 279, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 280 */
    ssSetOutputPortMatrixDimensions(S,  280, 9, 1);
    ssSetOutputPortDataType(S, 280, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 280, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 281 */
    ssSetOutputPortMatrixDimensions(S,  281, 9, 1);
    ssSetOutputPortDataType(S, 281, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 281, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 282 */
    ssSetOutputPortMatrixDimensions(S,  282, 9, 1);
    ssSetOutputPortDataType(S, 282, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 282, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 283 */
    ssSetOutputPortMatrixDimensions(S,  283, 9, 1);
    ssSetOutputPortDataType(S, 283, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 283, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 284 */
    ssSetOutputPortMatrixDimensions(S,  284, 9, 1);
    ssSetOutputPortDataType(S, 284, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 284, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 285 */
    ssSetOutputPortMatrixDimensions(S,  285, 9, 1);
    ssSetOutputPortDataType(S, 285, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 285, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 286 */
    ssSetOutputPortMatrixDimensions(S,  286, 9, 1);
    ssSetOutputPortDataType(S, 286, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 286, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 287 */
    ssSetOutputPortMatrixDimensions(S,  287, 9, 1);
    ssSetOutputPortDataType(S, 287, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 287, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 288 */
    ssSetOutputPortMatrixDimensions(S,  288, 9, 1);
    ssSetOutputPortDataType(S, 288, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 288, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 289 */
    ssSetOutputPortMatrixDimensions(S,  289, 9, 1);
    ssSetOutputPortDataType(S, 289, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 289, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 290 */
    ssSetOutputPortMatrixDimensions(S,  290, 9, 1);
    ssSetOutputPortDataType(S, 290, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 290, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 291 */
    ssSetOutputPortMatrixDimensions(S,  291, 9, 1);
    ssSetOutputPortDataType(S, 291, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 291, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 292 */
    ssSetOutputPortMatrixDimensions(S,  292, 9, 1);
    ssSetOutputPortDataType(S, 292, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 292, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 293 */
    ssSetOutputPortMatrixDimensions(S,  293, 9, 1);
    ssSetOutputPortDataType(S, 293, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 293, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 294 */
    ssSetOutputPortMatrixDimensions(S,  294, 9, 1);
    ssSetOutputPortDataType(S, 294, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 294, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 295 */
    ssSetOutputPortMatrixDimensions(S,  295, 9, 1);
    ssSetOutputPortDataType(S, 295, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 295, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 296 */
    ssSetOutputPortMatrixDimensions(S,  296, 9, 1);
    ssSetOutputPortDataType(S, 296, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 296, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 297 */
    ssSetOutputPortMatrixDimensions(S,  297, 9, 1);
    ssSetOutputPortDataType(S, 297, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 297, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 298 */
    ssSetOutputPortMatrixDimensions(S,  298, 9, 1);
    ssSetOutputPortDataType(S, 298, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 298, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 299 */
    ssSetOutputPortMatrixDimensions(S,  299, 9, 1);
    ssSetOutputPortDataType(S, 299, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 299, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 300 */
    ssSetOutputPortMatrixDimensions(S,  300, 9, 1);
    ssSetOutputPortDataType(S, 300, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 300, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 301 */
    ssSetOutputPortMatrixDimensions(S,  301, 9, 1);
    ssSetOutputPortDataType(S, 301, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 301, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 302 */
    ssSetOutputPortMatrixDimensions(S,  302, 9, 1);
    ssSetOutputPortDataType(S, 302, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 302, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 303 */
    ssSetOutputPortMatrixDimensions(S,  303, 9, 1);
    ssSetOutputPortDataType(S, 303, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 303, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 304 */
    ssSetOutputPortMatrixDimensions(S,  304, 9, 1);
    ssSetOutputPortDataType(S, 304, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 304, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 305 */
    ssSetOutputPortMatrixDimensions(S,  305, 9, 1);
    ssSetOutputPortDataType(S, 305, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 305, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 306 */
    ssSetOutputPortMatrixDimensions(S,  306, 9, 1);
    ssSetOutputPortDataType(S, 306, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 306, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 307 */
    ssSetOutputPortMatrixDimensions(S,  307, 9, 1);
    ssSetOutputPortDataType(S, 307, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 307, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 308 */
    ssSetOutputPortMatrixDimensions(S,  308, 9, 1);
    ssSetOutputPortDataType(S, 308, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 308, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 309 */
    ssSetOutputPortMatrixDimensions(S,  309, 9, 1);
    ssSetOutputPortDataType(S, 309, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 309, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 310 */
    ssSetOutputPortMatrixDimensions(S,  310, 9, 1);
    ssSetOutputPortDataType(S, 310, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 310, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 311 */
    ssSetOutputPortMatrixDimensions(S,  311, 9, 1);
    ssSetOutputPortDataType(S, 311, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 311, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 312 */
    ssSetOutputPortMatrixDimensions(S,  312, 9, 1);
    ssSetOutputPortDataType(S, 312, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 312, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 313 */
    ssSetOutputPortMatrixDimensions(S,  313, 9, 1);
    ssSetOutputPortDataType(S, 313, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 313, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 314 */
    ssSetOutputPortMatrixDimensions(S,  314, 9, 1);
    ssSetOutputPortDataType(S, 314, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 314, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 315 */
    ssSetOutputPortMatrixDimensions(S,  315, 9, 1);
    ssSetOutputPortDataType(S, 315, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 315, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 316 */
    ssSetOutputPortMatrixDimensions(S,  316, 9, 1);
    ssSetOutputPortDataType(S, 316, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 316, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 317 */
    ssSetOutputPortMatrixDimensions(S,  317, 9, 1);
    ssSetOutputPortDataType(S, 317, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 317, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 318 */
    ssSetOutputPortMatrixDimensions(S,  318, 9, 1);
    ssSetOutputPortDataType(S, 318, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 318, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 319 */
    ssSetOutputPortMatrixDimensions(S,  319, 9, 1);
    ssSetOutputPortDataType(S, 319, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 319, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 320 */
    ssSetOutputPortMatrixDimensions(S,  320, 9, 1);
    ssSetOutputPortDataType(S, 320, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 320, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 321 */
    ssSetOutputPortMatrixDimensions(S,  321, 9, 1);
    ssSetOutputPortDataType(S, 321, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 321, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 322 */
    ssSetOutputPortMatrixDimensions(S,  322, 9, 1);
    ssSetOutputPortDataType(S, 322, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 322, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 323 */
    ssSetOutputPortMatrixDimensions(S,  323, 9, 1);
    ssSetOutputPortDataType(S, 323, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 323, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 324 */
    ssSetOutputPortMatrixDimensions(S,  324, 9, 1);
    ssSetOutputPortDataType(S, 324, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 324, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 325 */
    ssSetOutputPortMatrixDimensions(S,  325, 9, 1);
    ssSetOutputPortDataType(S, 325, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 325, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 326 */
    ssSetOutputPortMatrixDimensions(S,  326, 9, 1);
    ssSetOutputPortDataType(S, 326, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 326, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 327 */
    ssSetOutputPortMatrixDimensions(S,  327, 9, 1);
    ssSetOutputPortDataType(S, 327, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 327, COMPLEX_NO); /* no complex signals suppported */


	/* set sampling time */
    ssSetNumSampleTimes(S, 1);

	/* set internal memory of block */
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ 
	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ 
    /* ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
		             SS_OPTION_WORKS_WITH_CODE_REUSE)); */
	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE );

	
}

#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct        *S, 
                                         int_T            port,
                                         const DimsInfo_T *dimsInfo)
{
    if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
#if defined(MDL_SET_OUTPUT_PORT_DIMENSION_INFO)
static void mdlSetOutputPortDimensionInfo(SimStruct        *S, 
                                          int_T            port, 
                                          const DimsInfo_T *dimsInfo)
{
    if (!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif
# define MDL_SET_INPUT_PORT_FRAME_DATA
static void mdlSetInputPortFrameData(SimStruct  *S, 
                                     int_T      port,
                                     Frame_T    frameData)
{
    ssSetInputPortFrameData(S, port, frameData);
}
/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct *S, solver_int32_default port, DTypeId dType)
{
    ssSetInputPortDataType( S, 0, dType);
}
#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, solver_int32_default port, DTypeId dType)
{
    ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
    ssSetInputPortDataType( S, 0, SS_DOUBLE);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}





/* Function: mdlOutputs =======================================================
 *
*/
static void mdlOutputs(SimStruct *S, int_T tid)
{
	solver_int32_default i, j, k;
	
	/* file pointer for printing */
	FILE *fp = NULL;

	/* Simulink data */
	const real_T *lb = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *ub = (const real_T*) ssGetInputPortSignal(S,1);
	const real_T *hu = (const real_T*) ssGetInputPortSignal(S,2);
	const real_T *hl = (const real_T*) ssGetInputPortSignal(S,3);
	const real_T *x0 = (const real_T*) ssGetInputPortSignal(S,4);
	const real_T *all_parameters = (const real_T*) ssGetInputPortSignal(S,5);
	
    real_T *x001 = (real_T*) ssGetOutputPortSignal(S,0);
	real_T *x002 = (real_T*) ssGetOutputPortSignal(S,1);
	real_T *x003 = (real_T*) ssGetOutputPortSignal(S,2);
	real_T *x004 = (real_T*) ssGetOutputPortSignal(S,3);
	real_T *x005 = (real_T*) ssGetOutputPortSignal(S,4);
	real_T *x006 = (real_T*) ssGetOutputPortSignal(S,5);
	real_T *x007 = (real_T*) ssGetOutputPortSignal(S,6);
	real_T *x008 = (real_T*) ssGetOutputPortSignal(S,7);
	real_T *x009 = (real_T*) ssGetOutputPortSignal(S,8);
	real_T *x010 = (real_T*) ssGetOutputPortSignal(S,9);
	real_T *x011 = (real_T*) ssGetOutputPortSignal(S,10);
	real_T *x012 = (real_T*) ssGetOutputPortSignal(S,11);
	real_T *x013 = (real_T*) ssGetOutputPortSignal(S,12);
	real_T *x014 = (real_T*) ssGetOutputPortSignal(S,13);
	real_T *x015 = (real_T*) ssGetOutputPortSignal(S,14);
	real_T *x016 = (real_T*) ssGetOutputPortSignal(S,15);
	real_T *x017 = (real_T*) ssGetOutputPortSignal(S,16);
	real_T *x018 = (real_T*) ssGetOutputPortSignal(S,17);
	real_T *x019 = (real_T*) ssGetOutputPortSignal(S,18);
	real_T *x020 = (real_T*) ssGetOutputPortSignal(S,19);
	real_T *x021 = (real_T*) ssGetOutputPortSignal(S,20);
	real_T *x022 = (real_T*) ssGetOutputPortSignal(S,21);
	real_T *x023 = (real_T*) ssGetOutputPortSignal(S,22);
	real_T *x024 = (real_T*) ssGetOutputPortSignal(S,23);
	real_T *x025 = (real_T*) ssGetOutputPortSignal(S,24);
	real_T *x026 = (real_T*) ssGetOutputPortSignal(S,25);
	real_T *x027 = (real_T*) ssGetOutputPortSignal(S,26);
	real_T *x028 = (real_T*) ssGetOutputPortSignal(S,27);
	real_T *x029 = (real_T*) ssGetOutputPortSignal(S,28);
	real_T *x030 = (real_T*) ssGetOutputPortSignal(S,29);
	real_T *x031 = (real_T*) ssGetOutputPortSignal(S,30);
	real_T *x032 = (real_T*) ssGetOutputPortSignal(S,31);
	real_T *x033 = (real_T*) ssGetOutputPortSignal(S,32);
	real_T *x034 = (real_T*) ssGetOutputPortSignal(S,33);
	real_T *x035 = (real_T*) ssGetOutputPortSignal(S,34);
	real_T *x036 = (real_T*) ssGetOutputPortSignal(S,35);
	real_T *x037 = (real_T*) ssGetOutputPortSignal(S,36);
	real_T *x038 = (real_T*) ssGetOutputPortSignal(S,37);
	real_T *x039 = (real_T*) ssGetOutputPortSignal(S,38);
	real_T *x040 = (real_T*) ssGetOutputPortSignal(S,39);
	real_T *x041 = (real_T*) ssGetOutputPortSignal(S,40);
	real_T *x042 = (real_T*) ssGetOutputPortSignal(S,41);
	real_T *x043 = (real_T*) ssGetOutputPortSignal(S,42);
	real_T *x044 = (real_T*) ssGetOutputPortSignal(S,43);
	real_T *x045 = (real_T*) ssGetOutputPortSignal(S,44);
	real_T *x046 = (real_T*) ssGetOutputPortSignal(S,45);
	real_T *x047 = (real_T*) ssGetOutputPortSignal(S,46);
	real_T *x048 = (real_T*) ssGetOutputPortSignal(S,47);
	real_T *x049 = (real_T*) ssGetOutputPortSignal(S,48);
	real_T *x050 = (real_T*) ssGetOutputPortSignal(S,49);
	real_T *x051 = (real_T*) ssGetOutputPortSignal(S,50);
	real_T *x052 = (real_T*) ssGetOutputPortSignal(S,51);
	real_T *x053 = (real_T*) ssGetOutputPortSignal(S,52);
	real_T *x054 = (real_T*) ssGetOutputPortSignal(S,53);
	real_T *x055 = (real_T*) ssGetOutputPortSignal(S,54);
	real_T *x056 = (real_T*) ssGetOutputPortSignal(S,55);
	real_T *x057 = (real_T*) ssGetOutputPortSignal(S,56);
	real_T *x058 = (real_T*) ssGetOutputPortSignal(S,57);
	real_T *x059 = (real_T*) ssGetOutputPortSignal(S,58);
	real_T *x060 = (real_T*) ssGetOutputPortSignal(S,59);
	real_T *x061 = (real_T*) ssGetOutputPortSignal(S,60);
	real_T *x062 = (real_T*) ssGetOutputPortSignal(S,61);
	real_T *x063 = (real_T*) ssGetOutputPortSignal(S,62);
	real_T *x064 = (real_T*) ssGetOutputPortSignal(S,63);
	real_T *x065 = (real_T*) ssGetOutputPortSignal(S,64);
	real_T *x066 = (real_T*) ssGetOutputPortSignal(S,65);
	real_T *x067 = (real_T*) ssGetOutputPortSignal(S,66);
	real_T *x068 = (real_T*) ssGetOutputPortSignal(S,67);
	real_T *x069 = (real_T*) ssGetOutputPortSignal(S,68);
	real_T *x070 = (real_T*) ssGetOutputPortSignal(S,69);
	real_T *x071 = (real_T*) ssGetOutputPortSignal(S,70);
	real_T *x072 = (real_T*) ssGetOutputPortSignal(S,71);
	real_T *x073 = (real_T*) ssGetOutputPortSignal(S,72);
	real_T *x074 = (real_T*) ssGetOutputPortSignal(S,73);
	real_T *x075 = (real_T*) ssGetOutputPortSignal(S,74);
	real_T *x076 = (real_T*) ssGetOutputPortSignal(S,75);
	real_T *x077 = (real_T*) ssGetOutputPortSignal(S,76);
	real_T *x078 = (real_T*) ssGetOutputPortSignal(S,77);
	real_T *x079 = (real_T*) ssGetOutputPortSignal(S,78);
	real_T *x080 = (real_T*) ssGetOutputPortSignal(S,79);
	real_T *x081 = (real_T*) ssGetOutputPortSignal(S,80);
	real_T *x082 = (real_T*) ssGetOutputPortSignal(S,81);
	real_T *x083 = (real_T*) ssGetOutputPortSignal(S,82);
	real_T *x084 = (real_T*) ssGetOutputPortSignal(S,83);
	real_T *x085 = (real_T*) ssGetOutputPortSignal(S,84);
	real_T *x086 = (real_T*) ssGetOutputPortSignal(S,85);
	real_T *x087 = (real_T*) ssGetOutputPortSignal(S,86);
	real_T *x088 = (real_T*) ssGetOutputPortSignal(S,87);
	real_T *x089 = (real_T*) ssGetOutputPortSignal(S,88);
	real_T *x090 = (real_T*) ssGetOutputPortSignal(S,89);
	real_T *x091 = (real_T*) ssGetOutputPortSignal(S,90);
	real_T *x092 = (real_T*) ssGetOutputPortSignal(S,91);
	real_T *x093 = (real_T*) ssGetOutputPortSignal(S,92);
	real_T *x094 = (real_T*) ssGetOutputPortSignal(S,93);
	real_T *x095 = (real_T*) ssGetOutputPortSignal(S,94);
	real_T *x096 = (real_T*) ssGetOutputPortSignal(S,95);
	real_T *x097 = (real_T*) ssGetOutputPortSignal(S,96);
	real_T *x098 = (real_T*) ssGetOutputPortSignal(S,97);
	real_T *x099 = (real_T*) ssGetOutputPortSignal(S,98);
	real_T *x100 = (real_T*) ssGetOutputPortSignal(S,99);
	real_T *x101 = (real_T*) ssGetOutputPortSignal(S,100);
	real_T *x102 = (real_T*) ssGetOutputPortSignal(S,101);
	real_T *x103 = (real_T*) ssGetOutputPortSignal(S,102);
	real_T *x104 = (real_T*) ssGetOutputPortSignal(S,103);
	real_T *x105 = (real_T*) ssGetOutputPortSignal(S,104);
	real_T *x106 = (real_T*) ssGetOutputPortSignal(S,105);
	real_T *x107 = (real_T*) ssGetOutputPortSignal(S,106);
	real_T *x108 = (real_T*) ssGetOutputPortSignal(S,107);
	real_T *x109 = (real_T*) ssGetOutputPortSignal(S,108);
	real_T *x110 = (real_T*) ssGetOutputPortSignal(S,109);
	real_T *x111 = (real_T*) ssGetOutputPortSignal(S,110);
	real_T *x112 = (real_T*) ssGetOutputPortSignal(S,111);
	real_T *x113 = (real_T*) ssGetOutputPortSignal(S,112);
	real_T *x114 = (real_T*) ssGetOutputPortSignal(S,113);
	real_T *x115 = (real_T*) ssGetOutputPortSignal(S,114);
	real_T *x116 = (real_T*) ssGetOutputPortSignal(S,115);
	real_T *x117 = (real_T*) ssGetOutputPortSignal(S,116);
	real_T *x118 = (real_T*) ssGetOutputPortSignal(S,117);
	real_T *x119 = (real_T*) ssGetOutputPortSignal(S,118);
	real_T *x120 = (real_T*) ssGetOutputPortSignal(S,119);
	real_T *x121 = (real_T*) ssGetOutputPortSignal(S,120);
	real_T *x122 = (real_T*) ssGetOutputPortSignal(S,121);
	real_T *x123 = (real_T*) ssGetOutputPortSignal(S,122);
	real_T *x124 = (real_T*) ssGetOutputPortSignal(S,123);
	real_T *x125 = (real_T*) ssGetOutputPortSignal(S,124);
	real_T *x126 = (real_T*) ssGetOutputPortSignal(S,125);
	real_T *x127 = (real_T*) ssGetOutputPortSignal(S,126);
	real_T *x128 = (real_T*) ssGetOutputPortSignal(S,127);
	real_T *x129 = (real_T*) ssGetOutputPortSignal(S,128);
	real_T *x130 = (real_T*) ssGetOutputPortSignal(S,129);
	real_T *x131 = (real_T*) ssGetOutputPortSignal(S,130);
	real_T *x132 = (real_T*) ssGetOutputPortSignal(S,131);
	real_T *x133 = (real_T*) ssGetOutputPortSignal(S,132);
	real_T *x134 = (real_T*) ssGetOutputPortSignal(S,133);
	real_T *x135 = (real_T*) ssGetOutputPortSignal(S,134);
	real_T *x136 = (real_T*) ssGetOutputPortSignal(S,135);
	real_T *x137 = (real_T*) ssGetOutputPortSignal(S,136);
	real_T *x138 = (real_T*) ssGetOutputPortSignal(S,137);
	real_T *x139 = (real_T*) ssGetOutputPortSignal(S,138);
	real_T *x140 = (real_T*) ssGetOutputPortSignal(S,139);
	real_T *x141 = (real_T*) ssGetOutputPortSignal(S,140);
	real_T *x142 = (real_T*) ssGetOutputPortSignal(S,141);
	real_T *x143 = (real_T*) ssGetOutputPortSignal(S,142);
	real_T *x144 = (real_T*) ssGetOutputPortSignal(S,143);
	real_T *x145 = (real_T*) ssGetOutputPortSignal(S,144);
	real_T *x146 = (real_T*) ssGetOutputPortSignal(S,145);
	real_T *x147 = (real_T*) ssGetOutputPortSignal(S,146);
	real_T *x148 = (real_T*) ssGetOutputPortSignal(S,147);
	real_T *x149 = (real_T*) ssGetOutputPortSignal(S,148);
	real_T *x150 = (real_T*) ssGetOutputPortSignal(S,149);
	real_T *x151 = (real_T*) ssGetOutputPortSignal(S,150);
	real_T *x152 = (real_T*) ssGetOutputPortSignal(S,151);
	real_T *x153 = (real_T*) ssGetOutputPortSignal(S,152);
	real_T *x154 = (real_T*) ssGetOutputPortSignal(S,153);
	real_T *x155 = (real_T*) ssGetOutputPortSignal(S,154);
	real_T *x156 = (real_T*) ssGetOutputPortSignal(S,155);
	real_T *x157 = (real_T*) ssGetOutputPortSignal(S,156);
	real_T *x158 = (real_T*) ssGetOutputPortSignal(S,157);
	real_T *x159 = (real_T*) ssGetOutputPortSignal(S,158);
	real_T *x160 = (real_T*) ssGetOutputPortSignal(S,159);
	real_T *x161 = (real_T*) ssGetOutputPortSignal(S,160);
	real_T *x162 = (real_T*) ssGetOutputPortSignal(S,161);
	real_T *x163 = (real_T*) ssGetOutputPortSignal(S,162);
	real_T *x164 = (real_T*) ssGetOutputPortSignal(S,163);
	real_T *x165 = (real_T*) ssGetOutputPortSignal(S,164);
	real_T *x166 = (real_T*) ssGetOutputPortSignal(S,165);
	real_T *x167 = (real_T*) ssGetOutputPortSignal(S,166);
	real_T *x168 = (real_T*) ssGetOutputPortSignal(S,167);
	real_T *x169 = (real_T*) ssGetOutputPortSignal(S,168);
	real_T *x170 = (real_T*) ssGetOutputPortSignal(S,169);
	real_T *x171 = (real_T*) ssGetOutputPortSignal(S,170);
	real_T *x172 = (real_T*) ssGetOutputPortSignal(S,171);
	real_T *x173 = (real_T*) ssGetOutputPortSignal(S,172);
	real_T *x174 = (real_T*) ssGetOutputPortSignal(S,173);
	real_T *x175 = (real_T*) ssGetOutputPortSignal(S,174);
	real_T *x176 = (real_T*) ssGetOutputPortSignal(S,175);
	real_T *x177 = (real_T*) ssGetOutputPortSignal(S,176);
	real_T *x178 = (real_T*) ssGetOutputPortSignal(S,177);
	real_T *x179 = (real_T*) ssGetOutputPortSignal(S,178);
	real_T *x180 = (real_T*) ssGetOutputPortSignal(S,179);
	real_T *x181 = (real_T*) ssGetOutputPortSignal(S,180);
	real_T *x182 = (real_T*) ssGetOutputPortSignal(S,181);
	real_T *x183 = (real_T*) ssGetOutputPortSignal(S,182);
	real_T *x184 = (real_T*) ssGetOutputPortSignal(S,183);
	real_T *x185 = (real_T*) ssGetOutputPortSignal(S,184);
	real_T *x186 = (real_T*) ssGetOutputPortSignal(S,185);
	real_T *x187 = (real_T*) ssGetOutputPortSignal(S,186);
	real_T *x188 = (real_T*) ssGetOutputPortSignal(S,187);
	real_T *x189 = (real_T*) ssGetOutputPortSignal(S,188);
	real_T *x190 = (real_T*) ssGetOutputPortSignal(S,189);
	real_T *x191 = (real_T*) ssGetOutputPortSignal(S,190);
	real_T *x192 = (real_T*) ssGetOutputPortSignal(S,191);
	real_T *x193 = (real_T*) ssGetOutputPortSignal(S,192);
	real_T *x194 = (real_T*) ssGetOutputPortSignal(S,193);
	real_T *x195 = (real_T*) ssGetOutputPortSignal(S,194);
	real_T *x196 = (real_T*) ssGetOutputPortSignal(S,195);
	real_T *x197 = (real_T*) ssGetOutputPortSignal(S,196);
	real_T *x198 = (real_T*) ssGetOutputPortSignal(S,197);
	real_T *x199 = (real_T*) ssGetOutputPortSignal(S,198);
	real_T *x200 = (real_T*) ssGetOutputPortSignal(S,199);
	real_T *x201 = (real_T*) ssGetOutputPortSignal(S,200);
	real_T *x202 = (real_T*) ssGetOutputPortSignal(S,201);
	real_T *x203 = (real_T*) ssGetOutputPortSignal(S,202);
	real_T *x204 = (real_T*) ssGetOutputPortSignal(S,203);
	real_T *x205 = (real_T*) ssGetOutputPortSignal(S,204);
	real_T *x206 = (real_T*) ssGetOutputPortSignal(S,205);
	real_T *x207 = (real_T*) ssGetOutputPortSignal(S,206);
	real_T *x208 = (real_T*) ssGetOutputPortSignal(S,207);
	real_T *x209 = (real_T*) ssGetOutputPortSignal(S,208);
	real_T *x210 = (real_T*) ssGetOutputPortSignal(S,209);
	real_T *x211 = (real_T*) ssGetOutputPortSignal(S,210);
	real_T *x212 = (real_T*) ssGetOutputPortSignal(S,211);
	real_T *x213 = (real_T*) ssGetOutputPortSignal(S,212);
	real_T *x214 = (real_T*) ssGetOutputPortSignal(S,213);
	real_T *x215 = (real_T*) ssGetOutputPortSignal(S,214);
	real_T *x216 = (real_T*) ssGetOutputPortSignal(S,215);
	real_T *x217 = (real_T*) ssGetOutputPortSignal(S,216);
	real_T *x218 = (real_T*) ssGetOutputPortSignal(S,217);
	real_T *x219 = (real_T*) ssGetOutputPortSignal(S,218);
	real_T *x220 = (real_T*) ssGetOutputPortSignal(S,219);
	real_T *x221 = (real_T*) ssGetOutputPortSignal(S,220);
	real_T *x222 = (real_T*) ssGetOutputPortSignal(S,221);
	real_T *x223 = (real_T*) ssGetOutputPortSignal(S,222);
	real_T *x224 = (real_T*) ssGetOutputPortSignal(S,223);
	real_T *x225 = (real_T*) ssGetOutputPortSignal(S,224);
	real_T *x226 = (real_T*) ssGetOutputPortSignal(S,225);
	real_T *x227 = (real_T*) ssGetOutputPortSignal(S,226);
	real_T *x228 = (real_T*) ssGetOutputPortSignal(S,227);
	real_T *x229 = (real_T*) ssGetOutputPortSignal(S,228);
	real_T *x230 = (real_T*) ssGetOutputPortSignal(S,229);
	real_T *x231 = (real_T*) ssGetOutputPortSignal(S,230);
	real_T *x232 = (real_T*) ssGetOutputPortSignal(S,231);
	real_T *x233 = (real_T*) ssGetOutputPortSignal(S,232);
	real_T *x234 = (real_T*) ssGetOutputPortSignal(S,233);
	real_T *x235 = (real_T*) ssGetOutputPortSignal(S,234);
	real_T *x236 = (real_T*) ssGetOutputPortSignal(S,235);
	real_T *x237 = (real_T*) ssGetOutputPortSignal(S,236);
	real_T *x238 = (real_T*) ssGetOutputPortSignal(S,237);
	real_T *x239 = (real_T*) ssGetOutputPortSignal(S,238);
	real_T *x240 = (real_T*) ssGetOutputPortSignal(S,239);
	real_T *x241 = (real_T*) ssGetOutputPortSignal(S,240);
	real_T *x242 = (real_T*) ssGetOutputPortSignal(S,241);
	real_T *x243 = (real_T*) ssGetOutputPortSignal(S,242);
	real_T *x244 = (real_T*) ssGetOutputPortSignal(S,243);
	real_T *x245 = (real_T*) ssGetOutputPortSignal(S,244);
	real_T *x246 = (real_T*) ssGetOutputPortSignal(S,245);
	real_T *x247 = (real_T*) ssGetOutputPortSignal(S,246);
	real_T *x248 = (real_T*) ssGetOutputPortSignal(S,247);
	real_T *x249 = (real_T*) ssGetOutputPortSignal(S,248);
	real_T *x250 = (real_T*) ssGetOutputPortSignal(S,249);
	real_T *x251 = (real_T*) ssGetOutputPortSignal(S,250);
	real_T *x252 = (real_T*) ssGetOutputPortSignal(S,251);
	real_T *x253 = (real_T*) ssGetOutputPortSignal(S,252);
	real_T *x254 = (real_T*) ssGetOutputPortSignal(S,253);
	real_T *x255 = (real_T*) ssGetOutputPortSignal(S,254);
	real_T *x256 = (real_T*) ssGetOutputPortSignal(S,255);
	real_T *x257 = (real_T*) ssGetOutputPortSignal(S,256);
	real_T *x258 = (real_T*) ssGetOutputPortSignal(S,257);
	real_T *x259 = (real_T*) ssGetOutputPortSignal(S,258);
	real_T *x260 = (real_T*) ssGetOutputPortSignal(S,259);
	real_T *x261 = (real_T*) ssGetOutputPortSignal(S,260);
	real_T *x262 = (real_T*) ssGetOutputPortSignal(S,261);
	real_T *x263 = (real_T*) ssGetOutputPortSignal(S,262);
	real_T *x264 = (real_T*) ssGetOutputPortSignal(S,263);
	real_T *x265 = (real_T*) ssGetOutputPortSignal(S,264);
	real_T *x266 = (real_T*) ssGetOutputPortSignal(S,265);
	real_T *x267 = (real_T*) ssGetOutputPortSignal(S,266);
	real_T *x268 = (real_T*) ssGetOutputPortSignal(S,267);
	real_T *x269 = (real_T*) ssGetOutputPortSignal(S,268);
	real_T *x270 = (real_T*) ssGetOutputPortSignal(S,269);
	real_T *x271 = (real_T*) ssGetOutputPortSignal(S,270);
	real_T *x272 = (real_T*) ssGetOutputPortSignal(S,271);
	real_T *x273 = (real_T*) ssGetOutputPortSignal(S,272);
	real_T *x274 = (real_T*) ssGetOutputPortSignal(S,273);
	real_T *x275 = (real_T*) ssGetOutputPortSignal(S,274);
	real_T *x276 = (real_T*) ssGetOutputPortSignal(S,275);
	real_T *x277 = (real_T*) ssGetOutputPortSignal(S,276);
	real_T *x278 = (real_T*) ssGetOutputPortSignal(S,277);
	real_T *x279 = (real_T*) ssGetOutputPortSignal(S,278);
	real_T *x280 = (real_T*) ssGetOutputPortSignal(S,279);
	real_T *x281 = (real_T*) ssGetOutputPortSignal(S,280);
	real_T *x282 = (real_T*) ssGetOutputPortSignal(S,281);
	real_T *x283 = (real_T*) ssGetOutputPortSignal(S,282);
	real_T *x284 = (real_T*) ssGetOutputPortSignal(S,283);
	real_T *x285 = (real_T*) ssGetOutputPortSignal(S,284);
	real_T *x286 = (real_T*) ssGetOutputPortSignal(S,285);
	real_T *x287 = (real_T*) ssGetOutputPortSignal(S,286);
	real_T *x288 = (real_T*) ssGetOutputPortSignal(S,287);
	real_T *x289 = (real_T*) ssGetOutputPortSignal(S,288);
	real_T *x290 = (real_T*) ssGetOutputPortSignal(S,289);
	real_T *x291 = (real_T*) ssGetOutputPortSignal(S,290);
	real_T *x292 = (real_T*) ssGetOutputPortSignal(S,291);
	real_T *x293 = (real_T*) ssGetOutputPortSignal(S,292);
	real_T *x294 = (real_T*) ssGetOutputPortSignal(S,293);
	real_T *x295 = (real_T*) ssGetOutputPortSignal(S,294);
	real_T *x296 = (real_T*) ssGetOutputPortSignal(S,295);
	real_T *x297 = (real_T*) ssGetOutputPortSignal(S,296);
	real_T *x298 = (real_T*) ssGetOutputPortSignal(S,297);
	real_T *x299 = (real_T*) ssGetOutputPortSignal(S,298);
	real_T *x300 = (real_T*) ssGetOutputPortSignal(S,299);
	real_T *x301 = (real_T*) ssGetOutputPortSignal(S,300);
	real_T *x302 = (real_T*) ssGetOutputPortSignal(S,301);
	real_T *x303 = (real_T*) ssGetOutputPortSignal(S,302);
	real_T *x304 = (real_T*) ssGetOutputPortSignal(S,303);
	real_T *x305 = (real_T*) ssGetOutputPortSignal(S,304);
	real_T *x306 = (real_T*) ssGetOutputPortSignal(S,305);
	real_T *x307 = (real_T*) ssGetOutputPortSignal(S,306);
	real_T *x308 = (real_T*) ssGetOutputPortSignal(S,307);
	real_T *x309 = (real_T*) ssGetOutputPortSignal(S,308);
	real_T *x310 = (real_T*) ssGetOutputPortSignal(S,309);
	real_T *x311 = (real_T*) ssGetOutputPortSignal(S,310);
	real_T *x312 = (real_T*) ssGetOutputPortSignal(S,311);
	real_T *x313 = (real_T*) ssGetOutputPortSignal(S,312);
	real_T *x314 = (real_T*) ssGetOutputPortSignal(S,313);
	real_T *x315 = (real_T*) ssGetOutputPortSignal(S,314);
	real_T *x316 = (real_T*) ssGetOutputPortSignal(S,315);
	real_T *x317 = (real_T*) ssGetOutputPortSignal(S,316);
	real_T *x318 = (real_T*) ssGetOutputPortSignal(S,317);
	real_T *x319 = (real_T*) ssGetOutputPortSignal(S,318);
	real_T *x320 = (real_T*) ssGetOutputPortSignal(S,319);
	real_T *x321 = (real_T*) ssGetOutputPortSignal(S,320);
	real_T *x322 = (real_T*) ssGetOutputPortSignal(S,321);
	real_T *x323 = (real_T*) ssGetOutputPortSignal(S,322);
	real_T *x324 = (real_T*) ssGetOutputPortSignal(S,323);
	real_T *x325 = (real_T*) ssGetOutputPortSignal(S,324);
	real_T *x326 = (real_T*) ssGetOutputPortSignal(S,325);
	real_T *x327 = (real_T*) ssGetOutputPortSignal(S,326);
	real_T *x328 = (real_T*) ssGetOutputPortSignal(S,327);
	
	

	/* Solver data */
	static CURVATUREsolver_params params;
	static CURVATUREsolver_output output;
	static CURVATUREsolver_info info;	
	solver_int32_default exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<2952; i++)
	{ 
		params.lb[i] = (double) lb[i]; 
	}

	for( i=0; i<2952; i++)
	{ 
		params.ub[i] = (double) ub[i]; 
	}

	for( i=0; i<656; i++)
	{ 
		params.hu[i] = (double) hu[i]; 
	}

	for( i=0; i<656; i++)
	{ 
		params.hl[i] = (double) hl[i]; 
	}

	for( i=0; i<2952; i++)
	{ 
		params.x0[i] = (double) x0[i]; 
	}

	for( i=0; i<7872; i++)
	{ 
		params.all_parameters[i] = (double) all_parameters[i]; 
	}

	

	

    #if SET_PRINTLEVEL_CURVATUREsolver > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* Call solver */
	exitflag = CURVATUREsolver_solve(&params, &output, &info, fp , pt2function_CURVATUREsolver);

	#if SET_PRINTLEVEL_CURVATUREsolver > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			ssPrintf("%c",i);
		}
		fclose(fp);
	#endif

	

	/* Copy outputs */
	for( i=0; i<9; i++)
	{ 
		x001[i] = (real_T) output.x001[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x002[i] = (real_T) output.x002[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x003[i] = (real_T) output.x003[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x004[i] = (real_T) output.x004[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x005[i] = (real_T) output.x005[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x006[i] = (real_T) output.x006[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x007[i] = (real_T) output.x007[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x008[i] = (real_T) output.x008[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x009[i] = (real_T) output.x009[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x010[i] = (real_T) output.x010[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x011[i] = (real_T) output.x011[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x012[i] = (real_T) output.x012[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x013[i] = (real_T) output.x013[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x014[i] = (real_T) output.x014[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x015[i] = (real_T) output.x015[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x016[i] = (real_T) output.x016[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x017[i] = (real_T) output.x017[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x018[i] = (real_T) output.x018[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x019[i] = (real_T) output.x019[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x020[i] = (real_T) output.x020[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x021[i] = (real_T) output.x021[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x022[i] = (real_T) output.x022[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x023[i] = (real_T) output.x023[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x024[i] = (real_T) output.x024[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x025[i] = (real_T) output.x025[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x026[i] = (real_T) output.x026[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x027[i] = (real_T) output.x027[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x028[i] = (real_T) output.x028[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x029[i] = (real_T) output.x029[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x030[i] = (real_T) output.x030[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x031[i] = (real_T) output.x031[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x032[i] = (real_T) output.x032[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x033[i] = (real_T) output.x033[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x034[i] = (real_T) output.x034[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x035[i] = (real_T) output.x035[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x036[i] = (real_T) output.x036[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x037[i] = (real_T) output.x037[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x038[i] = (real_T) output.x038[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x039[i] = (real_T) output.x039[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x040[i] = (real_T) output.x040[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x041[i] = (real_T) output.x041[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x042[i] = (real_T) output.x042[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x043[i] = (real_T) output.x043[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x044[i] = (real_T) output.x044[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x045[i] = (real_T) output.x045[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x046[i] = (real_T) output.x046[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x047[i] = (real_T) output.x047[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x048[i] = (real_T) output.x048[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x049[i] = (real_T) output.x049[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x050[i] = (real_T) output.x050[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x051[i] = (real_T) output.x051[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x052[i] = (real_T) output.x052[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x053[i] = (real_T) output.x053[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x054[i] = (real_T) output.x054[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x055[i] = (real_T) output.x055[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x056[i] = (real_T) output.x056[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x057[i] = (real_T) output.x057[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x058[i] = (real_T) output.x058[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x059[i] = (real_T) output.x059[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x060[i] = (real_T) output.x060[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x061[i] = (real_T) output.x061[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x062[i] = (real_T) output.x062[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x063[i] = (real_T) output.x063[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x064[i] = (real_T) output.x064[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x065[i] = (real_T) output.x065[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x066[i] = (real_T) output.x066[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x067[i] = (real_T) output.x067[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x068[i] = (real_T) output.x068[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x069[i] = (real_T) output.x069[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x070[i] = (real_T) output.x070[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x071[i] = (real_T) output.x071[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x072[i] = (real_T) output.x072[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x073[i] = (real_T) output.x073[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x074[i] = (real_T) output.x074[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x075[i] = (real_T) output.x075[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x076[i] = (real_T) output.x076[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x077[i] = (real_T) output.x077[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x078[i] = (real_T) output.x078[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x079[i] = (real_T) output.x079[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x080[i] = (real_T) output.x080[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x081[i] = (real_T) output.x081[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x082[i] = (real_T) output.x082[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x083[i] = (real_T) output.x083[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x084[i] = (real_T) output.x084[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x085[i] = (real_T) output.x085[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x086[i] = (real_T) output.x086[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x087[i] = (real_T) output.x087[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x088[i] = (real_T) output.x088[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x089[i] = (real_T) output.x089[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x090[i] = (real_T) output.x090[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x091[i] = (real_T) output.x091[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x092[i] = (real_T) output.x092[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x093[i] = (real_T) output.x093[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x094[i] = (real_T) output.x094[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x095[i] = (real_T) output.x095[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x096[i] = (real_T) output.x096[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x097[i] = (real_T) output.x097[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x098[i] = (real_T) output.x098[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x099[i] = (real_T) output.x099[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x100[i] = (real_T) output.x100[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x101[i] = (real_T) output.x101[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x102[i] = (real_T) output.x102[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x103[i] = (real_T) output.x103[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x104[i] = (real_T) output.x104[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x105[i] = (real_T) output.x105[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x106[i] = (real_T) output.x106[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x107[i] = (real_T) output.x107[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x108[i] = (real_T) output.x108[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x109[i] = (real_T) output.x109[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x110[i] = (real_T) output.x110[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x111[i] = (real_T) output.x111[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x112[i] = (real_T) output.x112[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x113[i] = (real_T) output.x113[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x114[i] = (real_T) output.x114[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x115[i] = (real_T) output.x115[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x116[i] = (real_T) output.x116[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x117[i] = (real_T) output.x117[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x118[i] = (real_T) output.x118[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x119[i] = (real_T) output.x119[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x120[i] = (real_T) output.x120[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x121[i] = (real_T) output.x121[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x122[i] = (real_T) output.x122[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x123[i] = (real_T) output.x123[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x124[i] = (real_T) output.x124[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x125[i] = (real_T) output.x125[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x126[i] = (real_T) output.x126[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x127[i] = (real_T) output.x127[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x128[i] = (real_T) output.x128[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x129[i] = (real_T) output.x129[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x130[i] = (real_T) output.x130[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x131[i] = (real_T) output.x131[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x132[i] = (real_T) output.x132[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x133[i] = (real_T) output.x133[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x134[i] = (real_T) output.x134[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x135[i] = (real_T) output.x135[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x136[i] = (real_T) output.x136[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x137[i] = (real_T) output.x137[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x138[i] = (real_T) output.x138[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x139[i] = (real_T) output.x139[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x140[i] = (real_T) output.x140[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x141[i] = (real_T) output.x141[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x142[i] = (real_T) output.x142[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x143[i] = (real_T) output.x143[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x144[i] = (real_T) output.x144[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x145[i] = (real_T) output.x145[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x146[i] = (real_T) output.x146[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x147[i] = (real_T) output.x147[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x148[i] = (real_T) output.x148[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x149[i] = (real_T) output.x149[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x150[i] = (real_T) output.x150[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x151[i] = (real_T) output.x151[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x152[i] = (real_T) output.x152[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x153[i] = (real_T) output.x153[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x154[i] = (real_T) output.x154[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x155[i] = (real_T) output.x155[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x156[i] = (real_T) output.x156[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x157[i] = (real_T) output.x157[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x158[i] = (real_T) output.x158[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x159[i] = (real_T) output.x159[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x160[i] = (real_T) output.x160[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x161[i] = (real_T) output.x161[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x162[i] = (real_T) output.x162[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x163[i] = (real_T) output.x163[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x164[i] = (real_T) output.x164[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x165[i] = (real_T) output.x165[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x166[i] = (real_T) output.x166[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x167[i] = (real_T) output.x167[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x168[i] = (real_T) output.x168[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x169[i] = (real_T) output.x169[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x170[i] = (real_T) output.x170[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x171[i] = (real_T) output.x171[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x172[i] = (real_T) output.x172[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x173[i] = (real_T) output.x173[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x174[i] = (real_T) output.x174[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x175[i] = (real_T) output.x175[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x176[i] = (real_T) output.x176[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x177[i] = (real_T) output.x177[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x178[i] = (real_T) output.x178[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x179[i] = (real_T) output.x179[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x180[i] = (real_T) output.x180[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x181[i] = (real_T) output.x181[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x182[i] = (real_T) output.x182[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x183[i] = (real_T) output.x183[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x184[i] = (real_T) output.x184[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x185[i] = (real_T) output.x185[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x186[i] = (real_T) output.x186[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x187[i] = (real_T) output.x187[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x188[i] = (real_T) output.x188[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x189[i] = (real_T) output.x189[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x190[i] = (real_T) output.x190[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x191[i] = (real_T) output.x191[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x192[i] = (real_T) output.x192[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x193[i] = (real_T) output.x193[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x194[i] = (real_T) output.x194[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x195[i] = (real_T) output.x195[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x196[i] = (real_T) output.x196[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x197[i] = (real_T) output.x197[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x198[i] = (real_T) output.x198[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x199[i] = (real_T) output.x199[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x200[i] = (real_T) output.x200[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x201[i] = (real_T) output.x201[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x202[i] = (real_T) output.x202[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x203[i] = (real_T) output.x203[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x204[i] = (real_T) output.x204[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x205[i] = (real_T) output.x205[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x206[i] = (real_T) output.x206[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x207[i] = (real_T) output.x207[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x208[i] = (real_T) output.x208[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x209[i] = (real_T) output.x209[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x210[i] = (real_T) output.x210[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x211[i] = (real_T) output.x211[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x212[i] = (real_T) output.x212[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x213[i] = (real_T) output.x213[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x214[i] = (real_T) output.x214[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x215[i] = (real_T) output.x215[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x216[i] = (real_T) output.x216[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x217[i] = (real_T) output.x217[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x218[i] = (real_T) output.x218[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x219[i] = (real_T) output.x219[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x220[i] = (real_T) output.x220[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x221[i] = (real_T) output.x221[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x222[i] = (real_T) output.x222[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x223[i] = (real_T) output.x223[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x224[i] = (real_T) output.x224[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x225[i] = (real_T) output.x225[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x226[i] = (real_T) output.x226[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x227[i] = (real_T) output.x227[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x228[i] = (real_T) output.x228[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x229[i] = (real_T) output.x229[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x230[i] = (real_T) output.x230[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x231[i] = (real_T) output.x231[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x232[i] = (real_T) output.x232[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x233[i] = (real_T) output.x233[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x234[i] = (real_T) output.x234[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x235[i] = (real_T) output.x235[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x236[i] = (real_T) output.x236[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x237[i] = (real_T) output.x237[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x238[i] = (real_T) output.x238[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x239[i] = (real_T) output.x239[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x240[i] = (real_T) output.x240[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x241[i] = (real_T) output.x241[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x242[i] = (real_T) output.x242[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x243[i] = (real_T) output.x243[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x244[i] = (real_T) output.x244[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x245[i] = (real_T) output.x245[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x246[i] = (real_T) output.x246[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x247[i] = (real_T) output.x247[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x248[i] = (real_T) output.x248[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x249[i] = (real_T) output.x249[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x250[i] = (real_T) output.x250[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x251[i] = (real_T) output.x251[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x252[i] = (real_T) output.x252[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x253[i] = (real_T) output.x253[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x254[i] = (real_T) output.x254[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x255[i] = (real_T) output.x255[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x256[i] = (real_T) output.x256[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x257[i] = (real_T) output.x257[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x258[i] = (real_T) output.x258[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x259[i] = (real_T) output.x259[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x260[i] = (real_T) output.x260[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x261[i] = (real_T) output.x261[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x262[i] = (real_T) output.x262[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x263[i] = (real_T) output.x263[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x264[i] = (real_T) output.x264[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x265[i] = (real_T) output.x265[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x266[i] = (real_T) output.x266[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x267[i] = (real_T) output.x267[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x268[i] = (real_T) output.x268[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x269[i] = (real_T) output.x269[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x270[i] = (real_T) output.x270[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x271[i] = (real_T) output.x271[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x272[i] = (real_T) output.x272[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x273[i] = (real_T) output.x273[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x274[i] = (real_T) output.x274[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x275[i] = (real_T) output.x275[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x276[i] = (real_T) output.x276[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x277[i] = (real_T) output.x277[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x278[i] = (real_T) output.x278[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x279[i] = (real_T) output.x279[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x280[i] = (real_T) output.x280[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x281[i] = (real_T) output.x281[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x282[i] = (real_T) output.x282[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x283[i] = (real_T) output.x283[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x284[i] = (real_T) output.x284[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x285[i] = (real_T) output.x285[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x286[i] = (real_T) output.x286[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x287[i] = (real_T) output.x287[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x288[i] = (real_T) output.x288[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x289[i] = (real_T) output.x289[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x290[i] = (real_T) output.x290[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x291[i] = (real_T) output.x291[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x292[i] = (real_T) output.x292[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x293[i] = (real_T) output.x293[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x294[i] = (real_T) output.x294[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x295[i] = (real_T) output.x295[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x296[i] = (real_T) output.x296[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x297[i] = (real_T) output.x297[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x298[i] = (real_T) output.x298[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x299[i] = (real_T) output.x299[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x300[i] = (real_T) output.x300[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x301[i] = (real_T) output.x301[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x302[i] = (real_T) output.x302[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x303[i] = (real_T) output.x303[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x304[i] = (real_T) output.x304[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x305[i] = (real_T) output.x305[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x306[i] = (real_T) output.x306[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x307[i] = (real_T) output.x307[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x308[i] = (real_T) output.x308[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x309[i] = (real_T) output.x309[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x310[i] = (real_T) output.x310[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x311[i] = (real_T) output.x311[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x312[i] = (real_T) output.x312[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x313[i] = (real_T) output.x313[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x314[i] = (real_T) output.x314[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x315[i] = (real_T) output.x315[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x316[i] = (real_T) output.x316[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x317[i] = (real_T) output.x317[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x318[i] = (real_T) output.x318[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x319[i] = (real_T) output.x319[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x320[i] = (real_T) output.x320[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x321[i] = (real_T) output.x321[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x322[i] = (real_T) output.x322[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x323[i] = (real_T) output.x323[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x324[i] = (real_T) output.x324[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x325[i] = (real_T) output.x325[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x326[i] = (real_T) output.x326[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x327[i] = (real_T) output.x327[i]; 
	}

	for( i=0; i<9; i++)
	{ 
		x328[i] = (real_T) output.x328[i]; 
	}

	
}





/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif


