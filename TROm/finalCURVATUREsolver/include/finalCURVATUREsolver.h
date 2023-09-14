/*
finalCURVATUREsolver : A fast customized optimization solver.

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

/* Generated by FORCESPRO v5.1.0 on Friday, January 21, 2022 at 5:08:41 PM */
#ifndef finalCURVATUREsolver_H
#define finalCURVATUREsolver_H

#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif


#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif


/* DATA TYPE ------------------------------------------------------------*/
typedef double finalCURVATUREsolver_float;
typedef double finalCURVATUREsolver_ldl_s_float;
typedef double finalCURVATUREsolver_ldl_r_float;
typedef double finalCURVATUREsolver_callback_float;

typedef double finalCURVATUREsolverinterface_float;

/* SOLVER SETTINGS ------------------------------------------------------*/

/* MISRA-C compliance */
#ifndef MISRA_C_finalCURVATUREsolver
#define MISRA_C_finalCURVATUREsolver (0)
#endif

/* restrict code */
#ifndef RESTRICT_CODE_finalCURVATUREsolver
#define RESTRICT_CODE_finalCURVATUREsolver (0)
#endif

/* print level */
#ifndef SET_PRINTLEVEL_finalCURVATUREsolver
#define SET_PRINTLEVEL_finalCURVATUREsolver    (1)
#endif

/* timing */
#ifndef SET_TIMING_finalCURVATUREsolver
#define SET_TIMING_finalCURVATUREsolver    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define SET_MAXIT_finalCURVATUREsolver			(100000)	

/* scaling factor of line search (FTB rule) */
#define SET_FLS_SCALE_finalCURVATUREsolver		(finalCURVATUREsolver_float)(0.99)      

/* maximum number of supported elements in the filter */
#define MAX_FILTER_SIZE_finalCURVATUREsolver	(100000) 

/* maximum number of supported elements in the filter */
#define MAX_SOC_IT_finalCURVATUREsolver			(4) 

/* desired relative duality gap */
#define SET_ACC_RDGAP_finalCURVATUREsolver		(finalCURVATUREsolver_float)(0.0001)

/* desired maximum residual on equality constraints */
#define SET_ACC_RESEQ_finalCURVATUREsolver		(finalCURVATUREsolver_float)(1E-06)

/* desired maximum residual on inequality constraints */
#define SET_ACC_RESINEQ_finalCURVATUREsolver	(finalCURVATUREsolver_float)(1E-06)

/* desired maximum violation of complementarity */
#define SET_ACC_KKTCOMPL_finalCURVATUREsolver	(finalCURVATUREsolver_float)(1E-06)


/* SOLVER RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define OPTIMAL_finalCURVATUREsolver      (1)

/* maximum number of iterations has been reached */
#define MAXITREACHED_finalCURVATUREsolver (0)

/* solver has stopped due to a timeout */
#define TIMEOUT_finalCURVATUREsolver   (2)

/* wrong number of inequalities error */
#define INVALID_NUM_INEQ_ERROR_finalCURVATUREsolver  (-4)

/* factorization error */
#define FACTORIZATION_ERROR_finalCURVATUREsolver   (-5)

/* NaN encountered in function evaluations */
#define BADFUNCEVAL_finalCURVATUREsolver  (-6)

/* no progress in method possible */
#define NOPROGRESS_finalCURVATUREsolver   (-7)

/* regularization error */
#define REGULARIZATION_ERROR_finalCURVATUREsolver   (-9)

/* invalid values in parameters */
#define PARAM_VALUE_ERROR_finalCURVATUREsolver   (-11)

/* too small timeout given */
#define INVALID_TIMEOUT_finalCURVATUREsolver   (-12)

/* thread error */
#define THREAD_FAILURE_finalCURVATUREsolver  (-98)

/* locking mechanism error */
#define LOCK_FAILURE_finalCURVATUREsolver  (-99)

/* licensing error - solver not valid on this machine */
#define LICENSE_ERROR_finalCURVATUREsolver  (-100)

/* INTEGRATORS RETURN CODE ------------*/
/* Integrator ran successfully */
#define INTEGRATOR_SUCCESS (11)
/* Number of steps set by user exceeds maximum number of steps allowed */
#define INTEGRATOR_MAXSTEPS_EXCEEDED (12)





/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct
{
    /* vector of size 713 */
    finalCURVATUREsolver_float lb[713];

    /* vector of size 713 */
    finalCURVATUREsolver_float ub[713];

    /* vector of size 160 */
    finalCURVATUREsolver_float hu[160];

    /* vector of size 160 */
    finalCURVATUREsolver_float hl[160];

    /* vector of size 720 */
    finalCURVATUREsolver_float x0[720];

    /* vector of size 7 */
    finalCURVATUREsolver_float xfinal[7];

    /* vector of size 1840 */
    finalCURVATUREsolver_float all_parameters[1840];


} finalCURVATUREsolver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct
{
    /* vector of size 9 */
    finalCURVATUREsolver_float x01[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x02[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x03[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x04[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x05[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x06[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x07[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x08[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x09[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x10[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x11[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x12[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x13[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x14[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x15[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x16[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x17[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x18[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x19[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x20[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x21[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x22[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x23[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x24[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x25[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x26[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x27[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x28[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x29[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x30[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x31[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x32[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x33[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x34[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x35[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x36[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x37[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x38[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x39[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x40[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x41[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x42[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x43[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x44[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x45[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x46[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x47[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x48[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x49[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x50[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x51[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x52[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x53[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x54[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x55[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x56[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x57[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x58[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x59[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x60[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x61[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x62[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x63[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x64[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x65[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x66[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x67[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x68[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x69[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x70[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x71[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x72[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x73[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x74[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x75[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x76[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x77[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x78[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x79[9];

    /* vector of size 9 */
    finalCURVATUREsolver_float x80[9];


} finalCURVATUREsolver_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct
{
    /* iteration number */
    solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;
	
    /* inf-norm of equality constraint residuals */
    finalCURVATUREsolver_float res_eq;
	
    /* inf-norm of inequality constraint residuals */
    finalCURVATUREsolver_float res_ineq;

	/* norm of stationarity condition */
    finalCURVATUREsolver_float rsnorm;

	/* max of all complementarity violations */
    finalCURVATUREsolver_float rcompnorm;

    /* primal objective */
    finalCURVATUREsolver_float pobj;	
	
    /* dual objective */
    finalCURVATUREsolver_float dobj;	

    /* duality gap := pobj - dobj */
    finalCURVATUREsolver_float dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    finalCURVATUREsolver_float rdgap;		

    /* duality measure */
    finalCURVATUREsolver_float mu;

	/* duality measure (after affine step) */
    finalCURVATUREsolver_float mu_aff;
	
    /* centering parameter */
    finalCURVATUREsolver_float sigma;
	
    /* number of backtracking line search steps (affine direction) */
    solver_int32_default lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    solver_int32_default lsit_cc;
    
    /* step size (affine direction) */
    finalCURVATUREsolver_float step_aff;
    
    /* step size (combined direction) */
    finalCURVATUREsolver_float step_cc;    

	/* solvertime */
	finalCURVATUREsolver_float solvetime;   

	/* time spent in function evaluations */
	finalCURVATUREsolver_float fevalstime;  


} finalCURVATUREsolver_info;









/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* Time of Solver Generation: (UTC) Friday, January 21, 2022 5:08:55 PM */
/* User License expires on: (UTC) Friday, February 4, 2022 10:00:00 PM (approx.) (at the time of code generation) */
/* Solver Static License expires on: (UTC) Friday, February 4, 2022 10:00:00 PM (approx.) */
/* Solver Generation Request Id: a7b6e9ca-4dd3-4ab5-b528-abbd57e0b174 */
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif		

typedef void (*finalCURVATUREsolver_extfunc)(finalCURVATUREsolver_float* x, finalCURVATUREsolver_float* y, finalCURVATUREsolver_float* lambda, finalCURVATUREsolver_float* params, finalCURVATUREsolver_float* pobj, finalCURVATUREsolver_float* g, finalCURVATUREsolver_float* c, finalCURVATUREsolver_float* Jeq, finalCURVATUREsolver_float* h, finalCURVATUREsolver_float* Jineq, finalCURVATUREsolver_float* H, solver_int32_default stage, solver_int32_default iterations, solver_int32_default threadID);

extern solver_int32_default finalCURVATUREsolver_solve(finalCURVATUREsolver_params *params, finalCURVATUREsolver_output *output, finalCURVATUREsolver_info *info, FILE *fs, finalCURVATUREsolver_extfunc evalextfunctions_finalCURVATUREsolver);	













#ifdef __cplusplus
}
#endif

#endif