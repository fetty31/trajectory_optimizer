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

/* Generated by FORCESPRO v5.1.0 on Thursday, February 3, 2022 at 10:27:14 AM */
#ifndef CURVATUREsolver_H
#define CURVATUREsolver_H

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
typedef double CURVATUREsolver_float;
typedef double CURVATUREsolver_ldl_s_float;
typedef double CURVATUREsolver_ldl_r_float;
typedef double CURVATUREsolver_callback_float;

typedef double CURVATUREsolverinterface_float;

/* SOLVER SETTINGS ------------------------------------------------------*/

/* MISRA-C compliance */
#ifndef MISRA_C_CURVATUREsolver
#define MISRA_C_CURVATUREsolver (0)
#endif

/* restrict code */
#ifndef RESTRICT_CODE_CURVATUREsolver
#define RESTRICT_CODE_CURVATUREsolver (0)
#endif

/* print level */
#ifndef SET_PRINTLEVEL_CURVATUREsolver
#define SET_PRINTLEVEL_CURVATUREsolver    (0)
#endif

/* timing */
#ifndef SET_TIMING_CURVATUREsolver
#define SET_TIMING_CURVATUREsolver    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define SET_MAXIT_CURVATUREsolver			(5000)	

/* scaling factor of line search (FTB rule) */
#define SET_FLS_SCALE_CURVATUREsolver		(CURVATUREsolver_float)(0.99)      

/* maximum number of supported elements in the filter */
#define MAX_FILTER_SIZE_CURVATUREsolver	(5000) 

/* maximum number of supported elements in the filter */
#define MAX_SOC_IT_CURVATUREsolver			(4) 

/* desired relative duality gap */
#define SET_ACC_RDGAP_CURVATUREsolver		(CURVATUREsolver_float)(0.0001)

/* desired maximum residual on equality constraints */
#define SET_ACC_RESEQ_CURVATUREsolver		(CURVATUREsolver_float)(1E-06)

/* desired maximum residual on inequality constraints */
#define SET_ACC_RESINEQ_CURVATUREsolver	(CURVATUREsolver_float)(1E-06)

/* desired maximum violation of complementarity */
#define SET_ACC_KKTCOMPL_CURVATUREsolver	(CURVATUREsolver_float)(1E-06)


/* SOLVER RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define OPTIMAL_CURVATUREsolver      (1)

/* maximum number of iterations has been reached */
#define MAXITREACHED_CURVATUREsolver (0)

/* solver has stopped due to a timeout */
#define TIMEOUT_CURVATUREsolver   (2)

/* wrong number of inequalities error */
#define INVALID_NUM_INEQ_ERROR_CURVATUREsolver  (-4)

/* factorization error */
#define FACTORIZATION_ERROR_CURVATUREsolver   (-5)

/* NaN encountered in function evaluations */
#define BADFUNCEVAL_CURVATUREsolver  (-6)

/* no progress in method possible */
#define NOPROGRESS_CURVATUREsolver   (-7)

/* regularization error */
#define REGULARIZATION_ERROR_CURVATUREsolver   (-9)

/* invalid values in parameters */
#define PARAM_VALUE_ERROR_CURVATUREsolver   (-11)

/* too small timeout given */
#define INVALID_TIMEOUT_CURVATUREsolver   (-12)

/* thread error */
#define THREAD_FAILURE_CURVATUREsolver  (-98)

/* locking mechanism error */
#define LOCK_FAILURE_CURVATUREsolver  (-99)

/* licensing error - solver not valid on this machine */
#define LICENSE_ERROR_CURVATUREsolver  (-100)

/* INTEGRATORS RETURN CODE ------------*/
/* Integrator ran successfully */
#define INTEGRATOR_SUCCESS (11)
/* Number of steps set by user exceeds maximum number of steps allowed */
#define INTEGRATOR_MAXSTEPS_EXCEEDED (12)





/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct
{
    /* vector of size 2952 */
    CURVATUREsolver_float lb[2952];

    /* vector of size 2952 */
    CURVATUREsolver_float ub[2952];

    /* vector of size 656 */
    CURVATUREsolver_float hu[656];

    /* vector of size 656 */
    CURVATUREsolver_float hl[656];

    /* vector of size 2952 */
    CURVATUREsolver_float x0[2952];

    /* vector of size 7872 */
    CURVATUREsolver_float all_parameters[7872];


} CURVATUREsolver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct
{
    /* vector of size 9 */
    CURVATUREsolver_float x001[9];

    /* vector of size 9 */
    CURVATUREsolver_float x002[9];

    /* vector of size 9 */
    CURVATUREsolver_float x003[9];

    /* vector of size 9 */
    CURVATUREsolver_float x004[9];

    /* vector of size 9 */
    CURVATUREsolver_float x005[9];

    /* vector of size 9 */
    CURVATUREsolver_float x006[9];

    /* vector of size 9 */
    CURVATUREsolver_float x007[9];

    /* vector of size 9 */
    CURVATUREsolver_float x008[9];

    /* vector of size 9 */
    CURVATUREsolver_float x009[9];

    /* vector of size 9 */
    CURVATUREsolver_float x010[9];

    /* vector of size 9 */
    CURVATUREsolver_float x011[9];

    /* vector of size 9 */
    CURVATUREsolver_float x012[9];

    /* vector of size 9 */
    CURVATUREsolver_float x013[9];

    /* vector of size 9 */
    CURVATUREsolver_float x014[9];

    /* vector of size 9 */
    CURVATUREsolver_float x015[9];

    /* vector of size 9 */
    CURVATUREsolver_float x016[9];

    /* vector of size 9 */
    CURVATUREsolver_float x017[9];

    /* vector of size 9 */
    CURVATUREsolver_float x018[9];

    /* vector of size 9 */
    CURVATUREsolver_float x019[9];

    /* vector of size 9 */
    CURVATUREsolver_float x020[9];

    /* vector of size 9 */
    CURVATUREsolver_float x021[9];

    /* vector of size 9 */
    CURVATUREsolver_float x022[9];

    /* vector of size 9 */
    CURVATUREsolver_float x023[9];

    /* vector of size 9 */
    CURVATUREsolver_float x024[9];

    /* vector of size 9 */
    CURVATUREsolver_float x025[9];

    /* vector of size 9 */
    CURVATUREsolver_float x026[9];

    /* vector of size 9 */
    CURVATUREsolver_float x027[9];

    /* vector of size 9 */
    CURVATUREsolver_float x028[9];

    /* vector of size 9 */
    CURVATUREsolver_float x029[9];

    /* vector of size 9 */
    CURVATUREsolver_float x030[9];

    /* vector of size 9 */
    CURVATUREsolver_float x031[9];

    /* vector of size 9 */
    CURVATUREsolver_float x032[9];

    /* vector of size 9 */
    CURVATUREsolver_float x033[9];

    /* vector of size 9 */
    CURVATUREsolver_float x034[9];

    /* vector of size 9 */
    CURVATUREsolver_float x035[9];

    /* vector of size 9 */
    CURVATUREsolver_float x036[9];

    /* vector of size 9 */
    CURVATUREsolver_float x037[9];

    /* vector of size 9 */
    CURVATUREsolver_float x038[9];

    /* vector of size 9 */
    CURVATUREsolver_float x039[9];

    /* vector of size 9 */
    CURVATUREsolver_float x040[9];

    /* vector of size 9 */
    CURVATUREsolver_float x041[9];

    /* vector of size 9 */
    CURVATUREsolver_float x042[9];

    /* vector of size 9 */
    CURVATUREsolver_float x043[9];

    /* vector of size 9 */
    CURVATUREsolver_float x044[9];

    /* vector of size 9 */
    CURVATUREsolver_float x045[9];

    /* vector of size 9 */
    CURVATUREsolver_float x046[9];

    /* vector of size 9 */
    CURVATUREsolver_float x047[9];

    /* vector of size 9 */
    CURVATUREsolver_float x048[9];

    /* vector of size 9 */
    CURVATUREsolver_float x049[9];

    /* vector of size 9 */
    CURVATUREsolver_float x050[9];

    /* vector of size 9 */
    CURVATUREsolver_float x051[9];

    /* vector of size 9 */
    CURVATUREsolver_float x052[9];

    /* vector of size 9 */
    CURVATUREsolver_float x053[9];

    /* vector of size 9 */
    CURVATUREsolver_float x054[9];

    /* vector of size 9 */
    CURVATUREsolver_float x055[9];

    /* vector of size 9 */
    CURVATUREsolver_float x056[9];

    /* vector of size 9 */
    CURVATUREsolver_float x057[9];

    /* vector of size 9 */
    CURVATUREsolver_float x058[9];

    /* vector of size 9 */
    CURVATUREsolver_float x059[9];

    /* vector of size 9 */
    CURVATUREsolver_float x060[9];

    /* vector of size 9 */
    CURVATUREsolver_float x061[9];

    /* vector of size 9 */
    CURVATUREsolver_float x062[9];

    /* vector of size 9 */
    CURVATUREsolver_float x063[9];

    /* vector of size 9 */
    CURVATUREsolver_float x064[9];

    /* vector of size 9 */
    CURVATUREsolver_float x065[9];

    /* vector of size 9 */
    CURVATUREsolver_float x066[9];

    /* vector of size 9 */
    CURVATUREsolver_float x067[9];

    /* vector of size 9 */
    CURVATUREsolver_float x068[9];

    /* vector of size 9 */
    CURVATUREsolver_float x069[9];

    /* vector of size 9 */
    CURVATUREsolver_float x070[9];

    /* vector of size 9 */
    CURVATUREsolver_float x071[9];

    /* vector of size 9 */
    CURVATUREsolver_float x072[9];

    /* vector of size 9 */
    CURVATUREsolver_float x073[9];

    /* vector of size 9 */
    CURVATUREsolver_float x074[9];

    /* vector of size 9 */
    CURVATUREsolver_float x075[9];

    /* vector of size 9 */
    CURVATUREsolver_float x076[9];

    /* vector of size 9 */
    CURVATUREsolver_float x077[9];

    /* vector of size 9 */
    CURVATUREsolver_float x078[9];

    /* vector of size 9 */
    CURVATUREsolver_float x079[9];

    /* vector of size 9 */
    CURVATUREsolver_float x080[9];

    /* vector of size 9 */
    CURVATUREsolver_float x081[9];

    /* vector of size 9 */
    CURVATUREsolver_float x082[9];

    /* vector of size 9 */
    CURVATUREsolver_float x083[9];

    /* vector of size 9 */
    CURVATUREsolver_float x084[9];

    /* vector of size 9 */
    CURVATUREsolver_float x085[9];

    /* vector of size 9 */
    CURVATUREsolver_float x086[9];

    /* vector of size 9 */
    CURVATUREsolver_float x087[9];

    /* vector of size 9 */
    CURVATUREsolver_float x088[9];

    /* vector of size 9 */
    CURVATUREsolver_float x089[9];

    /* vector of size 9 */
    CURVATUREsolver_float x090[9];

    /* vector of size 9 */
    CURVATUREsolver_float x091[9];

    /* vector of size 9 */
    CURVATUREsolver_float x092[9];

    /* vector of size 9 */
    CURVATUREsolver_float x093[9];

    /* vector of size 9 */
    CURVATUREsolver_float x094[9];

    /* vector of size 9 */
    CURVATUREsolver_float x095[9];

    /* vector of size 9 */
    CURVATUREsolver_float x096[9];

    /* vector of size 9 */
    CURVATUREsolver_float x097[9];

    /* vector of size 9 */
    CURVATUREsolver_float x098[9];

    /* vector of size 9 */
    CURVATUREsolver_float x099[9];

    /* vector of size 9 */
    CURVATUREsolver_float x100[9];

    /* vector of size 9 */
    CURVATUREsolver_float x101[9];

    /* vector of size 9 */
    CURVATUREsolver_float x102[9];

    /* vector of size 9 */
    CURVATUREsolver_float x103[9];

    /* vector of size 9 */
    CURVATUREsolver_float x104[9];

    /* vector of size 9 */
    CURVATUREsolver_float x105[9];

    /* vector of size 9 */
    CURVATUREsolver_float x106[9];

    /* vector of size 9 */
    CURVATUREsolver_float x107[9];

    /* vector of size 9 */
    CURVATUREsolver_float x108[9];

    /* vector of size 9 */
    CURVATUREsolver_float x109[9];

    /* vector of size 9 */
    CURVATUREsolver_float x110[9];

    /* vector of size 9 */
    CURVATUREsolver_float x111[9];

    /* vector of size 9 */
    CURVATUREsolver_float x112[9];

    /* vector of size 9 */
    CURVATUREsolver_float x113[9];

    /* vector of size 9 */
    CURVATUREsolver_float x114[9];

    /* vector of size 9 */
    CURVATUREsolver_float x115[9];

    /* vector of size 9 */
    CURVATUREsolver_float x116[9];

    /* vector of size 9 */
    CURVATUREsolver_float x117[9];

    /* vector of size 9 */
    CURVATUREsolver_float x118[9];

    /* vector of size 9 */
    CURVATUREsolver_float x119[9];

    /* vector of size 9 */
    CURVATUREsolver_float x120[9];

    /* vector of size 9 */
    CURVATUREsolver_float x121[9];

    /* vector of size 9 */
    CURVATUREsolver_float x122[9];

    /* vector of size 9 */
    CURVATUREsolver_float x123[9];

    /* vector of size 9 */
    CURVATUREsolver_float x124[9];

    /* vector of size 9 */
    CURVATUREsolver_float x125[9];

    /* vector of size 9 */
    CURVATUREsolver_float x126[9];

    /* vector of size 9 */
    CURVATUREsolver_float x127[9];

    /* vector of size 9 */
    CURVATUREsolver_float x128[9];

    /* vector of size 9 */
    CURVATUREsolver_float x129[9];

    /* vector of size 9 */
    CURVATUREsolver_float x130[9];

    /* vector of size 9 */
    CURVATUREsolver_float x131[9];

    /* vector of size 9 */
    CURVATUREsolver_float x132[9];

    /* vector of size 9 */
    CURVATUREsolver_float x133[9];

    /* vector of size 9 */
    CURVATUREsolver_float x134[9];

    /* vector of size 9 */
    CURVATUREsolver_float x135[9];

    /* vector of size 9 */
    CURVATUREsolver_float x136[9];

    /* vector of size 9 */
    CURVATUREsolver_float x137[9];

    /* vector of size 9 */
    CURVATUREsolver_float x138[9];

    /* vector of size 9 */
    CURVATUREsolver_float x139[9];

    /* vector of size 9 */
    CURVATUREsolver_float x140[9];

    /* vector of size 9 */
    CURVATUREsolver_float x141[9];

    /* vector of size 9 */
    CURVATUREsolver_float x142[9];

    /* vector of size 9 */
    CURVATUREsolver_float x143[9];

    /* vector of size 9 */
    CURVATUREsolver_float x144[9];

    /* vector of size 9 */
    CURVATUREsolver_float x145[9];

    /* vector of size 9 */
    CURVATUREsolver_float x146[9];

    /* vector of size 9 */
    CURVATUREsolver_float x147[9];

    /* vector of size 9 */
    CURVATUREsolver_float x148[9];

    /* vector of size 9 */
    CURVATUREsolver_float x149[9];

    /* vector of size 9 */
    CURVATUREsolver_float x150[9];

    /* vector of size 9 */
    CURVATUREsolver_float x151[9];

    /* vector of size 9 */
    CURVATUREsolver_float x152[9];

    /* vector of size 9 */
    CURVATUREsolver_float x153[9];

    /* vector of size 9 */
    CURVATUREsolver_float x154[9];

    /* vector of size 9 */
    CURVATUREsolver_float x155[9];

    /* vector of size 9 */
    CURVATUREsolver_float x156[9];

    /* vector of size 9 */
    CURVATUREsolver_float x157[9];

    /* vector of size 9 */
    CURVATUREsolver_float x158[9];

    /* vector of size 9 */
    CURVATUREsolver_float x159[9];

    /* vector of size 9 */
    CURVATUREsolver_float x160[9];

    /* vector of size 9 */
    CURVATUREsolver_float x161[9];

    /* vector of size 9 */
    CURVATUREsolver_float x162[9];

    /* vector of size 9 */
    CURVATUREsolver_float x163[9];

    /* vector of size 9 */
    CURVATUREsolver_float x164[9];

    /* vector of size 9 */
    CURVATUREsolver_float x165[9];

    /* vector of size 9 */
    CURVATUREsolver_float x166[9];

    /* vector of size 9 */
    CURVATUREsolver_float x167[9];

    /* vector of size 9 */
    CURVATUREsolver_float x168[9];

    /* vector of size 9 */
    CURVATUREsolver_float x169[9];

    /* vector of size 9 */
    CURVATUREsolver_float x170[9];

    /* vector of size 9 */
    CURVATUREsolver_float x171[9];

    /* vector of size 9 */
    CURVATUREsolver_float x172[9];

    /* vector of size 9 */
    CURVATUREsolver_float x173[9];

    /* vector of size 9 */
    CURVATUREsolver_float x174[9];

    /* vector of size 9 */
    CURVATUREsolver_float x175[9];

    /* vector of size 9 */
    CURVATUREsolver_float x176[9];

    /* vector of size 9 */
    CURVATUREsolver_float x177[9];

    /* vector of size 9 */
    CURVATUREsolver_float x178[9];

    /* vector of size 9 */
    CURVATUREsolver_float x179[9];

    /* vector of size 9 */
    CURVATUREsolver_float x180[9];

    /* vector of size 9 */
    CURVATUREsolver_float x181[9];

    /* vector of size 9 */
    CURVATUREsolver_float x182[9];

    /* vector of size 9 */
    CURVATUREsolver_float x183[9];

    /* vector of size 9 */
    CURVATUREsolver_float x184[9];

    /* vector of size 9 */
    CURVATUREsolver_float x185[9];

    /* vector of size 9 */
    CURVATUREsolver_float x186[9];

    /* vector of size 9 */
    CURVATUREsolver_float x187[9];

    /* vector of size 9 */
    CURVATUREsolver_float x188[9];

    /* vector of size 9 */
    CURVATUREsolver_float x189[9];

    /* vector of size 9 */
    CURVATUREsolver_float x190[9];

    /* vector of size 9 */
    CURVATUREsolver_float x191[9];

    /* vector of size 9 */
    CURVATUREsolver_float x192[9];

    /* vector of size 9 */
    CURVATUREsolver_float x193[9];

    /* vector of size 9 */
    CURVATUREsolver_float x194[9];

    /* vector of size 9 */
    CURVATUREsolver_float x195[9];

    /* vector of size 9 */
    CURVATUREsolver_float x196[9];

    /* vector of size 9 */
    CURVATUREsolver_float x197[9];

    /* vector of size 9 */
    CURVATUREsolver_float x198[9];

    /* vector of size 9 */
    CURVATUREsolver_float x199[9];

    /* vector of size 9 */
    CURVATUREsolver_float x200[9];

    /* vector of size 9 */
    CURVATUREsolver_float x201[9];

    /* vector of size 9 */
    CURVATUREsolver_float x202[9];

    /* vector of size 9 */
    CURVATUREsolver_float x203[9];

    /* vector of size 9 */
    CURVATUREsolver_float x204[9];

    /* vector of size 9 */
    CURVATUREsolver_float x205[9];

    /* vector of size 9 */
    CURVATUREsolver_float x206[9];

    /* vector of size 9 */
    CURVATUREsolver_float x207[9];

    /* vector of size 9 */
    CURVATUREsolver_float x208[9];

    /* vector of size 9 */
    CURVATUREsolver_float x209[9];

    /* vector of size 9 */
    CURVATUREsolver_float x210[9];

    /* vector of size 9 */
    CURVATUREsolver_float x211[9];

    /* vector of size 9 */
    CURVATUREsolver_float x212[9];

    /* vector of size 9 */
    CURVATUREsolver_float x213[9];

    /* vector of size 9 */
    CURVATUREsolver_float x214[9];

    /* vector of size 9 */
    CURVATUREsolver_float x215[9];

    /* vector of size 9 */
    CURVATUREsolver_float x216[9];

    /* vector of size 9 */
    CURVATUREsolver_float x217[9];

    /* vector of size 9 */
    CURVATUREsolver_float x218[9];

    /* vector of size 9 */
    CURVATUREsolver_float x219[9];

    /* vector of size 9 */
    CURVATUREsolver_float x220[9];

    /* vector of size 9 */
    CURVATUREsolver_float x221[9];

    /* vector of size 9 */
    CURVATUREsolver_float x222[9];

    /* vector of size 9 */
    CURVATUREsolver_float x223[9];

    /* vector of size 9 */
    CURVATUREsolver_float x224[9];

    /* vector of size 9 */
    CURVATUREsolver_float x225[9];

    /* vector of size 9 */
    CURVATUREsolver_float x226[9];

    /* vector of size 9 */
    CURVATUREsolver_float x227[9];

    /* vector of size 9 */
    CURVATUREsolver_float x228[9];

    /* vector of size 9 */
    CURVATUREsolver_float x229[9];

    /* vector of size 9 */
    CURVATUREsolver_float x230[9];

    /* vector of size 9 */
    CURVATUREsolver_float x231[9];

    /* vector of size 9 */
    CURVATUREsolver_float x232[9];

    /* vector of size 9 */
    CURVATUREsolver_float x233[9];

    /* vector of size 9 */
    CURVATUREsolver_float x234[9];

    /* vector of size 9 */
    CURVATUREsolver_float x235[9];

    /* vector of size 9 */
    CURVATUREsolver_float x236[9];

    /* vector of size 9 */
    CURVATUREsolver_float x237[9];

    /* vector of size 9 */
    CURVATUREsolver_float x238[9];

    /* vector of size 9 */
    CURVATUREsolver_float x239[9];

    /* vector of size 9 */
    CURVATUREsolver_float x240[9];

    /* vector of size 9 */
    CURVATUREsolver_float x241[9];

    /* vector of size 9 */
    CURVATUREsolver_float x242[9];

    /* vector of size 9 */
    CURVATUREsolver_float x243[9];

    /* vector of size 9 */
    CURVATUREsolver_float x244[9];

    /* vector of size 9 */
    CURVATUREsolver_float x245[9];

    /* vector of size 9 */
    CURVATUREsolver_float x246[9];

    /* vector of size 9 */
    CURVATUREsolver_float x247[9];

    /* vector of size 9 */
    CURVATUREsolver_float x248[9];

    /* vector of size 9 */
    CURVATUREsolver_float x249[9];

    /* vector of size 9 */
    CURVATUREsolver_float x250[9];

    /* vector of size 9 */
    CURVATUREsolver_float x251[9];

    /* vector of size 9 */
    CURVATUREsolver_float x252[9];

    /* vector of size 9 */
    CURVATUREsolver_float x253[9];

    /* vector of size 9 */
    CURVATUREsolver_float x254[9];

    /* vector of size 9 */
    CURVATUREsolver_float x255[9];

    /* vector of size 9 */
    CURVATUREsolver_float x256[9];

    /* vector of size 9 */
    CURVATUREsolver_float x257[9];

    /* vector of size 9 */
    CURVATUREsolver_float x258[9];

    /* vector of size 9 */
    CURVATUREsolver_float x259[9];

    /* vector of size 9 */
    CURVATUREsolver_float x260[9];

    /* vector of size 9 */
    CURVATUREsolver_float x261[9];

    /* vector of size 9 */
    CURVATUREsolver_float x262[9];

    /* vector of size 9 */
    CURVATUREsolver_float x263[9];

    /* vector of size 9 */
    CURVATUREsolver_float x264[9];

    /* vector of size 9 */
    CURVATUREsolver_float x265[9];

    /* vector of size 9 */
    CURVATUREsolver_float x266[9];

    /* vector of size 9 */
    CURVATUREsolver_float x267[9];

    /* vector of size 9 */
    CURVATUREsolver_float x268[9];

    /* vector of size 9 */
    CURVATUREsolver_float x269[9];

    /* vector of size 9 */
    CURVATUREsolver_float x270[9];

    /* vector of size 9 */
    CURVATUREsolver_float x271[9];

    /* vector of size 9 */
    CURVATUREsolver_float x272[9];

    /* vector of size 9 */
    CURVATUREsolver_float x273[9];

    /* vector of size 9 */
    CURVATUREsolver_float x274[9];

    /* vector of size 9 */
    CURVATUREsolver_float x275[9];

    /* vector of size 9 */
    CURVATUREsolver_float x276[9];

    /* vector of size 9 */
    CURVATUREsolver_float x277[9];

    /* vector of size 9 */
    CURVATUREsolver_float x278[9];

    /* vector of size 9 */
    CURVATUREsolver_float x279[9];

    /* vector of size 9 */
    CURVATUREsolver_float x280[9];

    /* vector of size 9 */
    CURVATUREsolver_float x281[9];

    /* vector of size 9 */
    CURVATUREsolver_float x282[9];

    /* vector of size 9 */
    CURVATUREsolver_float x283[9];

    /* vector of size 9 */
    CURVATUREsolver_float x284[9];

    /* vector of size 9 */
    CURVATUREsolver_float x285[9];

    /* vector of size 9 */
    CURVATUREsolver_float x286[9];

    /* vector of size 9 */
    CURVATUREsolver_float x287[9];

    /* vector of size 9 */
    CURVATUREsolver_float x288[9];

    /* vector of size 9 */
    CURVATUREsolver_float x289[9];

    /* vector of size 9 */
    CURVATUREsolver_float x290[9];

    /* vector of size 9 */
    CURVATUREsolver_float x291[9];

    /* vector of size 9 */
    CURVATUREsolver_float x292[9];

    /* vector of size 9 */
    CURVATUREsolver_float x293[9];

    /* vector of size 9 */
    CURVATUREsolver_float x294[9];

    /* vector of size 9 */
    CURVATUREsolver_float x295[9];

    /* vector of size 9 */
    CURVATUREsolver_float x296[9];

    /* vector of size 9 */
    CURVATUREsolver_float x297[9];

    /* vector of size 9 */
    CURVATUREsolver_float x298[9];

    /* vector of size 9 */
    CURVATUREsolver_float x299[9];

    /* vector of size 9 */
    CURVATUREsolver_float x300[9];

    /* vector of size 9 */
    CURVATUREsolver_float x301[9];

    /* vector of size 9 */
    CURVATUREsolver_float x302[9];

    /* vector of size 9 */
    CURVATUREsolver_float x303[9];

    /* vector of size 9 */
    CURVATUREsolver_float x304[9];

    /* vector of size 9 */
    CURVATUREsolver_float x305[9];

    /* vector of size 9 */
    CURVATUREsolver_float x306[9];

    /* vector of size 9 */
    CURVATUREsolver_float x307[9];

    /* vector of size 9 */
    CURVATUREsolver_float x308[9];

    /* vector of size 9 */
    CURVATUREsolver_float x309[9];

    /* vector of size 9 */
    CURVATUREsolver_float x310[9];

    /* vector of size 9 */
    CURVATUREsolver_float x311[9];

    /* vector of size 9 */
    CURVATUREsolver_float x312[9];

    /* vector of size 9 */
    CURVATUREsolver_float x313[9];

    /* vector of size 9 */
    CURVATUREsolver_float x314[9];

    /* vector of size 9 */
    CURVATUREsolver_float x315[9];

    /* vector of size 9 */
    CURVATUREsolver_float x316[9];

    /* vector of size 9 */
    CURVATUREsolver_float x317[9];

    /* vector of size 9 */
    CURVATUREsolver_float x318[9];

    /* vector of size 9 */
    CURVATUREsolver_float x319[9];

    /* vector of size 9 */
    CURVATUREsolver_float x320[9];

    /* vector of size 9 */
    CURVATUREsolver_float x321[9];

    /* vector of size 9 */
    CURVATUREsolver_float x322[9];

    /* vector of size 9 */
    CURVATUREsolver_float x323[9];

    /* vector of size 9 */
    CURVATUREsolver_float x324[9];

    /* vector of size 9 */
    CURVATUREsolver_float x325[9];

    /* vector of size 9 */
    CURVATUREsolver_float x326[9];

    /* vector of size 9 */
    CURVATUREsolver_float x327[9];

    /* vector of size 9 */
    CURVATUREsolver_float x328[9];


} CURVATUREsolver_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct
{
    /* iteration number */
    solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;
	
    /* inf-norm of equality constraint residuals */
    CURVATUREsolver_float res_eq;
	
    /* inf-norm of inequality constraint residuals */
    CURVATUREsolver_float res_ineq;

	/* norm of stationarity condition */
    CURVATUREsolver_float rsnorm;

	/* max of all complementarity violations */
    CURVATUREsolver_float rcompnorm;

    /* primal objective */
    CURVATUREsolver_float pobj;	
	
    /* dual objective */
    CURVATUREsolver_float dobj;	

    /* duality gap := pobj - dobj */
    CURVATUREsolver_float dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    CURVATUREsolver_float rdgap;		

    /* duality measure */
    CURVATUREsolver_float mu;

	/* duality measure (after affine step) */
    CURVATUREsolver_float mu_aff;
	
    /* centering parameter */
    CURVATUREsolver_float sigma;
	
    /* number of backtracking line search steps (affine direction) */
    solver_int32_default lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    solver_int32_default lsit_cc;
    
    /* step size (affine direction) */
    CURVATUREsolver_float step_aff;
    
    /* step size (combined direction) */
    CURVATUREsolver_float step_cc;    

	/* solvertime */
	CURVATUREsolver_float solvetime;   

	/* time spent in function evaluations */
	CURVATUREsolver_float fevalstime;  


} CURVATUREsolver_info;









/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* Time of Solver Generation: (UTC) Thursday, February 3, 2022 10:28:16 AM */
/* User License expires on: (UTC) Friday, February 4, 2022 10:00:00 PM (approx.) (at the time of code generation) */
/* Solver Static License expires on: (UTC) Friday, February 4, 2022 10:00:00 PM (approx.) */
/* Solver Generation Request Id: a3fb5412-b797-419f-9447-b081a78d78e7 */
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif		

typedef void (*CURVATUREsolver_extfunc)(CURVATUREsolver_float* x, CURVATUREsolver_float* y, CURVATUREsolver_float* lambda, CURVATUREsolver_float* params, CURVATUREsolver_float* pobj, CURVATUREsolver_float* g, CURVATUREsolver_float* c, CURVATUREsolver_float* Jeq, CURVATUREsolver_float* h, CURVATUREsolver_float* Jineq, CURVATUREsolver_float* H, solver_int32_default stage, solver_int32_default iterations, solver_int32_default threadID);

extern solver_int32_default CURVATUREsolver_solve(CURVATUREsolver_params *params, CURVATUREsolver_output *output, CURVATUREsolver_info *info, FILE *fs, CURVATUREsolver_extfunc evalextfunctions_CURVATUREsolver);	













#ifdef __cplusplus
}
#endif

#endif
