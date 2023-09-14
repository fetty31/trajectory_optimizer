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


#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME finalCURVATUREsolver_simulinkBlockcompact

#include "simstruc.h"



/* include FORCESPRO functions and defs */
#include "../include/finalCURVATUREsolver.h" 

/* SYSTEM INCLUDES FOR TIMING ------------------------------------------ */


#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

typedef finalCURVATUREsolverinterface_float finalCURVATUREsolvernmpc_float;

extern void finalCURVATUREsolver_casadi2forces(double *x, double *y, double *l, double *p, double *f, double *nabla_f, double *c, double *nabla_c, double *h, double *nabla_h, double *hess, solver_int32_default stage, solver_int32_default iteration, solver_int32_default threadID);
finalCURVATUREsolver_extfunc pt2function_finalCURVATUREsolver = &finalCURVATUREsolver_casadi2forces;




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

	/* initialize input ports - there are 7 in total */
    if (!ssSetNumInputPorts(S, 7)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 160, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 160, 1);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/
	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 713, 1);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/
	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 713, 1);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/
	
	/* Input Port 4 */
    ssSetInputPortMatrixDimensions(S,  4, 720, 1);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 4, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/
	
	/* Input Port 5 */
    ssSetInputPortMatrixDimensions(S,  5, 7, 1);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 5, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 5, 1); /*direct input signal access*/
	
	/* Input Port 6 */
    ssSetInputPortMatrixDimensions(S,  6, 1840, 1);
    ssSetInputPortDataType(S, 6, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 6, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 6, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 6, 1); /*direct input signal access*/
 


	/* initialize output ports - there are 1 in total */
    if (!ssSetNumOutputPorts(S, 1)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 720, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */


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
	const real_T *hu = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *hl = (const real_T*) ssGetInputPortSignal(S,1);
	const real_T *lb = (const real_T*) ssGetInputPortSignal(S,2);
	const real_T *ub = (const real_T*) ssGetInputPortSignal(S,3);
	const real_T *x0 = (const real_T*) ssGetInputPortSignal(S,4);
	const real_T *xfinal = (const real_T*) ssGetInputPortSignal(S,5);
	const real_T *all_parameters = (const real_T*) ssGetInputPortSignal(S,6);
	
    real_T *outputs = (real_T*) ssGetOutputPortSignal(S,0);
	
	

	/* Solver data */
	static finalCURVATUREsolver_params params;
	static finalCURVATUREsolver_output output;
	static finalCURVATUREsolver_info info;	
	solver_int32_default exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<713; i++)
	{ 
		params.lb[i] = (double) lb[i]; 
	}

	for( i=0; i<713; i++)
	{ 
		params.ub[i] = (double) ub[i]; 
	}

	for( i=0; i<160; i++)
	{ 
		params.hu[i] = (double) hu[i]; 
	}

	for( i=0; i<160; i++)
	{ 
		params.hl[i] = (double) hl[i]; 
	}

	for( i=0; i<720; i++)
	{ 
		params.x0[i] = (double) x0[i]; 
	}

	for( i=0; i<7; i++)
	{ 
		params.xfinal[i] = (double) xfinal[i]; 
	}

	for( i=0; i<1840; i++)
	{ 
		params.all_parameters[i] = (double) all_parameters[i]; 
	}

	

	

    #if SET_PRINTLEVEL_finalCURVATUREsolver > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* Call solver */
	exitflag = finalCURVATUREsolver_solve(&params, &output, &info, fp , pt2function_finalCURVATUREsolver);

	#if SET_PRINTLEVEL_finalCURVATUREsolver > 0
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
		outputs[i] = (real_T) output.x01[i]; 
	}

	k=9; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x02[i]; 
	}

	k=18; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x03[i]; 
	}

	k=27; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x04[i]; 
	}

	k=36; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x05[i]; 
	}

	k=45; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x06[i]; 
	}

	k=54; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x07[i]; 
	}

	k=63; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x08[i]; 
	}

	k=72; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x09[i]; 
	}

	k=81; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x10[i]; 
	}

	k=90; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x11[i]; 
	}

	k=99; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x12[i]; 
	}

	k=108; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x13[i]; 
	}

	k=117; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x14[i]; 
	}

	k=126; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x15[i]; 
	}

	k=135; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x16[i]; 
	}

	k=144; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x17[i]; 
	}

	k=153; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x18[i]; 
	}

	k=162; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x19[i]; 
	}

	k=171; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x20[i]; 
	}

	k=180; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x21[i]; 
	}

	k=189; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x22[i]; 
	}

	k=198; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x23[i]; 
	}

	k=207; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x24[i]; 
	}

	k=216; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x25[i]; 
	}

	k=225; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x26[i]; 
	}

	k=234; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x27[i]; 
	}

	k=243; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x28[i]; 
	}

	k=252; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x29[i]; 
	}

	k=261; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x30[i]; 
	}

	k=270; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x31[i]; 
	}

	k=279; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x32[i]; 
	}

	k=288; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x33[i]; 
	}

	k=297; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x34[i]; 
	}

	k=306; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x35[i]; 
	}

	k=315; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x36[i]; 
	}

	k=324; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x37[i]; 
	}

	k=333; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x38[i]; 
	}

	k=342; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x39[i]; 
	}

	k=351; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x40[i]; 
	}

	k=360; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x41[i]; 
	}

	k=369; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x42[i]; 
	}

	k=378; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x43[i]; 
	}

	k=387; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x44[i]; 
	}

	k=396; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x45[i]; 
	}

	k=405; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x46[i]; 
	}

	k=414; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x47[i]; 
	}

	k=423; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x48[i]; 
	}

	k=432; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x49[i]; 
	}

	k=441; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x50[i]; 
	}

	k=450; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x51[i]; 
	}

	k=459; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x52[i]; 
	}

	k=468; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x53[i]; 
	}

	k=477; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x54[i]; 
	}

	k=486; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x55[i]; 
	}

	k=495; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x56[i]; 
	}

	k=504; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x57[i]; 
	}

	k=513; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x58[i]; 
	}

	k=522; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x59[i]; 
	}

	k=531; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x60[i]; 
	}

	k=540; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x61[i]; 
	}

	k=549; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x62[i]; 
	}

	k=558; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x63[i]; 
	}

	k=567; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x64[i]; 
	}

	k=576; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x65[i]; 
	}

	k=585; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x66[i]; 
	}

	k=594; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x67[i]; 
	}

	k=603; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x68[i]; 
	}

	k=612; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x69[i]; 
	}

	k=621; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x70[i]; 
	}

	k=630; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x71[i]; 
	}

	k=639; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x72[i]; 
	}

	k=648; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x73[i]; 
	}

	k=657; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x74[i]; 
	}

	k=666; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x75[i]; 
	}

	k=675; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x76[i]; 
	}

	k=684; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x77[i]; 
	}

	k=693; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x78[i]; 
	}

	k=702; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x79[i]; 
	}

	k=711; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x80[i]; 
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


