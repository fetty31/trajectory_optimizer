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
#define S_FUNCTION_NAME CURVATUREsolver_simulinkBlockcompact

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
    ssSetInputPortMatrixDimensions(S,  0, 656, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 656, 1);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/
	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 2952, 1);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/
	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 2952, 1);
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
 


	/* initialize output ports - there are 1 in total */
    if (!ssSetNumOutputPorts(S, 1)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 2952, 1);
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
	const real_T *all_parameters = (const real_T*) ssGetInputPortSignal(S,5);
	
    real_T *outputs = (real_T*) ssGetOutputPortSignal(S,0);
	
	

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
		outputs[i] = (real_T) output.x001[i]; 
	}

	k=9; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x002[i]; 
	}

	k=18; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x003[i]; 
	}

	k=27; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x004[i]; 
	}

	k=36; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x005[i]; 
	}

	k=45; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x006[i]; 
	}

	k=54; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x007[i]; 
	}

	k=63; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x008[i]; 
	}

	k=72; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x009[i]; 
	}

	k=81; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x010[i]; 
	}

	k=90; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x011[i]; 
	}

	k=99; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x012[i]; 
	}

	k=108; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x013[i]; 
	}

	k=117; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x014[i]; 
	}

	k=126; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x015[i]; 
	}

	k=135; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x016[i]; 
	}

	k=144; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x017[i]; 
	}

	k=153; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x018[i]; 
	}

	k=162; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x019[i]; 
	}

	k=171; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x020[i]; 
	}

	k=180; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x021[i]; 
	}

	k=189; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x022[i]; 
	}

	k=198; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x023[i]; 
	}

	k=207; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x024[i]; 
	}

	k=216; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x025[i]; 
	}

	k=225; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x026[i]; 
	}

	k=234; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x027[i]; 
	}

	k=243; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x028[i]; 
	}

	k=252; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x029[i]; 
	}

	k=261; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x030[i]; 
	}

	k=270; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x031[i]; 
	}

	k=279; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x032[i]; 
	}

	k=288; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x033[i]; 
	}

	k=297; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x034[i]; 
	}

	k=306; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x035[i]; 
	}

	k=315; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x036[i]; 
	}

	k=324; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x037[i]; 
	}

	k=333; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x038[i]; 
	}

	k=342; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x039[i]; 
	}

	k=351; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x040[i]; 
	}

	k=360; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x041[i]; 
	}

	k=369; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x042[i]; 
	}

	k=378; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x043[i]; 
	}

	k=387; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x044[i]; 
	}

	k=396; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x045[i]; 
	}

	k=405; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x046[i]; 
	}

	k=414; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x047[i]; 
	}

	k=423; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x048[i]; 
	}

	k=432; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x049[i]; 
	}

	k=441; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x050[i]; 
	}

	k=450; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x051[i]; 
	}

	k=459; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x052[i]; 
	}

	k=468; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x053[i]; 
	}

	k=477; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x054[i]; 
	}

	k=486; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x055[i]; 
	}

	k=495; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x056[i]; 
	}

	k=504; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x057[i]; 
	}

	k=513; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x058[i]; 
	}

	k=522; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x059[i]; 
	}

	k=531; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x060[i]; 
	}

	k=540; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x061[i]; 
	}

	k=549; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x062[i]; 
	}

	k=558; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x063[i]; 
	}

	k=567; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x064[i]; 
	}

	k=576; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x065[i]; 
	}

	k=585; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x066[i]; 
	}

	k=594; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x067[i]; 
	}

	k=603; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x068[i]; 
	}

	k=612; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x069[i]; 
	}

	k=621; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x070[i]; 
	}

	k=630; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x071[i]; 
	}

	k=639; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x072[i]; 
	}

	k=648; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x073[i]; 
	}

	k=657; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x074[i]; 
	}

	k=666; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x075[i]; 
	}

	k=675; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x076[i]; 
	}

	k=684; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x077[i]; 
	}

	k=693; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x078[i]; 
	}

	k=702; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x079[i]; 
	}

	k=711; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x080[i]; 
	}

	k=720; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x081[i]; 
	}

	k=729; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x082[i]; 
	}

	k=738; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x083[i]; 
	}

	k=747; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x084[i]; 
	}

	k=756; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x085[i]; 
	}

	k=765; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x086[i]; 
	}

	k=774; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x087[i]; 
	}

	k=783; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x088[i]; 
	}

	k=792; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x089[i]; 
	}

	k=801; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x090[i]; 
	}

	k=810; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x091[i]; 
	}

	k=819; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x092[i]; 
	}

	k=828; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x093[i]; 
	}

	k=837; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x094[i]; 
	}

	k=846; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x095[i]; 
	}

	k=855; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x096[i]; 
	}

	k=864; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x097[i]; 
	}

	k=873; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x098[i]; 
	}

	k=882; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x099[i]; 
	}

	k=891; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x100[i]; 
	}

	k=900; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x101[i]; 
	}

	k=909; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x102[i]; 
	}

	k=918; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x103[i]; 
	}

	k=927; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x104[i]; 
	}

	k=936; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x105[i]; 
	}

	k=945; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x106[i]; 
	}

	k=954; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x107[i]; 
	}

	k=963; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x108[i]; 
	}

	k=972; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x109[i]; 
	}

	k=981; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x110[i]; 
	}

	k=990; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x111[i]; 
	}

	k=999; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x112[i]; 
	}

	k=1008; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x113[i]; 
	}

	k=1017; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x114[i]; 
	}

	k=1026; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x115[i]; 
	}

	k=1035; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x116[i]; 
	}

	k=1044; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x117[i]; 
	}

	k=1053; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x118[i]; 
	}

	k=1062; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x119[i]; 
	}

	k=1071; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x120[i]; 
	}

	k=1080; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x121[i]; 
	}

	k=1089; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x122[i]; 
	}

	k=1098; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x123[i]; 
	}

	k=1107; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x124[i]; 
	}

	k=1116; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x125[i]; 
	}

	k=1125; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x126[i]; 
	}

	k=1134; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x127[i]; 
	}

	k=1143; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x128[i]; 
	}

	k=1152; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x129[i]; 
	}

	k=1161; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x130[i]; 
	}

	k=1170; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x131[i]; 
	}

	k=1179; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x132[i]; 
	}

	k=1188; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x133[i]; 
	}

	k=1197; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x134[i]; 
	}

	k=1206; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x135[i]; 
	}

	k=1215; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x136[i]; 
	}

	k=1224; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x137[i]; 
	}

	k=1233; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x138[i]; 
	}

	k=1242; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x139[i]; 
	}

	k=1251; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x140[i]; 
	}

	k=1260; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x141[i]; 
	}

	k=1269; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x142[i]; 
	}

	k=1278; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x143[i]; 
	}

	k=1287; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x144[i]; 
	}

	k=1296; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x145[i]; 
	}

	k=1305; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x146[i]; 
	}

	k=1314; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x147[i]; 
	}

	k=1323; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x148[i]; 
	}

	k=1332; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x149[i]; 
	}

	k=1341; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x150[i]; 
	}

	k=1350; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x151[i]; 
	}

	k=1359; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x152[i]; 
	}

	k=1368; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x153[i]; 
	}

	k=1377; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x154[i]; 
	}

	k=1386; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x155[i]; 
	}

	k=1395; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x156[i]; 
	}

	k=1404; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x157[i]; 
	}

	k=1413; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x158[i]; 
	}

	k=1422; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x159[i]; 
	}

	k=1431; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x160[i]; 
	}

	k=1440; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x161[i]; 
	}

	k=1449; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x162[i]; 
	}

	k=1458; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x163[i]; 
	}

	k=1467; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x164[i]; 
	}

	k=1476; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x165[i]; 
	}

	k=1485; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x166[i]; 
	}

	k=1494; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x167[i]; 
	}

	k=1503; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x168[i]; 
	}

	k=1512; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x169[i]; 
	}

	k=1521; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x170[i]; 
	}

	k=1530; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x171[i]; 
	}

	k=1539; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x172[i]; 
	}

	k=1548; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x173[i]; 
	}

	k=1557; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x174[i]; 
	}

	k=1566; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x175[i]; 
	}

	k=1575; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x176[i]; 
	}

	k=1584; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x177[i]; 
	}

	k=1593; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x178[i]; 
	}

	k=1602; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x179[i]; 
	}

	k=1611; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x180[i]; 
	}

	k=1620; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x181[i]; 
	}

	k=1629; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x182[i]; 
	}

	k=1638; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x183[i]; 
	}

	k=1647; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x184[i]; 
	}

	k=1656; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x185[i]; 
	}

	k=1665; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x186[i]; 
	}

	k=1674; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x187[i]; 
	}

	k=1683; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x188[i]; 
	}

	k=1692; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x189[i]; 
	}

	k=1701; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x190[i]; 
	}

	k=1710; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x191[i]; 
	}

	k=1719; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x192[i]; 
	}

	k=1728; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x193[i]; 
	}

	k=1737; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x194[i]; 
	}

	k=1746; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x195[i]; 
	}

	k=1755; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x196[i]; 
	}

	k=1764; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x197[i]; 
	}

	k=1773; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x198[i]; 
	}

	k=1782; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x199[i]; 
	}

	k=1791; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x200[i]; 
	}

	k=1800; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x201[i]; 
	}

	k=1809; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x202[i]; 
	}

	k=1818; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x203[i]; 
	}

	k=1827; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x204[i]; 
	}

	k=1836; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x205[i]; 
	}

	k=1845; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x206[i]; 
	}

	k=1854; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x207[i]; 
	}

	k=1863; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x208[i]; 
	}

	k=1872; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x209[i]; 
	}

	k=1881; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x210[i]; 
	}

	k=1890; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x211[i]; 
	}

	k=1899; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x212[i]; 
	}

	k=1908; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x213[i]; 
	}

	k=1917; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x214[i]; 
	}

	k=1926; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x215[i]; 
	}

	k=1935; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x216[i]; 
	}

	k=1944; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x217[i]; 
	}

	k=1953; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x218[i]; 
	}

	k=1962; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x219[i]; 
	}

	k=1971; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x220[i]; 
	}

	k=1980; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x221[i]; 
	}

	k=1989; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x222[i]; 
	}

	k=1998; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x223[i]; 
	}

	k=2007; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x224[i]; 
	}

	k=2016; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x225[i]; 
	}

	k=2025; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x226[i]; 
	}

	k=2034; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x227[i]; 
	}

	k=2043; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x228[i]; 
	}

	k=2052; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x229[i]; 
	}

	k=2061; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x230[i]; 
	}

	k=2070; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x231[i]; 
	}

	k=2079; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x232[i]; 
	}

	k=2088; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x233[i]; 
	}

	k=2097; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x234[i]; 
	}

	k=2106; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x235[i]; 
	}

	k=2115; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x236[i]; 
	}

	k=2124; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x237[i]; 
	}

	k=2133; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x238[i]; 
	}

	k=2142; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x239[i]; 
	}

	k=2151; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x240[i]; 
	}

	k=2160; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x241[i]; 
	}

	k=2169; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x242[i]; 
	}

	k=2178; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x243[i]; 
	}

	k=2187; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x244[i]; 
	}

	k=2196; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x245[i]; 
	}

	k=2205; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x246[i]; 
	}

	k=2214; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x247[i]; 
	}

	k=2223; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x248[i]; 
	}

	k=2232; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x249[i]; 
	}

	k=2241; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x250[i]; 
	}

	k=2250; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x251[i]; 
	}

	k=2259; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x252[i]; 
	}

	k=2268; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x253[i]; 
	}

	k=2277; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x254[i]; 
	}

	k=2286; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x255[i]; 
	}

	k=2295; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x256[i]; 
	}

	k=2304; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x257[i]; 
	}

	k=2313; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x258[i]; 
	}

	k=2322; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x259[i]; 
	}

	k=2331; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x260[i]; 
	}

	k=2340; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x261[i]; 
	}

	k=2349; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x262[i]; 
	}

	k=2358; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x263[i]; 
	}

	k=2367; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x264[i]; 
	}

	k=2376; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x265[i]; 
	}

	k=2385; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x266[i]; 
	}

	k=2394; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x267[i]; 
	}

	k=2403; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x268[i]; 
	}

	k=2412; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x269[i]; 
	}

	k=2421; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x270[i]; 
	}

	k=2430; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x271[i]; 
	}

	k=2439; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x272[i]; 
	}

	k=2448; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x273[i]; 
	}

	k=2457; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x274[i]; 
	}

	k=2466; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x275[i]; 
	}

	k=2475; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x276[i]; 
	}

	k=2484; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x277[i]; 
	}

	k=2493; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x278[i]; 
	}

	k=2502; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x279[i]; 
	}

	k=2511; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x280[i]; 
	}

	k=2520; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x281[i]; 
	}

	k=2529; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x282[i]; 
	}

	k=2538; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x283[i]; 
	}

	k=2547; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x284[i]; 
	}

	k=2556; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x285[i]; 
	}

	k=2565; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x286[i]; 
	}

	k=2574; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x287[i]; 
	}

	k=2583; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x288[i]; 
	}

	k=2592; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x289[i]; 
	}

	k=2601; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x290[i]; 
	}

	k=2610; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x291[i]; 
	}

	k=2619; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x292[i]; 
	}

	k=2628; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x293[i]; 
	}

	k=2637; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x294[i]; 
	}

	k=2646; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x295[i]; 
	}

	k=2655; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x296[i]; 
	}

	k=2664; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x297[i]; 
	}

	k=2673; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x298[i]; 
	}

	k=2682; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x299[i]; 
	}

	k=2691; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x300[i]; 
	}

	k=2700; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x301[i]; 
	}

	k=2709; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x302[i]; 
	}

	k=2718; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x303[i]; 
	}

	k=2727; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x304[i]; 
	}

	k=2736; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x305[i]; 
	}

	k=2745; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x306[i]; 
	}

	k=2754; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x307[i]; 
	}

	k=2763; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x308[i]; 
	}

	k=2772; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x309[i]; 
	}

	k=2781; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x310[i]; 
	}

	k=2790; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x311[i]; 
	}

	k=2799; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x312[i]; 
	}

	k=2808; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x313[i]; 
	}

	k=2817; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x314[i]; 
	}

	k=2826; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x315[i]; 
	}

	k=2835; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x316[i]; 
	}

	k=2844; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x317[i]; 
	}

	k=2853; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x318[i]; 
	}

	k=2862; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x319[i]; 
	}

	k=2871; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x320[i]; 
	}

	k=2880; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x321[i]; 
	}

	k=2889; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x322[i]; 
	}

	k=2898; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x323[i]; 
	}

	k=2907; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x324[i]; 
	}

	k=2916; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x325[i]; 
	}

	k=2925; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x326[i]; 
	}

	k=2934; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x327[i]; 
	}

	k=2943; 
	for( i=0; i<9; i++)
	{ 
		outputs[k++] = (real_T) output.x328[i]; 
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


