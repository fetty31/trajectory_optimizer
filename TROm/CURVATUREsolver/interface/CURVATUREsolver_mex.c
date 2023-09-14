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

#include "mex.h"
#include "math.h"
#include "../include/CURVATUREsolver.h"
#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif



/* copy functions */

void copyCArrayToM_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double) (*src++) ;
    }
}

void copyMValueToC_double(double * src, double * dest)
{
	*dest = (double) *src;
}



extern void CURVATUREsolver_casadi2forces(CURVATUREsolver_float *x, CURVATUREsolver_float *y, CURVATUREsolver_float *l, CURVATUREsolver_float *p, CURVATUREsolver_float *f, CURVATUREsolver_float *nabla_f, CURVATUREsolver_float *c, CURVATUREsolver_float *nabla_c, CURVATUREsolver_float *h, CURVATUREsolver_float *nabla_h, CURVATUREsolver_float *hess, solver_int32_default stage, solver_int32_default iteration, solver_int32_default threadID);
CURVATUREsolver_extfunc pt2function_CURVATUREsolver = &CURVATUREsolver_casadi2forces;


/* Some memory for mex-function */
static CURVATUREsolver_params params;
static CURVATUREsolver_output output;
static CURVATUREsolver_info info;

/* THE mex-function */
void mexFunction( solver_int32_default nlhs, mxArray *plhs[], solver_int32_default nrhs, const mxArray *prhs[] )  
{
	/* file pointer for printing */
	FILE *fp = NULL;

	/* define variables */	
	mxArray *par;
	mxArray *outvar;
	const mxArray *PARAMS = prhs[0]; 
	double *pvalue;
	solver_int32_default i;
	solver_int32_default exitflag;
	const solver_int8_default *fname;
	const solver_int8_default *outputnames[328] = {"x001","x002","x003","x004","x005","x006","x007","x008","x009","x010","x011","x012","x013","x014","x015","x016","x017","x018","x019","x020","x021","x022","x023","x024","x025","x026","x027","x028","x029","x030","x031","x032","x033","x034","x035","x036","x037","x038","x039","x040","x041","x042","x043","x044","x045","x046","x047","x048","x049","x050","x051","x052","x053","x054","x055","x056","x057","x058","x059","x060","x061","x062","x063","x064","x065","x066","x067","x068","x069","x070","x071","x072","x073","x074","x075","x076","x077","x078","x079","x080","x081","x082","x083","x084","x085","x086","x087","x088","x089","x090","x091","x092","x093","x094","x095","x096","x097","x098","x099","x100","x101","x102","x103","x104","x105","x106","x107","x108","x109","x110","x111","x112","x113","x114","x115","x116","x117","x118","x119","x120","x121","x122","x123","x124","x125","x126","x127","x128","x129","x130","x131","x132","x133","x134","x135","x136","x137","x138","x139","x140","x141","x142","x143","x144","x145","x146","x147","x148","x149","x150","x151","x152","x153","x154","x155","x156","x157","x158","x159","x160","x161","x162","x163","x164","x165","x166","x167","x168","x169","x170","x171","x172","x173","x174","x175","x176","x177","x178","x179","x180","x181","x182","x183","x184","x185","x186","x187","x188","x189","x190","x191","x192","x193","x194","x195","x196","x197","x198","x199","x200","x201","x202","x203","x204","x205","x206","x207","x208","x209","x210","x211","x212","x213","x214","x215","x216","x217","x218","x219","x220","x221","x222","x223","x224","x225","x226","x227","x228","x229","x230","x231","x232","x233","x234","x235","x236","x237","x238","x239","x240","x241","x242","x243","x244","x245","x246","x247","x248","x249","x250","x251","x252","x253","x254","x255","x256","x257","x258","x259","x260","x261","x262","x263","x264","x265","x266","x267","x268","x269","x270","x271","x272","x273","x274","x275","x276","x277","x278","x279","x280","x281","x282","x283","x284","x285","x286","x287","x288","x289","x290","x291","x292","x293","x294","x295","x296","x297","x298","x299","x300","x301","x302","x303","x304","x305","x306","x307","x308","x309","x310","x311","x312","x313","x314","x315","x316","x317","x318","x319","x320","x321","x322","x323","x324","x325","x326","x327","x328"};
	const solver_int8_default *infofields[10] = { "it", "it2opt", "res_eq", "res_ineq",  "rsnorm",  "rcompnorm",  "pobj",  "mu",  "solvetime",  "fevalstime"};
	
	/* Check for proper number of arguments */
    if (nrhs != 1)
	{
		mexErrMsgTxt("This function requires exactly 1 input: PARAMS struct.\nType 'help CURVATUREsolver_mex' for details.");
	}    
	if (nlhs > 3) 
	{
        mexErrMsgTxt("This function returns at most 3 outputs.\nType 'help CURVATUREsolver_mex' for details.");
    }

	/* Check whether params is actually a structure */
	if( !mxIsStruct(PARAMS) ) 
	{
		mexErrMsgTxt("PARAMS must be a structure.");
	}
	 

	/* copy parameters into the right location */
	par = mxGetField(PARAMS, 0, "lb");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.lb not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb must be a double.");
    }
    if( mxGetM(par) != 2952 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.lb must be of size [2952 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.lb,2952);

	}
	par = mxGetField(PARAMS, 0, "ub");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.ub not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub must be a double.");
    }
    if( mxGetM(par) != 2952 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.ub must be of size [2952 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.ub,2952);

	}
	par = mxGetField(PARAMS, 0, "hu");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.hu not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.hu must be a double.");
    }
    if( mxGetM(par) != 656 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.hu must be of size [656 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.hu,656);

	}
	par = mxGetField(PARAMS, 0, "hl");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.hl not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.hl must be a double.");
    }
    if( mxGetM(par) != 656 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.hl must be of size [656 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.hl,656);

	}
	par = mxGetField(PARAMS, 0, "x0");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.x0 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.x0 must be a double.");
    }
    if( mxGetM(par) != 2952 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.x0 must be of size [2952 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.x0,2952);

	}
	par = mxGetField(PARAMS, 0, "all_parameters");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.all_parameters not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.all_parameters must be a double.");
    }
    if( mxGetM(par) != 7872 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.all_parameters must be of size [7872 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.all_parameters,7872);

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

	/* call solver */
	exitflag = CURVATUREsolver_solve(&params, &output, &info, fp, pt2function_CURVATUREsolver);
	
	#if SET_PRINTLEVEL_CURVATUREsolver > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			mexPrintf("%c",i);
		}
		fclose(fp);
	#endif

	/* copy output to matlab arrays */
	plhs[0] = mxCreateStructMatrix(1, 1, 328, outputnames);
		outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x001, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x001", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x002, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x002", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x003, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x003", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x004, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x004", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x005, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x005", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x006, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x006", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x007, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x007", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x008, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x008", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x009, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x009", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x010, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x010", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x011, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x011", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x012, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x012", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x013, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x013", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x014, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x014", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x015, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x015", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x016, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x016", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x017, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x017", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x018, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x018", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x019, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x019", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x020, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x020", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x021, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x021", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x022, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x022", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x023, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x023", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x024, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x024", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x025, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x025", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x026, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x026", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x027, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x027", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x028, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x028", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x029, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x029", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x030, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x030", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x031, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x031", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x032, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x032", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x033, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x033", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x034, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x034", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x035, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x035", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x036, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x036", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x037, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x037", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x038, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x038", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x039, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x039", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x040, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x040", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x041, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x041", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x042, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x042", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x043, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x043", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x044, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x044", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x045, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x045", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x046, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x046", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x047, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x047", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x048, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x048", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x049, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x049", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x050, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x050", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x051, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x051", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x052, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x052", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x053, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x053", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x054, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x054", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x055, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x055", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x056, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x056", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x057, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x057", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x058, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x058", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x059, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x059", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x060, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x060", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x061, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x061", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x062, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x062", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x063, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x063", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x064, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x064", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x065, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x065", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x066, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x066", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x067, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x067", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x068, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x068", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x069, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x069", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x070, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x070", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x071, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x071", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x072, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x072", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x073, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x073", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x074, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x074", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x075, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x075", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x076, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x076", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x077, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x077", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x078, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x078", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x079, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x079", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x080, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x080", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x081, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x081", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x082, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x082", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x083, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x083", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x084, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x084", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x085, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x085", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x086, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x086", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x087, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x087", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x088, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x088", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x089, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x089", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x090, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x090", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x091, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x091", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x092, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x092", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x093, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x093", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x094, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x094", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x095, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x095", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x096, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x096", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x097, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x097", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x098, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x098", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x099, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x099", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x100, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x100", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x101, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x101", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x102, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x102", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x103, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x103", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x104, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x104", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x105, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x105", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x106, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x106", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x107, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x107", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x108, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x108", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x109, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x109", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x110, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x110", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x111, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x111", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x112, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x112", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x113, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x113", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x114, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x114", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x115, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x115", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x116, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x116", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x117, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x117", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x118, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x118", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x119, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x119", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x120, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x120", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x121, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x121", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x122, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x122", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x123, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x123", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x124, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x124", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x125, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x125", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x126, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x126", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x127, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x127", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x128, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x128", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x129, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x129", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x130, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x130", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x131, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x131", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x132, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x132", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x133, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x133", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x134, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x134", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x135, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x135", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x136, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x136", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x137, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x137", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x138, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x138", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x139, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x139", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x140, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x140", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x141, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x141", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x142, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x142", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x143, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x143", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x144, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x144", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x145, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x145", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x146, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x146", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x147, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x147", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x148, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x148", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x149, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x149", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x150, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x150", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x151, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x151", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x152, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x152", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x153, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x153", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x154, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x154", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x155, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x155", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x156, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x156", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x157, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x157", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x158, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x158", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x159, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x159", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x160, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x160", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x161, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x161", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x162, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x162", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x163, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x163", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x164, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x164", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x165, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x165", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x166, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x166", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x167, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x167", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x168, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x168", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x169, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x169", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x170, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x170", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x171, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x171", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x172, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x172", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x173, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x173", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x174, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x174", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x175, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x175", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x176, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x176", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x177, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x177", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x178, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x178", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x179, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x179", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x180, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x180", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x181, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x181", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x182, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x182", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x183, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x183", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x184, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x184", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x185, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x185", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x186, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x186", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x187, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x187", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x188, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x188", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x189, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x189", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x190, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x190", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x191, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x191", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x192, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x192", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x193, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x193", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x194, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x194", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x195, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x195", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x196, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x196", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x197, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x197", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x198, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x198", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x199, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x199", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x200, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x200", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x201, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x201", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x202, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x202", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x203, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x203", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x204, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x204", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x205, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x205", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x206, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x206", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x207, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x207", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x208, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x208", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x209, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x209", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x210, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x210", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x211, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x211", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x212, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x212", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x213, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x213", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x214, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x214", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x215, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x215", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x216, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x216", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x217, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x217", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x218, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x218", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x219, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x219", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x220, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x220", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x221, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x221", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x222, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x222", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x223, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x223", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x224, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x224", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x225, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x225", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x226, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x226", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x227, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x227", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x228, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x228", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x229, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x229", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x230, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x230", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x231, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x231", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x232, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x232", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x233, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x233", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x234, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x234", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x235, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x235", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x236, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x236", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x237, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x237", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x238, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x238", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x239, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x239", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x240, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x240", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x241, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x241", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x242, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x242", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x243, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x243", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x244, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x244", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x245, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x245", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x246, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x246", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x247, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x247", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x248, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x248", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x249, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x249", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x250, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x250", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x251, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x251", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x252, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x252", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x253, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x253", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x254, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x254", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x255, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x255", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x256, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x256", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x257, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x257", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x258, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x258", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x259, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x259", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x260, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x260", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x261, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x261", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x262, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x262", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x263, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x263", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x264, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x264", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x265, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x265", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x266, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x266", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x267, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x267", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x268, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x268", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x269, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x269", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x270, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x270", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x271, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x271", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x272, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x272", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x273, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x273", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x274, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x274", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x275, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x275", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x276, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x276", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x277, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x277", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x278, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x278", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x279, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x279", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x280, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x280", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x281, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x281", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x282, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x282", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x283, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x283", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x284, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x284", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x285, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x285", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x286, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x286", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x287, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x287", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x288, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x288", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x289, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x289", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x290, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x290", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x291, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x291", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x292, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x292", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x293, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x293", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x294, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x294", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x295, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x295", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x296, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x296", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x297, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x297", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x298, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x298", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x299, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x299", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x300, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x300", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x301, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x301", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x302, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x302", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x303, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x303", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x304, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x304", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x305, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x305", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x306, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x306", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x307, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x307", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x308, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x308", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x309, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x309", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x310, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x310", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x311, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x311", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x312, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x312", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x313, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x313", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x314, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x314", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x315, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x315", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x316, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x316", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x317, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x317", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x318, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x318", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x319, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x319", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x320, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x320", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x321, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x321", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x322, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x322", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x323, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x323", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x324, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x324", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x325, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x325", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x326, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x326", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x327, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x327", outvar);

	outvar = mxCreateDoubleMatrix(9, 1, mxREAL);
	copyCArrayToM_double( output.x328, mxGetPr(outvar), 9);
	mxSetField(plhs[0], 0, "x328", outvar);



	/* copy exitflag */
	if( nlhs > 1 )
	{
	plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
	*mxGetPr(plhs[1]) = (double)exitflag;
	}

	/* copy info struct */
	if( nlhs > 2 )
	{
	        plhs[2] = mxCreateStructMatrix(1, 1, 10, infofields);
         
		
		/* iterations */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it;
		mxSetField(plhs[2], 0, "it", outvar);

		/* iterations to optimality (branch and bound) */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it2opt;
		mxSetField(plhs[2], 0, "it2opt", outvar);
		
		/* res_eq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_eq;
		mxSetField(plhs[2], 0, "res_eq", outvar);

		/* res_ineq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_ineq;
		mxSetField(plhs[2], 0, "res_ineq", outvar);

		/* rsnorm */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.rsnorm;
		mxSetField(plhs[2], 0, "rsnorm", outvar);

		/* rcompnorm */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.rcompnorm;
		mxSetField(plhs[2], 0, "rcompnorm", outvar);
		
		/* pobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.pobj;
		mxSetField(plhs[2], 0, "pobj", outvar);

		/* mu */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu;
		mxSetField(plhs[2], 0, "mu", outvar);

		/* solver time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.solvetime;
		mxSetField(plhs[2], 0, "solvetime", outvar);

		/* solver time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.fevalstime;
		mxSetField(plhs[2], 0, "fevalstime", outvar);

	}
}
