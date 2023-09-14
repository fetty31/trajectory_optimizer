/*
 * CasADi to FORCESPRO Template - missing information to be filled in by createCasadi.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "CURVATUREsolver/include/CURVATUREsolver.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif


#include "CURVATUREsolver_casadi.h"



/* copies data from sparse matrix into a dense one */
static void sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, CURVATUREsolver_callback_float *data, CURVATUREsolver_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((CURVATUREsolver_float) data[j]);
        }
    }
}




/* CasADi to FORCESPRO interface */
extern void CURVATUREsolver_casadi2forces(CURVATUREsolver_float *x,        /* primal vars                                         */
                                 CURVATUREsolver_float *y,        /* eq. constraint multiplers                           */
                                 CURVATUREsolver_float *l,        /* ineq. constraint multipliers                        */
                                 CURVATUREsolver_float *p,        /* parameters                                          */
                                 CURVATUREsolver_float *f,        /* objective function (scalar)                         */
                                 CURVATUREsolver_float *nabla_f,  /* gradient of objective function                      */
                                 CURVATUREsolver_float *c,        /* dynamics                                            */
                                 CURVATUREsolver_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 CURVATUREsolver_float *h,        /* inequality constraints                              */
                                 CURVATUREsolver_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 CURVATUREsolver_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* CasADi input and output arrays */
    const CURVATUREsolver_callback_float *in[4];
    CURVATUREsolver_callback_float *out[7];
	

	/* Allocate working arrays for CasADi */
	CURVATUREsolver_callback_float w[181];
	
    /* temporary storage for CasADi sparse output */
    CURVATUREsolver_callback_float this_f;
    CURVATUREsolver_callback_float nabla_f_sparse[7];
    CURVATUREsolver_callback_float h_sparse[2];
    CURVATUREsolver_callback_float nabla_h_sparse[4];
    CURVATUREsolver_callback_float c_sparse[7];
    CURVATUREsolver_callback_float nabla_c_sparse[49];
            
    
    /* pointers to row and column info for 
     * column compressed format used by CasADi */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for CasADi */
	in[0] = x;
	in[1] = p;
	in[2] = l;
	in[3] = y;

	if ((stage >= 0) && (stage < 327))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		CURVATUREsolver_objective_1(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = CURVATUREsolver_objective_1_sparsity_out(1)[0];
			ncol = CURVATUREsolver_objective_1_sparsity_out(1)[1];
			colind = CURVATUREsolver_objective_1_sparsity_out(1) + 2;
			row = CURVATUREsolver_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		CURVATUREsolver_dynamics_1(in, out, NULL, w, 0);
		if (c != NULL)
		{
			nrow = CURVATUREsolver_dynamics_1_sparsity_out(0)[0];
			ncol = CURVATUREsolver_dynamics_1_sparsity_out(0)[1];
			colind = CURVATUREsolver_dynamics_1_sparsity_out(0) + 2;
			row = CURVATUREsolver_dynamics_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}

		if (nabla_c != NULL)
		{
			nrow = CURVATUREsolver_dynamics_1_sparsity_out(1)[0];
			ncol = CURVATUREsolver_dynamics_1_sparsity_out(1)[1];
			colind = CURVATUREsolver_dynamics_1_sparsity_out(1) + 2;
			row = CURVATUREsolver_dynamics_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		CURVATUREsolver_inequalities_1(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = CURVATUREsolver_inequalities_1_sparsity_out(0)[0];
			ncol = CURVATUREsolver_inequalities_1_sparsity_out(0)[1];
			colind = CURVATUREsolver_inequalities_1_sparsity_out(0) + 2;
			row = CURVATUREsolver_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = CURVATUREsolver_inequalities_1_sparsity_out(1)[0];
			ncol = CURVATUREsolver_inequalities_1_sparsity_out(1)[1];
			colind = CURVATUREsolver_inequalities_1_sparsity_out(1) + 2;
			row = CURVATUREsolver_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}

	if ((stage >= 327) && (stage < 328))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		CURVATUREsolver_objective_328(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = CURVATUREsolver_objective_328_sparsity_out(1)[0];
			ncol = CURVATUREsolver_objective_328_sparsity_out(1)[1];
			colind = CURVATUREsolver_objective_328_sparsity_out(1) + 2;
			row = CURVATUREsolver_objective_328_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		CURVATUREsolver_inequalities_328(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = CURVATUREsolver_inequalities_328_sparsity_out(0)[0];
			ncol = CURVATUREsolver_inequalities_328_sparsity_out(0)[1];
			colind = CURVATUREsolver_inequalities_328_sparsity_out(0) + 2;
			row = CURVATUREsolver_inequalities_328_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = CURVATUREsolver_inequalities_328_sparsity_out(1)[0];
			ncol = CURVATUREsolver_inequalities_328_sparsity_out(1)[1];
			colind = CURVATUREsolver_inequalities_328_sparsity_out(1) + 2;
			row = CURVATUREsolver_inequalities_328_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}


    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((CURVATUREsolver_float) this_f);
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif
