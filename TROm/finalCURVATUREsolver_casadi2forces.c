/*
 * CasADi to FORCESPRO Template - missing information to be filled in by createCasadi.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "finalCURVATUREsolver/include/finalCURVATUREsolver.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif


#include "finalCURVATUREsolver_casadi.h"



/* copies data from sparse matrix into a dense one */
static void sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, finalCURVATUREsolver_callback_float *data, finalCURVATUREsolver_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((finalCURVATUREsolver_float) data[j]);
        }
    }
}




/* CasADi to FORCESPRO interface */
extern void finalCURVATUREsolver_casadi2forces(finalCURVATUREsolver_float *x,        /* primal vars                                         */
                                 finalCURVATUREsolver_float *y,        /* eq. constraint multiplers                           */
                                 finalCURVATUREsolver_float *l,        /* ineq. constraint multipliers                        */
                                 finalCURVATUREsolver_float *p,        /* parameters                                          */
                                 finalCURVATUREsolver_float *f,        /* objective function (scalar)                         */
                                 finalCURVATUREsolver_float *nabla_f,  /* gradient of objective function                      */
                                 finalCURVATUREsolver_float *c,        /* dynamics                                            */
                                 finalCURVATUREsolver_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 finalCURVATUREsolver_float *h,        /* inequality constraints                              */
                                 finalCURVATUREsolver_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 finalCURVATUREsolver_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* CasADi input and output arrays */
    const finalCURVATUREsolver_callback_float *in[4];
    finalCURVATUREsolver_callback_float *out[7];
	

	/* Allocate working arrays for CasADi */
	finalCURVATUREsolver_callback_float w[181];
	
    /* temporary storage for CasADi sparse output */
    finalCURVATUREsolver_callback_float this_f;
    finalCURVATUREsolver_callback_float nabla_f_sparse[4];
    finalCURVATUREsolver_callback_float h_sparse[2];
    finalCURVATUREsolver_callback_float nabla_h_sparse[4];
    finalCURVATUREsolver_callback_float c_sparse[7];
    finalCURVATUREsolver_callback_float nabla_c_sparse[49];
            
    
    /* pointers to row and column info for 
     * column compressed format used by CasADi */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for CasADi */
	in[0] = x;
	in[1] = p;
	in[2] = l;
	in[3] = y;

	if ((stage >= 0) && (stage < 79))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		finalCURVATUREsolver_objective_1(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = finalCURVATUREsolver_objective_1_sparsity_out(1)[0];
			ncol = finalCURVATUREsolver_objective_1_sparsity_out(1)[1];
			colind = finalCURVATUREsolver_objective_1_sparsity_out(1) + 2;
			row = finalCURVATUREsolver_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		finalCURVATUREsolver_dynamics_1(in, out, NULL, w, 0);
		if (c != NULL)
		{
			nrow = finalCURVATUREsolver_dynamics_1_sparsity_out(0)[0];
			ncol = finalCURVATUREsolver_dynamics_1_sparsity_out(0)[1];
			colind = finalCURVATUREsolver_dynamics_1_sparsity_out(0) + 2;
			row = finalCURVATUREsolver_dynamics_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}

		if (nabla_c != NULL)
		{
			nrow = finalCURVATUREsolver_dynamics_1_sparsity_out(1)[0];
			ncol = finalCURVATUREsolver_dynamics_1_sparsity_out(1)[1];
			colind = finalCURVATUREsolver_dynamics_1_sparsity_out(1) + 2;
			row = finalCURVATUREsolver_dynamics_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		finalCURVATUREsolver_inequalities_1(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = finalCURVATUREsolver_inequalities_1_sparsity_out(0)[0];
			ncol = finalCURVATUREsolver_inequalities_1_sparsity_out(0)[1];
			colind = finalCURVATUREsolver_inequalities_1_sparsity_out(0) + 2;
			row = finalCURVATUREsolver_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = finalCURVATUREsolver_inequalities_1_sparsity_out(1)[0];
			ncol = finalCURVATUREsolver_inequalities_1_sparsity_out(1)[1];
			colind = finalCURVATUREsolver_inequalities_1_sparsity_out(1) + 2;
			row = finalCURVATUREsolver_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}

	if ((stage >= 79) && (stage < 80))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		finalCURVATUREsolver_objective_80(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = finalCURVATUREsolver_objective_80_sparsity_out(1)[0];
			ncol = finalCURVATUREsolver_objective_80_sparsity_out(1)[1];
			colind = finalCURVATUREsolver_objective_80_sparsity_out(1) + 2;
			row = finalCURVATUREsolver_objective_80_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		finalCURVATUREsolver_inequalities_80(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = finalCURVATUREsolver_inequalities_80_sparsity_out(0)[0];
			ncol = finalCURVATUREsolver_inequalities_80_sparsity_out(0)[1];
			colind = finalCURVATUREsolver_inequalities_80_sparsity_out(0) + 2;
			row = finalCURVATUREsolver_inequalities_80_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = finalCURVATUREsolver_inequalities_80_sparsity_out(1)[0];
			ncol = finalCURVATUREsolver_inequalities_80_sparsity_out(1)[1];
			colind = finalCURVATUREsolver_inequalities_80_sparsity_out(1) + 2;
			row = finalCURVATUREsolver_inequalities_80_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}


    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((finalCURVATUREsolver_float) this_f);
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif
