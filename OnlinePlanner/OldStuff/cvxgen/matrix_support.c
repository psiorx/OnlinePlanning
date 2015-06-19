/* Produced by CVXGEN, 2013-07-30 13:25:14 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.A[0])-rhs[1]*(params.A[3])-rhs[2]*(params.A[6]);
  lhs[1] = -rhs[0]*(params.A[1])-rhs[1]*(params.A[4])-rhs[2]*(params.A[7]);
  lhs[2] = -rhs[0]*(params.A[2])-rhs[1]*(params.A[5])-rhs[2]*(params.A[8]);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.A[0])-rhs[1]*(params.A[1])-rhs[2]*(params.A[2]);
  lhs[1] = -rhs[0]*(params.A[3])-rhs[1]*(params.A[4])-rhs[2]*(params.A[5]);
  lhs[2] = -rhs[0]*(params.A[6])-rhs[1]*(params.A[7])-rhs[2]*(params.A[8]);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.S[0])+rhs[1]*(2*params.S[3])+rhs[2]*(2*params.S[6]);
  lhs[1] = rhs[0]*(2*params.S[1])+rhs[1]*(2*params.S[4])+rhs[2]*(2*params.S[7]);
  lhs[2] = rhs[0]*(2*params.S[2])+rhs[1]*(2*params.S[5])+rhs[2]*(2*params.S[8]);
}
void fillq(void) {
  work.q[0] = params.s1[0];
  work.q[1] = params.s1[1];
  work.q[2] = params.s1[2];
}
void fillh(void) {
  work.h[0] = params.b[0];
  work.h[1] = params.b[1];
  work.h[2] = params.b[2];
}
void fillb(void) {
}
void pre_ops(void) {
}
