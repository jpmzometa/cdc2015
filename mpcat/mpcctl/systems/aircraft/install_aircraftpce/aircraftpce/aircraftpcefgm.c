/** Quadratic program solver based on the fast gradient method.
 */

#include <string.h> /* sizeof */

#include "aircraftpcemtxops.h"
#include "include/aircraftpcefgm.h"

/* static functions declaration */
static void aircraftpce_fgm_compute_projected_grad_step(const struct aircraftpce_fgm *fgm,
		real_t u[], const real_t w[],
		const real_t gradoL[]);


/* external functions definition */

/* Minimize MPC quadratic program using fast gradient method */
void aircraftpce_fgm_solve_problem(const struct aircraftpce_fgm *fgm) {
	real_t *u_old = fgm->tmp1_optvar_seqlen;
	real_t *w = fgm->tmp2_optvar_seqlen;
	real_t *gradoL = fgm->tmp3_optvar_seqlen;
	uint32_t j;

	memcpy(fgm->u_opt, fgm->u_ini, fgm->sizeof_optvar_seqlen);
	memcpy(w, fgm->u_ini, fgm->sizeof_optvar_seqlen);
	memcpy(u_old, fgm->u_ini, fgm->sizeof_optvar_seqlen);

	for (j = 0; j < *(fgm->j_in); j++) {
		aircraftpce_fgm_compute_grad_over_L(fgm, gradoL, w);
		aircraftpce_fgm_minimize_qp_iteration(fgm, fgm->u_opt, u_old, w, gradoL);
	}

	return;
}

/* Execute one iteration of the minimization algorithm */
void aircraftpce_fgm_minimize_qp_iteration(const struct aircraftpce_fgm *fgm, real_t u[],
		real_t u_old[], real_t w[],	const real_t gradoL[]) {
	real_t *du = fgm->tmp1_optvar_seqlen;
	real_t *nu_du = fgm->tmp2_optvar_seqlen;

	aircraftpce_fgm_compute_projected_grad_step(fgm, u, w, gradoL);
  aircraftpce_mtx_substract(du, u, u_old, fgm->optvar_seqlen, 1);
	aircraftpce_mtx_scale(nu_du, du, *(fgm->nu), fgm->optvar_seqlen, 1);
	aircraftpce_mtx_add(w, u, nu_du, fgm->optvar_seqlen, 1);
	memcpy(u_old, u, fgm->sizeof_optvar_seqlen);

	return;
}

/* Compute gradient divided by Lipschitz constant */
void aircraftpce_fgm_compute_grad_over_L(const struct aircraftpce_fgm *fgm, real_t gradoL[],
								const real_t w[]) {
	real_t *HoL_w = fgm->tmp1_optvar_seqlen;
	/* gradoL = (H/L) * w + (g/L) */
	aircraftpce_mtx_multiply_mtx_vec(HoL_w, fgm->HoL, w, fgm->optvar_seqlen, fgm->optvar_seqlen);
	aircraftpce_mtx_add(gradoL, HoL_w, fgm->goL, fgm->optvar_seqlen, 1);

	return;
}

/* Copy the contents of the input sequence, except for the first vector, to the
 * output sequence. The last vector of the output sequence is set to zero
 */
void aircraftpce_warmstart_sequence(struct aircraftpce_fgm *fgm, real_t outseq[], const real_t seq[],
		const uint32_t veclen, const uint32_t seqlen)
{
	uint32_t i;
	/* shift vector one horizon step backwards */
	for (i=0; i < (seqlen - veclen); i++) {
		outseq[i] = seq[i + veclen];
	}

	/* set last element to zero */
	for (i=(seqlen - veclen); i < seqlen; i++) {
		outseq[i] = 0.;
	}
	return;
}

/* static functions definition */

static void aircraftpce_fgm_compute_projected_grad_step(const struct aircraftpce_fgm *fgm,
		real_t u[], const real_t w[],
		const real_t gradoL[]) {
	aircraftpce_mtx_substract(u, w, gradoL, fgm->optvar_seqlen, 1);
	aircraftpce_mtx_saturate_vec(u, fgm->u_lb, fgm->u_ub, fgm->optvar_seqlen);

	return;
}
