#ifndef AIRCRAFTPCE_FGM_H
#define AIRCRAFTPCE_FGM_H

#include "mc04types.h"
#include "arithmetic.h"

/* Quadratic Programming online solver using Fast gradients.
 * Most of these functions are intended for internal use.
 */
struct aircraftpce_fgm_conf {
uint32_t in_iter;  /**< Maximum number of internal loop (FGM) iterations (j_in). */
uint32_t warmstart;  /**< If not 0, automatically warmstart next algorithm iteration. */
}; /**< Configuration parameters of the AIRCRAFTPCE algorithm. */

struct aircraftpce_fgm {
struct aircraftpce_fgm_conf *conf;  /**< Algorithm configuration data. */
real_t *u_opt;  /**< Solution to the optimal control problem. */
real_t *u_ini;  /**< Initial guess for the optimal control sequence. */
real_t *goL;  /**< Gradient vector over Lipschitz for the current system state. */
uint32_t *j_in;  /**< Maximun number of internal loop (FGM) iterations .*/
real_t *HoL;  /**< Hessian matrix of QP over Lipschitz constant. */
real_t *u_lb;  /**< Lower bound constraint of the inputs for condensed QP. */
real_t *u_ub;  /**< Upper bound constraint of the inputs for condensed QP. */
real_t *nu;  /**< Fast gradient extra step constant. */
real_t *tmp1_optvar_seqlen;  /**< First temporary variable of length optvar_seqlen. */
real_t *tmp2_optvar_seqlen;  /**< Second temporary variable of length optvar_seqlen. */
real_t *tmp3_optvar_seqlen;  /**< Third temporary variable of length optvar_seqlen. */
uint32_t optvar_veclen;  /**< The length of each vector in the optimation variable sequence. */
uint32_t optvar_seqlen;  /**< The full length of optimization variable sequence. */
uint32_t sizeof_optvar_seqlen;  /**< Number of bytes in the optimization variable sequence. */
};  /**< Variables used by the fast gradient method. */

/* External function declarations */

extern void aircraftpce_fgm_solve_problem(const struct aircraftpce_fgm *fgm);

extern void aircraftpce_fgm_minimize_qp_iteration(const struct aircraftpce_fgm *fgm,
		real_t u[], real_t u_old[], real_t w[], const real_t gradoL[]);

extern void aircraftpce_fgm_compute_grad_over_L(const struct aircraftpce_fgm *fgm,
		real_t gradoL[], const real_t w[]);

extern void aircraftpce_compute_gxoL(struct aircraftpce_fgm *fgm, const real_t x[]);

extern void aircraftpce_warmstart_vector(struct aircraftpce_fgm *fgm, real_t outseq[],
    const real_t seq[], const uint32_t veclen, const uint32_t seqlen);

struct aircraftpce_fgm_solver {
struct aircraftpce_fgm_conf *conf;  /**< Algorithm configuration data. */
struct aircraftpce_fgm *data;  /**< Algorithm data. */
void (*solve_problem)(const struct aircraftpce_fgm *data);
};

#endif /* AIRCRAFTPCE_FGM_H */
