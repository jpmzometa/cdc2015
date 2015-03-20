#ifndef AIRCRAFTPCEFORMQP_H
#define AIRCRAFTPCEFORMQP_H

#include "mc04types.h"  /* typedefs */
#include "arithmetic.h"

/* Declarations that depend only on the structure of the problem
 * These are used to generate code independent of the data size
 */
    enum {
        AIRCRAFTPCE_X_K,

        AIRCRAFTPCE_PAR_NUM
        };

    enum {
        AIRCRAFTPCE_H,
        AIRCRAFTPCE_U_LB,
        AIRCRAFTPCE_V,
        AIRCRAFTPCE_U_UB,

        AIRCRAFTPCE_CONSTANT_NUM
        };

    enum {
        AIRCRAFTPCE_V_UB,
        AIRCRAFTPCE_G,
        AIRCRAFTPCE_V_LB,

        AIRCRAFTPCE_PMETRIC_NUM
        };

    struct aircraftpce_term {
        uint32_t rows;
        uint32_t cols;
        real_t *data;
    };

    struct aircraftpce_pmetric {
        uint32_t *fac_num;
        struct aircraftpce_term *val;
        struct aircraftpce_term *aux;
        struct aircraftpce_term *fac0;
        struct aircraftpce_term **fac;
        struct aircraftpce_term **par;
    };

    struct aircraftpce_cvp_prb {
    struct aircraftpce_term *x_k;
    struct aircraftpce_term *v_ub;
    struct aircraftpce_term *g;
    struct aircraftpce_term *v_lb;
    struct aircraftpce_term *H;
    struct aircraftpce_term *u_lb;
    struct aircraftpce_term *V;
    struct aircraftpce_term *u_ub;

#if 0  /* left as reference */
        struct aircraftpce_term *H;  /**< The Hessian matrix. */
        struct aircraftpce_term *g;  /**< The gradient vector. */
        struct aircraftpce_term *u_lb;  /**< The lower bound for the box constraints. */
        struct aircraftpce_term *u_ub;  /**< The upper bound for the box constraints. */
        struct aircraftpce_term *V;  /**< The mixed constraints matrix. */
        struct aircraftpce_term *v_lb;  /**< The lower bound for the mixed constraints. */
        struct aircraftpce_term *v_ub;  /**< The upper bound for the mixed constraints. */
#endif
    };  /**< AIRCRAFTPCE quadratic program form for a given system state x. 
 * The quadratic program to solve has the form:
 * minimize 0.5 * u^T * HoL * u + u^T * goL
 * subject to u_lb <= u <= u_ub
 *            v_lb <= V * x <= v_ub
 *
 * the transpose of a matrix is denote by ^T.
 */

    struct aircraftpce_cvp {
        struct aircraftpce_term *par[AIRCRAFTPCE_PAR_NUM];
        struct aircraftpce_term *constant[AIRCRAFTPCE_CONSTANT_NUM];
        struct aircraftpce_pmetric *pmetric[AIRCRAFTPCE_PMETRIC_NUM];
        struct aircraftpce_cvp_prb *prb;
    };

extern void aircraftpce_cvp_form_problem(struct aircraftpce_cvp *cvp);

    struct aircraftpce_cvp_former {
      struct aircraftpce_cvp *data;
      void (*form_problem)(const struct aircraftpce_cvp *data);
    };

#endif
