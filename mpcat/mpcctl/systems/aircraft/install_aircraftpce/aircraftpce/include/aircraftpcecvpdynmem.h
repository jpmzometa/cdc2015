#include "aircraftpcecvp.h"

int aircraftpce_cvp_setup_former(struct aircraftpce_cvp *cvp, char *fname);
struct aircraftpce_cvp *aircraftpce_cvp_allocate_former(void);

    struct aircraftpce_cvp_former_dynmem {
      struct aircraftpce_cvp_former *former;
      int (*setup_problem)(struct aircraftpce_cvp *cvp, char *fname);
      struct aircraftpce_cvp *(*allocate_former)(void);
    };

