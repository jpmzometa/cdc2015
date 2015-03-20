#include <stdio.h>  /* fopen */
#include <stdlib.h>  /* malloc */

#include "include/cjson.h"
#include "include/aircraftpcecvp.h"
#include "include/aircraftpcefgmdynmem.h"

int aircraftpce_fgm_parse_json(struct aircraftpce_fgm *fgm, char *fname);
void aircraftpce_fgm_parse_elements(struct aircraftpce_fgm *fgm, cJSON *data);

/* Extern function definitions */

struct aircraftpce_fgm *aircraftpce_fgm_allocate_solver(void)
{
      struct aircraftpce_fgm *fgm = (struct aircraftpce_fgm*)malloc(sizeof(struct aircraftpce_fgm));
      if (NULL == fgm) return NULL;
      struct aircraftpce_fgm_conf *conf = (struct aircraftpce_fgm_conf*)malloc(sizeof(struct aircraftpce_fgm_conf));
      if (NULL == conf) return NULL;
      fgm->conf = conf;
    return fgm;
}

void aircraftpce_fgm_setup_solver(struct aircraftpce_fgm *fgm, struct aircraftpce_cvp_prb *prb, char *fname)
{
  fgm->goL = prb->g->data;
  fgm->HoL = prb->H->data;
  fgm->u_lb = prb->u_lb->data;
  fgm->u_ub = prb->u_ub->data;
  fgm->optvar_seqlen = prb->u_lb->rows;
  /* fgm->optvar_veclen = ; FIXME this is needed for warmstart */
  fgm->sizeof_optvar_seqlen = sizeof(real_t) * prb->u_lb->rows;

  fgm->u_ini = (real_t *)malloc(fgm->sizeof_optvar_seqlen);
  fgm->u_opt = (real_t *)malloc(fgm->sizeof_optvar_seqlen);

  fgm->tmp1_optvar_seqlen = (real_t *)malloc(fgm->sizeof_optvar_seqlen);
  fgm->tmp2_optvar_seqlen = (real_t *)malloc(fgm->sizeof_optvar_seqlen);
  fgm->tmp3_optvar_seqlen = (real_t *)malloc(fgm->sizeof_optvar_seqlen);

  fgm->j_in = (uint32_t *)malloc(sizeof(uint32_t));
  aircraftpce_fgm_parse_json(fgm, fname);
}

/* Static function definitions */

int aircraftpce_fgm_parse_json(struct aircraftpce_fgm *fgm, char *fname) {
        int i;
        char *fdata;
        FILE *f=fopen(fname,"rb");
        if (NULL == f) return 1;
        fseek(f,0,SEEK_END);long len=ftell(f);fseek(f,0,SEEK_SET);
        fdata=(char*)malloc(len+1);
        if (NULL == fdata) return 1;
        fread(fdata,1,len,f);fclose(f);
        cJSON *data;
        data = cJSON_Parse(fdata);
        free(fdata);
        aircraftpce_fgm_parse_elements(fgm, data);
        return 0;
}

void aircraftpce_fgm_parse_elements(struct aircraftpce_fgm *fgm, cJSON *data)
{
    fgm->nu = (real_t *)malloc(sizeof(real_t));
    *(fgm->nu) = (real_t)cJSON_GetObjectItem(data, "nu")->valuedouble;
    return;
}

