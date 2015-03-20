#include <stdio.h>  /* fopen */
#include <stdlib.h>  /* malloc */
#include "include/cjson.h"
#include "include/aircraftpcecvpdynmem.h"

/* Static functions declarations */
static void aircraftpce_get_json_term(struct aircraftpce_term *term, cJSON *data, char *jname, char *term_name);
static void aircraftpce_get_json_sub_term(struct aircraftpce_term *term, cJSON *data, char *jname, char *term_name, char *sub_name);
static void aircraftpce_parse_elements(struct aircraftpce_cvp *cvp, cJSON *data);
static void aircraftpce_get_json_fac_term(struct aircraftpce_term *term, cJSON *data, char *jname, char *list_name, int fac_pos);
static void aircraftpce_get_json_term_items(struct aircraftpce_term *term, cJSON *jobj);
static void aircraftpce_alloc_data(double **data, int elems);

/* Extern function definitions */
/* Memory allocation specific for given data */
int aircraftpce_cvp_setup_former(struct aircraftpce_cvp *cvp, char *fname) {
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
        aircraftpce_parse_elements(cvp, data);
        return 0;
}

struct aircraftpce_cvp *aircraftpce_cvp_allocate_former(void) {
  int i;
  struct aircraftpce_term *t;
  struct aircraftpce_pmetric *p;
  struct aircraftpce_cvp *cvp = (struct aircraftpce_cvp*)malloc(sizeof(struct aircraftpce_cvp));
  if (NULL == cvp) return NULL;
  /* parameters */
  t = (struct aircraftpce_term*)calloc(AIRCRAFTPCE_PAR_NUM, sizeof(struct aircraftpce_term));
  if (NULL == t) return NULL;
  for (i=0; i<AIRCRAFTPCE_PAR_NUM; i++) {
   cvp->par[i] = &t[i];
  }

  /* constants */
  t = (struct aircraftpce_term*)calloc(AIRCRAFTPCE_CONSTANT_NUM, sizeof(struct aircraftpce_term));
  if (NULL == t) return NULL;
  for (i=0; i<AIRCRAFTPCE_CONSTANT_NUM; i++) {
   cvp->constant[i] = &t[i];
  }


  /* parametric */
  p = (struct aircraftpce_pmetric*)calloc(AIRCRAFTPCE_PMETRIC_NUM, sizeof(struct aircraftpce_pmetric));
  if (NULL == p) return NULL;
  for (i=0; i<AIRCRAFTPCE_PMETRIC_NUM; i++) {
      cvp->pmetric[i] = &p[i];
      cvp->pmetric[i]->fac_num = (uint32_t*)malloc(sizeof(uint32_t*));
      if (NULL == cvp->pmetric[i]->fac_num) return NULL;
      cvp->pmetric[i]->val = (struct aircraftpce_term*)malloc(sizeof(struct aircraftpce_term));
      if (NULL == cvp->pmetric[i]->val) return NULL;
      cvp->pmetric[i]->aux = (struct aircraftpce_term*)malloc(sizeof(struct aircraftpce_term));
      if (NULL == cvp->pmetric[i]->aux) return NULL;
      cvp->pmetric[i]->fac0 = (struct aircraftpce_term*)malloc(sizeof(struct aircraftpce_term));
      if (NULL == cvp->pmetric[i]->fac0) return NULL;
  }

        cvp->pmetric[AIRCRAFTPCE_V_UB]->fac_num[0] = 1;
        cvp->pmetric[AIRCRAFTPCE_V_UB]->fac = (struct aircraftpce_term**)calloc(1, sizeof(struct aircraftpce_term*));
if (NULL == cvp->pmetric[AIRCRAFTPCE_V_UB]->fac) return NULL;
        cvp->pmetric[AIRCRAFTPCE_V_UB]->par = (struct aircraftpce_term**)calloc(1, sizeof(struct aircraftpce_term*));
if (NULL == cvp->pmetric[AIRCRAFTPCE_V_UB]->par) return NULL;
        t = (struct aircraftpce_term*)calloc(1, sizeof(struct aircraftpce_term));
if (NULL == t) return NULL;
        cvp->pmetric[AIRCRAFTPCE_V_UB]->fac[0] = &t[0];
        cvp->pmetric[AIRCRAFTPCE_V_UB]->par[0] = cvp->par[AIRCRAFTPCE_X_K];
        cvp->pmetric[AIRCRAFTPCE_G]->fac_num[0] = 1;
        cvp->pmetric[AIRCRAFTPCE_G]->fac = (struct aircraftpce_term**)calloc(1, sizeof(struct aircraftpce_term*));
if (NULL == cvp->pmetric[AIRCRAFTPCE_G]->fac) return NULL;
        cvp->pmetric[AIRCRAFTPCE_G]->par = (struct aircraftpce_term**)calloc(1, sizeof(struct aircraftpce_term*));
if (NULL == cvp->pmetric[AIRCRAFTPCE_G]->par) return NULL;
        t = (struct aircraftpce_term*)calloc(1, sizeof(struct aircraftpce_term));
if (NULL == t) return NULL;
        cvp->pmetric[AIRCRAFTPCE_G]->fac[0] = &t[0];
        cvp->pmetric[AIRCRAFTPCE_G]->par[0] = cvp->par[AIRCRAFTPCE_X_K];
        cvp->pmetric[AIRCRAFTPCE_V_LB]->fac_num[0] = 1;
        cvp->pmetric[AIRCRAFTPCE_V_LB]->fac = (struct aircraftpce_term**)calloc(1, sizeof(struct aircraftpce_term*));
if (NULL == cvp->pmetric[AIRCRAFTPCE_V_LB]->fac) return NULL;
        cvp->pmetric[AIRCRAFTPCE_V_LB]->par = (struct aircraftpce_term**)calloc(1, sizeof(struct aircraftpce_term*));
if (NULL == cvp->pmetric[AIRCRAFTPCE_V_LB]->par) return NULL;
        t = (struct aircraftpce_term*)calloc(1, sizeof(struct aircraftpce_term));
if (NULL == t) return NULL;
        cvp->pmetric[AIRCRAFTPCE_V_LB]->fac[0] = &t[0];
        cvp->pmetric[AIRCRAFTPCE_V_LB]->par[0] = cvp->par[AIRCRAFTPCE_X_K];


  /* the evaluated problem itself */
  cvp->prb = (struct aircraftpce_cvp_prb*)malloc(sizeof(struct aircraftpce_cvp_prb));
  if (NULL == cvp->prb) return NULL;
        cvp->prb->v_ub = cvp->pmetric[AIRCRAFTPCE_V_UB]->val;
        cvp->prb->g = cvp->pmetric[AIRCRAFTPCE_G]->val;
        cvp->prb->v_lb = cvp->pmetric[AIRCRAFTPCE_V_LB]->val;
        cvp->prb->H = cvp->constant[AIRCRAFTPCE_H];
        cvp->prb->u_lb = cvp->constant[AIRCRAFTPCE_U_LB];
        cvp->prb->V = cvp->constant[AIRCRAFTPCE_V];
        cvp->prb->u_ub = cvp->constant[AIRCRAFTPCE_U_UB];

    return cvp;
}
/* Static function definitions */
void aircraftpce_parse_elements(struct aircraftpce_cvp *cvp, cJSON *data)
{
        aircraftpce_get_json_term(cvp->par[AIRCRAFTPCE_X_K], data, "par", "x_k");
        aircraftpce_get_json_sub_term(cvp->pmetric[AIRCRAFTPCE_V_UB]->val, data, "pmetric", "v_ub", "val");
        aircraftpce_get_json_sub_term(cvp->pmetric[AIRCRAFTPCE_V_UB]->fac0, data, "pmetric", "v_ub", "fac0");
        aircraftpce_get_json_sub_term(cvp->pmetric[AIRCRAFTPCE_V_UB]->aux, data, "pmetric", "v_ub", "aux");
        aircraftpce_get_json_fac_term(cvp->pmetric[AIRCRAFTPCE_V_UB]->fac[0], data, "pmetric", "v_ub", 0);
        aircraftpce_get_json_sub_term(cvp->pmetric[AIRCRAFTPCE_G]->val, data, "pmetric", "g", "val");
        aircraftpce_get_json_sub_term(cvp->pmetric[AIRCRAFTPCE_G]->fac0, data, "pmetric", "g", "fac0");
        aircraftpce_get_json_sub_term(cvp->pmetric[AIRCRAFTPCE_G]->aux, data, "pmetric", "g", "aux");
        aircraftpce_get_json_fac_term(cvp->pmetric[AIRCRAFTPCE_G]->fac[0], data, "pmetric", "g", 0);
        aircraftpce_get_json_sub_term(cvp->pmetric[AIRCRAFTPCE_V_LB]->val, data, "pmetric", "v_lb", "val");
        aircraftpce_get_json_sub_term(cvp->pmetric[AIRCRAFTPCE_V_LB]->fac0, data, "pmetric", "v_lb", "fac0");
        aircraftpce_get_json_sub_term(cvp->pmetric[AIRCRAFTPCE_V_LB]->aux, data, "pmetric", "v_lb", "aux");
        aircraftpce_get_json_fac_term(cvp->pmetric[AIRCRAFTPCE_V_LB]->fac[0], data, "pmetric", "v_lb", 0);
        aircraftpce_get_json_term(cvp->constant[AIRCRAFTPCE_H], data, "constant", "H");
        aircraftpce_get_json_term(cvp->constant[AIRCRAFTPCE_U_LB], data, "constant", "u_lb");
        aircraftpce_get_json_term(cvp->constant[AIRCRAFTPCE_V], data, "constant", "V");
        aircraftpce_get_json_term(cvp->constant[AIRCRAFTPCE_U_UB], data, "constant", "u_ub");

return;
}
void aircraftpce_get_json_term(struct aircraftpce_term *term, cJSON *data, char *jname, char *term_name)
{
            cJSON *jobj = cJSON_GetObjectItem(data, jname);
            cJSON *jterm = cJSON_GetObjectItem(jobj, term_name);
            aircraftpce_get_json_term_items(term, jterm);
            return;
 }

void aircraftpce_get_json_sub_term(struct aircraftpce_term *term, cJSON *data, char *jname, char *term_name, char *sub_name)
{
            cJSON *jobj = cJSON_GetObjectItem(data, jname);
            cJSON *jpmetric = cJSON_GetObjectItem(jobj, term_name);
            cJSON *jterm = cJSON_GetObjectItem(jpmetric, sub_name);
            aircraftpce_get_json_term_items(term, jterm);
            return;
 }

void aircraftpce_get_json_fac_term(struct aircraftpce_term *term, cJSON *data, char *jname, char *list_name, int fac_pos)
{
            cJSON *jobj = cJSON_GetObjectItem(data, jname);
            cJSON *jpmetric = cJSON_GetObjectItem(jobj, list_name);
            cJSON *jfac_list = cJSON_GetObjectItem(jpmetric, "fac");
            cJSON *jfac = cJSON_GetArrayItem(jfac_list, fac_pos);
            aircraftpce_get_json_term_items(term, jfac);
            return;
 }

void aircraftpce_get_json_term_items(struct aircraftpce_term *term, cJSON *jobj)
{
            term->cols = cJSON_GetObjectItem(jobj, "cols")->valueint;
            term->rows = cJSON_GetObjectItem(jobj, "rows")->valueint;
            cJSON *jdata = cJSON_GetObjectItem(jobj, "data");
            int elems = term->rows * term->cols;

            if (elems != cJSON_GetArraySize(jdata)) {
                printf("Size error\n");
            } else {
               aircraftpce_alloc_data(&(term->data), elems);
                int i;
                for (i=0;i<elems;i++) {
                term->data[i] = cJSON_GetArrayItem(jdata, i)->valuedouble;
              }
            }
            return;
 }

void aircraftpce_alloc_data(double **data, int elems)
{
        data[0] = (double*) malloc(elems*sizeof(double));
        if (NULL == data[0]) {
          printf("Memory error");
        }
        return;
 }
