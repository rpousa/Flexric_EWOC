/*
 * xapp_cuup_reloc_trigger -- move a UE to another gNB-CU-UP when its DL RLC
 * buffer stays full.
 *
 * Test xApp for the inter-CU-UP relocation (TS 38.401 8.9.5) of the cuup-reloc
 * branch. It
 *   1. subscribes to E2SM-KPM REPORT Style 4 on every gNB-DU, asking only for
 *      DRB.RlcBufOccDl (bytes in the UE's DL RLC buffer, a vendor-specific
 *      measurement of OAI_EWOC's DU), one subscription per watched S-NSSAI;
 *   2. watches each UE's value, and the first time one UE is above the
 *      threshold on RELOC_CONSECUTIVE reports in a row,
 *   3. sends one E2SM-RC CONTROL to the gNB-CU-CP: Style 3, action 1000
 *      "gNB-CU-UP Change Control", naming that UE and, optionally, the target
 *      gNB-CU-UP ID;
 *   4. keeps printing the buffer for RELOC_LINGER_S seconds and exits.
 *
 * It triggers once per run, on purpose: a relocation that did not relieve the
 * buffer must not turn into a ping-pong between CU-UPs.
 *
 * The UE identity is free: KPM reports a DU UE as gNB-CU UE F1AP ID, and in
 * OAI that is the CU-CP's rrc_ue_id, which is what the CONTROL handler looks
 * up.
 *
 * Settings come from the environment, so the usual FlexRIC -c/-d/-n options
 * keep working:
 *   RELOC_THRESHOLD_BYTES  default 100000
 *   RELOC_CONSECUTIVE      default 3      reports in a row above the threshold
 *   RELOC_PERIOD_MS        default 1000   KPM report period
 *   RELOC_UE               default 0      only watch this rrc_ue_id (0 = any)
 *   RELOC_TARGET_CUUP      default -1     gNB-CU-UP ID to move to (-1 = any other)
 *   RELOC_LINGER_S         default 30     keep reporting after the trigger
 *   RELOC_TIMEOUT_S        default 600    give up if nothing triggers
 *
 * Run it with its own DB, or it will share (and lock) the daemon's:
 *   xapp_cuup_reloc_trigger -d /tmp/ -n reloc_trigger_db
 */

#include "../../../../src/xApp/e42_xapp_api.h"
#include "../../../../src/sm/rc_sm/ie/ir/ran_param_struct.h"
#include "../../../../src/sm/rc_sm/ie/ir/ran_param_list.h"
#include "../../../../src/util/byte_array.h"
#include "../../../../src/util/time_now_us.h"

#include <assert.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define KPM_RAN_FUNC_ID 2
#define RC_RAN_FUNC_ID 3

/* Must match ran_func_rc.c (RC_CTRL_ACT_CUUP_CHANGE / RC_PARAM_TARGET_CUUP_ID)
 * and ran_func_kpm.c (the DU measurement). */
#define RC_STYLE_CONN_MODE_MOBILITY 3
#define RC_CTRL_ACT_CUUP_CHANGE 1000
#define RC_PARAM_TARGET_CUUP_ID 1000
static const char METRIC[] = "DRB.RlcBufOccDl";

/* S-NSSAIs to watch, as in xapp_kpm_moni: the DU's KPM Style 4 filters UEs by
 * one S-NSSAI per subscription. */
typedef struct {
  uint8_t sst;
  uint32_t sd;
} snssai_t;
static const snssai_t WATCH_NSSAI[] = {{1, 1}, {2, 1}, {3, 1}};
#define WATCH_NSSAI_LEN (sizeof(WATCH_NSSAI) / sizeof(WATCH_NSSAI[0]))

#define MAX_UES 64

typedef struct {
  long threshold;
  int consecutive;
  uint64_t period_ms;
  uint32_t only_ue;
  long target_cuup;
  int linger_s;
  int timeout_s;
} cfg_t;

static cfg_t cfg;

static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static struct {
  uint32_t rrc_ue_id;
  int above; // consecutive reports above the threshold
  uint32_t last;
  bool used;
} ues[MAX_UES];
static bool pending = false; // a UE crossed the line, main thread must act
static bool fired = false; // the CONTROL has been sent (once per run)
static uint32_t pending_ue = 0;
static uint32_t pending_bytes = 0;

static volatile sig_atomic_t stop = 0;
static void on_signal(int sig)
{
  (void)sig;
  stop = 1;
}

static long env_long(const char* name, long dflt)
{
  const char* v = getenv(name);
  return (v != NULL && *v != '\0') ? strtol(v, NULL, 0) : dflt;
}

/* ---- KPM indication ---------------------------------------------------- */

static void on_ue_value(uint32_t rrc_ue_id, uint32_t bytes)
{
  if (cfg.only_ue != 0 && rrc_ue_id != cfg.only_ue)
    return;

  pthread_mutex_lock(&mtx);
  int slot = -1;
  for (int i = 0; i < MAX_UES; i++) {
    if (ues[i].used && ues[i].rrc_ue_id == rrc_ue_id) {
      slot = i;
      break;
    }
    if (slot < 0 && !ues[i].used)
      slot = i;
  }
  if (slot >= 0) {
    if (!ues[slot].used) {
      ues[slot].used = true;
      ues[slot].rrc_ue_id = rrc_ue_id;
      ues[slot].above = 0;
    }
    ues[slot].last = bytes;
    ues[slot].above = (long)bytes > cfg.threshold ? ues[slot].above + 1 : 0;

    printf("[RELOC] UE %u %s = %u bytes%s (%d/%d above %ld)\n",
           rrc_ue_id,
           METRIC,
           bytes,
           fired ? " [after trigger]" : "",
           ues[slot].above,
           cfg.consecutive,
           cfg.threshold);

    if (!fired && !pending && ues[slot].above >= cfg.consecutive) {
      pending = true;
      pending_ue = rrc_ue_id;
      pending_bytes = bytes;
    }
  }
  pthread_mutex_unlock(&mtx);
}

static void sm_cb_kpm(sm_ag_if_rd_t const* rd)
{
  assert(rd != NULL);
  if (rd->type != INDICATION_MSG_AGENT_IF_ANS_V0 || rd->ind.type != KPM_STATS_V3_0)
    return;

  kpm_ind_data_t const* ind = &rd->ind.kpm.ind;
  if (ind->msg.type != FORMAT_3_INDICATION_MESSAGE)
    return;

  kpm_ind_msg_format_3_t const* frm_3 = &ind->msg.frm_3;
  for (size_t u = 0; u < frm_3->ue_meas_report_lst_len; u++) {
    ue_id_e2sm_t const* id = &frm_3->meas_report_per_ue[u].ue_meas_report_lst;
    if (id->type != GNB_DU_UE_ID_E2SM)
      continue;
    const uint32_t rrc_ue_id = id->gnb_du.gnb_cu_ue_f1ap; // == CU-CP rrc_ue_id in OAI

    kpm_ind_msg_format_1_t const* m = &frm_3->meas_report_per_ue[u].ind_msg_format_1;
    if (m->meas_data_lst_len == 0)
      continue;
    /* One label per measurement in our action definition, so record k is
     * measurement k. Use the last granularity period of the report. */
    meas_data_lst_t const* data = &m->meas_data_lst[m->meas_data_lst_len - 1];
    for (size_t k = 0; k < m->meas_info_lst_len && k < data->meas_record_len; k++) {
      meas_type_t const* t = &m->meas_info_lst[k].meas_type;
      if (t->type != NAME_MEAS_TYPE || cmp_str_ba(METRIC, t->name) != 0)
        continue;
      meas_record_lst_t const* r = &data->meas_record_lst[k];
      if (r->value == INTEGER_MEAS_VALUE)
        on_ue_value(rrc_ue_id, r->int_val);
      else if (r->value == REAL_MEAS_VALUE)
        on_ue_value(rrc_ue_id, (uint32_t)r->real_val);
    }
  }
}

/* ---- KPM subscription --------------------------------------------------- */

static test_info_lst_t nssai_predicate(uint8_t sst, uint32_t sd)
{
  test_info_lst_t dst = {0};
  dst.test_cond_type = S_NSSAI_TEST_COND_TYPE;
  dst.S_NSSAI = TRUE_TEST_COND_TYPE;

  dst.test_cond = calloc(1, sizeof(test_cond_e));
  assert(dst.test_cond != NULL && "Memory exhausted");
  *dst.test_cond = EQUAL_TEST_COND;

  dst.test_cond_value = calloc(1, sizeof(test_cond_value_t));
  assert(dst.test_cond_value != NULL && "Memory exhausted");
  dst.test_cond_value->type = OCTET_STRING_TEST_COND_VALUE;
  dst.test_cond_value->octet_string_value = calloc(1, sizeof(byte_array_t));
  assert(dst.test_cond_value->octet_string_value != NULL && "Memory exhausted");

  uint8_t* b = calloc(4, sizeof(uint8_t)); // 1 B SST + 3 B SD, as xapp_kpm_moni sends it
  assert(b != NULL && "Memory exhausted");
  b[0] = sst;
  b[1] = (sd >> 16) & 0xff;
  b[2] = (sd >> 8) & 0xff;
  b[3] = sd & 0xff;
  dst.test_cond_value->octet_string_value->len = 4;
  dst.test_cond_value->octet_string_value->buf = b;
  return dst;
}

static kpm_sub_data_t gen_sub(snssai_t nssai)
{
  kpm_sub_data_t sub = {0};
  sub.ev_trg_def.type = FORMAT_1_RIC_EVENT_TRIGGER;
  sub.ev_trg_def.kpm_ric_event_trigger_format_1.report_period_ms = cfg.period_ms;

  sub.sz_ad = 1;
  sub.ad = calloc(1, sizeof(kpm_act_def_t));
  assert(sub.ad != NULL && "Memory exhausted");
  kpm_act_def_t* ad = sub.ad;
  ad->type = FORMAT_4_ACTION_DEFINITION;

  ad->frm_4.matching_cond_lst_len = 1;
  ad->frm_4.matching_cond_lst = calloc(1, sizeof(matching_condition_format_4_lst_t));
  assert(ad->frm_4.matching_cond_lst != NULL && "Memory exhausted");
  ad->frm_4.matching_cond_lst[0].test_info_lst = nssai_predicate(nssai.sst, nssai.sd);

  kpm_act_def_format_1_t* f1 = &ad->frm_4.action_def_format_1;
  f1->meas_info_lst_len = 1;
  f1->meas_info_lst = calloc(1, sizeof(meas_info_format_1_lst_t));
  assert(f1->meas_info_lst != NULL && "Memory exhausted");
  f1->meas_info_lst[0].meas_type.type = NAME_MEAS_TYPE;
  f1->meas_info_lst[0].meas_type.name = cp_str_to_ba(METRIC);
  f1->meas_info_lst[0].label_info_lst_len = 1;
  f1->meas_info_lst[0].label_info_lst = calloc(1, sizeof(label_info_lst_t));
  assert(f1->meas_info_lst[0].label_info_lst != NULL && "Memory exhausted");
  f1->meas_info_lst[0].label_info_lst[0].noLabel = calloc(1, sizeof(enum_value_e));
  assert(f1->meas_info_lst[0].label_info_lst[0].noLabel != NULL && "Memory exhausted");
  *f1->meas_info_lst[0].label_info_lst[0].noLabel = TRUE_ENUM_VALUE;
  f1->gran_period_ms = cfg.period_ms;
  f1->cell_global_id = NULL;
  f1->meas_bin_range_info_lst_len = 0;
  f1->meas_bin_info_lst = NULL;

  return sub;
}

static sm_ran_function_t const* find_rf(e2_node_connected_xapp_t const* n, int id)
{
  for (size_t i = 0; i < n->len_rf; i++)
    if (n->rf[i].id == id)
      return &n->rf[i];
  return NULL;
}

/* Does this DU's KPM REPORT Style 4 offer our measurement? */
static bool du_offers_metric(sm_ran_function_t const* rf)
{
  if (rf == NULL || rf->defn.type != KPM_RAN_FUNC_DEF_E)
    return false;
  for (size_t j = 0; j < rf->defn.kpm.sz_ric_report_style_list; j++) {
    ric_report_style_item_t const* it = &rf->defn.kpm.ric_report_style_list[j];
    if (it->report_style_type != STYLE_4_RIC_SERVICE_REPORT)
      continue;
    for (size_t k = 0; k < it->meas_info_for_action_lst_len; k++)
      if (cmp_str_ba(METRIC, it->meas_info_for_action_lst[k].name) == 0)
        return true;
  }
  return false;
}

/* ---- RC CONTROL --------------------------------------------------------- */

static rc_ctrl_req_data_t gen_cuup_change(global_e2_node_id_t const* cucp, uint32_t rrc_ue_id, long target_cuup)
{
  rc_ctrl_req_data_t req = {0};

  req.hdr.format = FORMAT_1_E2SM_RC_CTRL_HDR;
  req.hdr.frmt_1.ric_style_type = RC_STYLE_CONN_MODE_MOBILITY;
  req.hdr.frmt_1.ctrl_act_id = RC_CTRL_ACT_CUUP_CHANGE;

  /* gNB UE ID: AMF UE NGAP ID and GUAMI are mandatory in the IE but OAI only
   * reads the RAN UE ID, which must be the CU-CP's rrc_ue_id. The PLMN is the
   * CU-CP's own, so the IE stays well formed. */
  req.hdr.frmt_1.ue_id.type = GNB_UE_ID_E2SM;
  req.hdr.frmt_1.ue_id.gnb.amf_ue_ngap_id = 0;
  req.hdr.frmt_1.ue_id.gnb.guami.plmn_id.mcc = cucp->plmn.mcc;
  req.hdr.frmt_1.ue_id.gnb.guami.plmn_id.mnc = cucp->plmn.mnc;
  req.hdr.frmt_1.ue_id.gnb.guami.plmn_id.mnc_digit_len = cucp->plmn.mnc_digit_len;
  req.hdr.frmt_1.ue_id.gnb.ran_ue_id = calloc(1, sizeof(uint64_t));
  assert(req.hdr.frmt_1.ue_id.gnb.ran_ue_id != NULL && "Memory exhausted");
  *req.hdr.frmt_1.ue_id.gnb.ran_ue_id = rrc_ue_id;

  req.msg.format = FORMAT_1_E2SM_RC_CTRL_MSG;
  if (target_cuup >= 0) {
    req.msg.frmt_1.sz_ran_param = 1;
    req.msg.frmt_1.ran_param = calloc(1, sizeof(seq_ran_param_t));
    assert(req.msg.frmt_1.ran_param != NULL && "Memory exhausted");
    seq_ran_param_t* p = &req.msg.frmt_1.ran_param[0];
    p->ran_param_id = RC_PARAM_TARGET_CUUP_ID;
    p->ran_param_val.type = ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE;
    p->ran_param_val.flag_false = calloc(1, sizeof(ran_parameter_value_t));
    assert(p->ran_param_val.flag_false != NULL && "Memory exhausted");
    p->ran_param_val.flag_false->type = INTEGER_RAN_PARAMETER_VALUE;
    p->ran_param_val.flag_false->int_ran = target_cuup;
  } else {
    req.msg.frmt_1.sz_ran_param = 0;
    req.msg.frmt_1.ran_param = NULL;
  }
  return req;
}

/* ---- main --------------------------------------------------------------- */

int main(int argc, char* argv[])
{
  cfg.threshold = env_long("RELOC_THRESHOLD_BYTES", 100000);
  cfg.consecutive = (int)env_long("RELOC_CONSECUTIVE", 3);
  cfg.period_ms = (uint64_t)env_long("RELOC_PERIOD_MS", 1000);
  cfg.only_ue = (uint32_t)env_long("RELOC_UE", 0);
  cfg.target_cuup = env_long("RELOC_TARGET_CUUP", -1);
  cfg.linger_s = (int)env_long("RELOC_LINGER_S", 30);
  cfg.timeout_s = (int)env_long("RELOC_TIMEOUT_S", 600);
  if (cfg.consecutive < 1)
    cfg.consecutive = 1;

  printf("[RELOC] threshold %ld bytes, %d reports in a row, period %lu ms, UE %s, target gNB-CU-UP %s\n",
         cfg.threshold,
         cfg.consecutive,
         (unsigned long)cfg.period_ms,
         cfg.only_ue ? "fixed" : "any",
         cfg.target_cuup >= 0 ? "fixed" : "any other");
  if (cfg.only_ue)
    printf("[RELOC] watching only rrc_ue_id %u\n", cfg.only_ue);
  if (cfg.target_cuup >= 0)
    printf("[RELOC] target gNB-CU-UP ID %ld (0x%lx)\n", cfg.target_cuup, cfg.target_cuup);

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);

  fr_args_t args = init_fr_args(argc, argv);
  init_xapp_api(&args);
  sleep(1);

  e2_node_arr_xapp_t nodes = e2_nodes_xapp_api();
  printf("[RELOC] %d E2 node(s) connected\n", nodes.len);

  global_e2_node_id_t const* cucp = NULL;
  sm_ans_xapp_t hndl[16 * WATCH_NSSAI_LEN];
  size_t n_hndl = 0;

  for (size_t i = 0; i < nodes.len; i++) {
    e2_node_connected_xapp_t* n = &nodes.n[i];
    if (n->id.type == ngran_gNB_CUCP || n->id.type == ngran_gNB_CU || n->id.type == ngran_gNB) {
      if (find_rf(n, RC_RAN_FUNC_ID) != NULL && cucp == NULL)
        cucp = &n->id;
    }
    if (n->id.type != ngran_gNB_DU && n->id.type != ngran_gNB)
      continue;

    sm_ran_function_t const* rf = find_rf(n, KPM_RAN_FUNC_ID);
    if (!du_offers_metric(rf)) {
      printf("[RELOC] DU (nb_id %u) does not offer %s in KPM Style 4 -- is it built from the cuup-reloc branch "
             "with this metric?\n",
             n->id.nb_id.nb_id,
             METRIC);
      continue;
    }
    for (size_t k = 0; k < WATCH_NSSAI_LEN && n_hndl < sizeof(hndl) / sizeof(hndl[0]); k++) {
      kpm_sub_data_t sub = gen_sub(WATCH_NSSAI[k]);
      hndl[n_hndl] = report_sm_xapp_api(&n->id, KPM_RAN_FUNC_ID, &sub, sm_cb_kpm);
      free_kpm_sub_data(&sub);
      if (hndl[n_hndl].success)
        n_hndl++;
      else
        printf("[RELOC] KPM subscription for S-NSSAI (%u, %u) on DU nb_id %u failed\n",
               WATCH_NSSAI[k].sst,
               WATCH_NSSAI[k].sd,
               n->id.nb_id.nb_id);
    }
  }

  if (cucp == NULL)
    printf("[RELOC] no gNB-CU-CP with E2SM-RC connected: can watch, cannot trigger\n");
  if (n_hndl == 0)
    printf("[RELOC] no KPM subscription succeeded, nothing to watch\n");

  const int64_t t0 = time_now_us();
  int64_t fired_at = 0;
  while (!stop && n_hndl > 0) {
    usleep(100 * 1000);
    const int64_t now = time_now_us();

    bool send = false;
    uint32_t ue = 0, bytes = 0;
    pthread_mutex_lock(&mtx);
    if (pending && !fired) {
      send = true;
      ue = pending_ue;
      bytes = pending_bytes;
      fired = true;
      pending = false;
    }
    pthread_mutex_unlock(&mtx);

    if (send) {
      printf("[RELOC] UE %u: %s %u bytes above %ld on %d reports in a row -> gNB-CU-UP Change\n",
             ue,
             METRIC,
             bytes,
             cfg.threshold,
             cfg.consecutive);
      if (cucp != NULL) {
        rc_ctrl_req_data_t req = gen_cuup_change(cucp, ue, cfg.target_cuup);
        sm_ans_xapp_t ans = control_sm_xapp_api((global_e2_node_id_t*)cucp, RC_RAN_FUNC_ID, &req);
        free_rc_ctrl_req_data(&req);
        printf("[RELOC] CONTROL sent to the gNB-CU-CP: %s\n", ans.success ? "acknowledged" : "NOT acknowledged");
      }
      fired_at = now;
    }

    if (fired_at != 0 && now - fired_at > (int64_t)cfg.linger_s * 1000000)
      break;
    if (fired_at == 0 && now - t0 > (int64_t)cfg.timeout_s * 1000000) {
      printf("[RELOC] no UE above %ld bytes for %d reports within %d s, giving up\n",
             cfg.threshold,
             cfg.consecutive,
             cfg.timeout_s);
      break;
    }
  }

  for (size_t i = 0; i < n_hndl; i++)
    rm_report_sm_xapp_api(hndl[i].u.handle);

  while (try_stop_xapp_api() == false)
    usleep(1000);
  free_e2_node_arr_xapp(&nodes);

  printf("[RELOC] done%s\n", fired_at ? "" : " (no trigger)");
  return 0;
}
