/*
 * xapp_cuup_reloc_trigger -- move a UE to another gNB-CU-UP when a chosen KPM
 * measurement crosses a threshold.
 *
 * Test xApp for the inter-CU-UP relocation (TS 38.401 8.9.5) of the cuup-reloc
 * branch. Which measurement drives it is chosen at start:
 *
 *   1. discovers, from every E2 node's KPM RAN function definition, which nodes
 *      offer the chosen measurement in REPORT Style 4 (per-UE) -- a gNB-DU, a
 *      gNB-CU-UP or a monolithic gNB, whichever has it;
 *   2. subscribes there (one subscription per watched S-NSSAI, since OAI's
 *      Style 4 matches UEs on one S-NSSAI per subscription), adding the guard
 *      measurement to the same subscription when the node offers it too;
 *   3. for each UE, counts consecutive reports on which the measurement is
 *      above (or below) the threshold -- and, if a guard is set, the guard's
 *      latest value also satisfies its own comparison;
 *   4. the first time a UE reaches RELOC_CONSECUTIVE, sends one E2SM-RC CONTROL
 *      to the gNB-CU-CP: Style 3, vendor action 1000 "gNB-CU-UP Change Control",
 *      naming that UE and, optionally, the target gNB-CU-UP ID;
 *   5. keeps printing for RELOC_LINGER_S seconds and exits.
 *
 * It triggers once per run, on purpose: a relocation that did not help must not
 * turn into a ping-pong between CU-UPs.
 *
 * The UE identity needs no mapping in OAI: a gNB-DU reports the gNB-CU UE F1AP
 * ID, a gNB-CU-UP the gNB-CU-CP UE E1AP ID, and both equal the CU-CP's
 * rrc_ue_id, which is what the CONTROL names.
 *
 * Measurements of OAI_EWOC worth using (TS 28.552 unless noted):
 *   DRB.PdcpSduDelayDl   gNB-CU-UP  mean DL delay NG-U in -> F1-U out, 0.1 ms
 *                                   (5.1.3.3.1). High = this CU-UP is the
 *                                   bottleneck: the textbook reason to move.
 *   DRB.PdcpSduVolumeDL  gNB-CU-UP  DL volume per period, Mb (5.1.2.1.1.1).
 *   DRB.PdcpSduVolumeUL  gNB-CU-UP  UL volume per period, Mb.
 *   DRB.RlcSduDelayDl    gNB-DU     mean DL delay in RLC, us. A good GUARD:
 *                                   if the radio is the bottleneck, another
 *                                   CU-UP will not help.
 *   DRB.UEThpDl/Ul       gNB-DU     UE throughput, kbps.
 *   DRB.RlcBufOccDl      gNB-DU     DL RLC buffer, bytes (vendor-specific).
 * RELOC_LIST=1 prints what each connected node actually offers, and exits.
 *
 * Settings, from the environment so the FlexRIC -c/-d/-n options keep working:
 *   RELOC_METRIC           default DRB.PdcpSduDelayDl
 *   RELOC_CMP              gt | lt, default gt
 *   RELOC_THRESHOLD        default 10 (= 1 ms for DRB.PdcpSduDelayDl)
 *   RELOC_GUARD_METRIC     optional second measurement that must also hold
 *   RELOC_GUARD_CMP        gt | lt, default lt
 *   RELOC_GUARD_THRESHOLD  default 0
 *   RELOC_CONSECUTIVE      default 3      reports in a row
 *   RELOC_PERIOD_MS        default 1000   KPM report period
 *   RELOC_UE               default 0      only watch this rrc_ue_id (0 = any)
 *   RELOC_TARGET_CUUP      default -1     gNB-CU-UP ID to move to (-1 = any other)
 *   RELOC_LINGER_S         default 30     keep reporting after the trigger
 *   RELOC_TIMEOUT_S        default 600    give up if nothing triggers
 *   RELOC_LIST             1 = list the measurements each node offers and exit
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

/* Must match ran_func_rc.c (RC_CTRL_ACT_CUUP_CHANGE / RC_PARAM_TARGET_CUUP_ID). */
#define RC_STYLE_CONN_MODE_MOBILITY 3
#define RC_CTRL_ACT_CUUP_CHANGE 1000
#define RC_PARAM_TARGET_CUUP_ID 1000

typedef struct {
  uint8_t sst;
  uint32_t sd;
} snssai_t;
static const snssai_t WATCH_NSSAI[] = {{1, 1}, {2, 1}, {3, 1}};
#define WATCH_NSSAI_LEN (sizeof(WATCH_NSSAI) / sizeof(WATCH_NSSAI[0]))

#define MAX_UES 64
#define MAX_HANDLES 128

typedef struct {
  const char* metric;
  bool lt; // trigger when below instead of above
  double threshold;
  const char* guard; // NULL = no guard
  bool guard_lt;
  double guard_threshold;
  int consecutive;
  uint64_t period_ms;
  uint32_t only_ue;
  long target_cuup;
  int linger_s;
  int timeout_s;
  bool list_only;
} cfg_t;

static cfg_t cfg;

static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static struct {
  bool used;
  uint32_t rrc_ue_id;
  int streak; // consecutive primary reports satisfying the rule
  double guard_val;
  int64_t guard_at_us; // 0 = never seen
} ues[MAX_UES];
static bool pending = false;
static bool fired = false;
static uint32_t pending_ue = 0;
static double pending_val = 0;

static volatile sig_atomic_t stop = 0;
static void on_signal(int sig)
{
  (void)sig;
  stop = 1;
}

static const char* env_str(const char* name, const char* dflt)
{
  const char* v = getenv(name);
  return (v != NULL && *v != '\0') ? v : dflt;
}

static double env_double(const char* name, double dflt)
{
  const char* v = getenv(name);
  return (v != NULL && *v != '\0') ? strtod(v, NULL) : dflt;
}

static bool env_lt(const char* name, bool dflt)
{
  const char* v = getenv(name);
  if (v == NULL || *v == '\0')
    return dflt;
  if (strcmp(v, "lt") == 0 || strcmp(v, "<") == 0 || strcmp(v, "below") == 0)
    return true;
  if (strcmp(v, "gt") == 0 || strcmp(v, ">") == 0 || strcmp(v, "above") == 0)
    return false;
  fprintf(stderr, "[RELOC] %s=%s not understood, using %s\n", name, v, dflt ? "lt" : "gt");
  return dflt;
}

static bool holds(double v, bool lt, double thr)
{
  return lt ? v < thr : v > thr;
}

/* ---- KPM indication ---------------------------------------------------- */

static bool rrc_ue_id_of(ue_id_e2sm_t const* id, uint32_t* out)
{
  switch (id->type) {
    case GNB_DU_UE_ID_E2SM:
      *out = id->gnb_du.gnb_cu_ue_f1ap;
      return true;
    case GNB_CU_UP_UE_ID_E2SM:
      *out = id->gnb_cu_up.gnb_cu_cp_ue_e1ap;
      return true;
    case GNB_UE_ID_E2SM:
      if (id->gnb.ran_ue_id != NULL) {
        *out = (uint32_t)*id->gnb.ran_ue_id;
        return true;
      }
      if (id->gnb.gnb_cu_ue_f1ap_lst_len > 0) {
        *out = id->gnb.gnb_cu_ue_f1ap_lst[0];
        return true;
      }
      return false;
    default:
      return false;
  }
}

static int ue_slot(uint32_t rrc_ue_id) // with mtx held
{
  int free_slot = -1;
  for (int i = 0; i < MAX_UES; i++) {
    if (ues[i].used && ues[i].rrc_ue_id == rrc_ue_id)
      return i;
    if (free_slot < 0 && !ues[i].used)
      free_slot = i;
  }
  if (free_slot >= 0) {
    memset(&ues[free_slot], 0, sizeof(ues[free_slot]));
    ues[free_slot].used = true;
    ues[free_slot].rrc_ue_id = rrc_ue_id;
  }
  return free_slot;
}

static void on_value(uint32_t rrc_ue_id, bool is_guard, double v)
{
  if (cfg.only_ue != 0 && rrc_ue_id != cfg.only_ue)
    return;

  const int64_t now = time_now_us();
  pthread_mutex_lock(&mtx);
  const int s = ue_slot(rrc_ue_id);
  if (s < 0) {
    pthread_mutex_unlock(&mtx);
    return;
  }

  if (is_guard) {
    ues[s].guard_val = v;
    ues[s].guard_at_us = now;
    pthread_mutex_unlock(&mtx);
    return;
  }

  bool ok = holds(v, cfg.lt, cfg.threshold);
  bool guard_ok = true;
  if (cfg.guard != NULL) {
    /* A guard older than three report periods is as good as absent: refuse. */
    const bool fresh = ues[s].guard_at_us != 0 && now - ues[s].guard_at_us <= (int64_t)(3 * cfg.period_ms * 1000);
    guard_ok = fresh && holds(ues[s].guard_val, cfg.guard_lt, cfg.guard_threshold);
  }
  ues[s].streak = (ok && guard_ok) ? ues[s].streak + 1 : 0;

  if (cfg.guard != NULL)
    printf("[RELOC] UE %u %s = %.3f (%s), %s = %.3f (%s) -> %d/%d%s\n",
           rrc_ue_id,
           cfg.metric,
           v,
           ok ? "over the line" : "fine",
           cfg.guard,
           ues[s].guard_val,
           guard_ok ? "allows" : "blocks",
           ues[s].streak,
           cfg.consecutive,
           fired ? " [after trigger]" : "");
  else
    printf("[RELOC] UE %u %s = %.3f -> %d/%d%s\n",
           rrc_ue_id,
           cfg.metric,
           v,
           ues[s].streak,
           cfg.consecutive,
           fired ? " [after trigger]" : "");

  if (!fired && !pending && ues[s].streak >= cfg.consecutive) {
    pending = true;
    pending_ue = rrc_ue_id;
    pending_val = v;
  }
  pthread_mutex_unlock(&mtx);
}

static bool record_value(meas_record_lst_t const* r, double* out)
{
  if (r->value == INTEGER_MEAS_VALUE) {
    *out = r->int_val;
    return true;
  }
  if (r->value == REAL_MEAS_VALUE) {
    *out = r->real_val;
    return true;
  }
  return false; // no value: skip, do not count it either way
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
    uint32_t rrc_ue_id = 0;
    if (!rrc_ue_id_of(&frm_3->meas_report_per_ue[u].ue_meas_report_lst, &rrc_ue_id))
      continue;

    kpm_ind_msg_format_1_t const* m = &frm_3->meas_report_per_ue[u].ind_msg_format_1;
    if (m->meas_data_lst_len == 0)
      continue;
    /* Our action definitions give every measurement exactly one label, so
     * record k belongs to measurement k. Use the latest granularity period. */
    meas_data_lst_t const* data = &m->meas_data_lst[m->meas_data_lst_len - 1];

    /* Guard first, so the primary on the same report sees it. */
    for (int pass = 0; pass < 2; pass++) {
      const bool want_guard = pass == 0;
      const char* name = want_guard ? cfg.guard : cfg.metric;
      if (name == NULL)
        continue;
      for (size_t k = 0; k < m->meas_info_lst_len && k < data->meas_record_len; k++) {
        meas_type_t const* t = &m->meas_info_lst[k].meas_type;
        if (t->type != NAME_MEAS_TYPE || cmp_str_ba(name, t->name) != 0)
          continue;
        double v;
        if (record_value(&data->meas_record_lst[k], &v))
          on_value(rrc_ue_id, want_guard, v);
      }
    }
  }
}

/* ---- discovery ---------------------------------------------------------- */

static sm_ran_function_t const* find_rf(e2_node_connected_xapp_t const* n, int id)
{
  for (size_t i = 0; i < n->len_rf; i++)
    if (n->rf[i].id == id)
      return &n->rf[i];
  return NULL;
}

static ric_report_style_item_t const* style4(e2_node_connected_xapp_t const* n)
{
  sm_ran_function_t const* rf = find_rf(n, KPM_RAN_FUNC_ID);
  if (rf == NULL || rf->defn.type != KPM_RAN_FUNC_DEF_E)
    return NULL;
  for (size_t j = 0; j < rf->defn.kpm.sz_ric_report_style_list; j++)
    if (rf->defn.kpm.ric_report_style_list[j].report_style_type == STYLE_4_RIC_SERVICE_REPORT)
      return &rf->defn.kpm.ric_report_style_list[j];
  return NULL;
}

static bool offers(ric_report_style_item_t const* it, const char* name)
{
  if (it == NULL || name == NULL)
    return false;
  for (size_t k = 0; k < it->meas_info_for_action_lst_len; k++)
    if (cmp_str_ba(name, it->meas_info_for_action_lst[k].name) == 0)
      return true;
  return false;
}

static const char* node_name(ngran_node_t t)
{
  switch (t) {
    case ngran_gNB:
      return "gNB";
    case ngran_gNB_CU:
      return "gNB-CU";
    case ngran_gNB_CUCP:
      return "gNB-CU-CP";
    case ngran_gNB_CUUP:
      return "gNB-CU-UP";
    case ngran_gNB_DU:
      return "gNB-DU";
    default:
      return "other";
  }
}

static void print_offers(e2_node_connected_xapp_t const* n)
{
  const unsigned long cu_du = n->id.cu_du_id != NULL ? (unsigned long)*n->id.cu_du_id : 0;
  ric_report_style_item_t const* it = style4(n);
  printf("[RELOC] %s nb_id %u cu_du_id %lu: ", node_name(n->id.type), n->id.nb_id.nb_id, cu_du);
  if (it == NULL) {
    printf("no per-UE KPM (REPORT Style 4)\n");
    return;
  }
  for (size_t k = 0; k < it->meas_info_for_action_lst_len; k++) {
    byte_array_t const name = it->meas_info_for_action_lst[k].name;
    printf("%s%.*s", k ? ", " : "", (int)name.len, (char const*)name.buf);
  }
  printf("\n");
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

static void fill_meas(meas_info_format_1_lst_t* m, const char* name)
{
  m->meas_type.type = NAME_MEAS_TYPE;
  m->meas_type.name = cp_str_to_ba(name);
  m->label_info_lst_len = 1;
  m->label_info_lst = calloc(1, sizeof(label_info_lst_t));
  assert(m->label_info_lst != NULL && "Memory exhausted");
  m->label_info_lst[0].noLabel = calloc(1, sizeof(enum_value_e));
  assert(m->label_info_lst[0].noLabel != NULL && "Memory exhausted");
  *m->label_info_lst[0].noLabel = TRUE_ENUM_VALUE;
}

static kpm_sub_data_t gen_sub(snssai_t nssai, const char* const* names, size_t n_names)
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
  f1->meas_info_lst_len = n_names;
  f1->meas_info_lst = calloc(n_names, sizeof(meas_info_format_1_lst_t));
  assert(f1->meas_info_lst != NULL && "Memory exhausted");
  for (size_t i = 0; i < n_names; i++)
    fill_meas(&f1->meas_info_lst[i], names[i]);
  f1->gran_period_ms = cfg.period_ms;
  f1->cell_global_id = NULL;
  f1->meas_bin_range_info_lst_len = 0;
  f1->meas_bin_info_lst = NULL;

  return sub;
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
  cfg.metric = env_str("RELOC_METRIC", "DRB.PdcpSduDelayDl");
  cfg.lt = env_lt("RELOC_CMP", false);
  cfg.threshold = env_double("RELOC_THRESHOLD", 10);
  cfg.guard = env_str("RELOC_GUARD_METRIC", NULL);
  cfg.guard_lt = env_lt("RELOC_GUARD_CMP", true);
  cfg.guard_threshold = env_double("RELOC_GUARD_THRESHOLD", 0);
  cfg.consecutive = (int)env_double("RELOC_CONSECUTIVE", 3);
  cfg.period_ms = (uint64_t)env_double("RELOC_PERIOD_MS", 1000);
  cfg.only_ue = (uint32_t)env_double("RELOC_UE", 0);
  cfg.target_cuup = strtol(env_str("RELOC_TARGET_CUUP", "-1"), NULL, 0);
  cfg.linger_s = (int)env_double("RELOC_LINGER_S", 30);
  cfg.timeout_s = (int)env_double("RELOC_TIMEOUT_S", 600);
  cfg.list_only = env_double("RELOC_LIST", 0) != 0;
  if (cfg.consecutive < 1)
    cfg.consecutive = 1;
  if (cfg.guard != NULL && strcmp(cfg.guard, cfg.metric) == 0)
    cfg.guard = NULL;

  if (!cfg.list_only) {
    printf("[RELOC] trigger: %s %s %g on %d reports in a row (period %lu ms)\n",
           cfg.metric,
           cfg.lt ? "<" : ">",
           cfg.threshold,
           cfg.consecutive,
           (unsigned long)cfg.period_ms);
    if (cfg.guard != NULL)
      printf("[RELOC] guard: only while %s %s %g\n", cfg.guard, cfg.guard_lt ? "<" : ">", cfg.guard_threshold);
    if (cfg.only_ue)
      printf("[RELOC] watching only rrc_ue_id %u\n", cfg.only_ue);
    if (cfg.target_cuup >= 0)
      printf("[RELOC] target gNB-CU-UP ID %ld (0x%lx)\n", cfg.target_cuup, cfg.target_cuup);
    else
      printf("[RELOC] target: any gNB-CU-UP other than the UE's current one\n");
  }

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);

  fr_args_t args = init_fr_args(argc, argv);
  init_xapp_api(&args);
  sleep(1);

  e2_node_arr_xapp_t nodes = e2_nodes_xapp_api();
  printf("[RELOC] %d E2 node(s) connected\n", nodes.len);
  for (size_t i = 0; i < nodes.len; i++)
    print_offers(&nodes.n[i]);

  global_e2_node_id_t const* cucp = NULL;
  sm_ans_xapp_t hndl[MAX_HANDLES];
  size_t n_hndl = 0;
  bool metric_seen = false, guard_seen = false;

  if (!cfg.list_only) {
    for (size_t i = 0; i < nodes.len; i++) {
      e2_node_connected_xapp_t* n = &nodes.n[i];
      if ((n->id.type == ngran_gNB_CUCP || n->id.type == ngran_gNB_CU || n->id.type == ngran_gNB) && cucp == NULL
          && find_rf(n, RC_RAN_FUNC_ID) != NULL)
        cucp = &n->id;

      ric_report_style_item_t const* it = style4(n);
      const char* names[2];
      size_t n_names = 0;
      if (offers(it, cfg.metric)) {
        names[n_names++] = cfg.metric;
        metric_seen = true;
      }
      if (offers(it, cfg.guard)) {
        names[n_names++] = cfg.guard;
        guard_seen = true;
      }
      if (n_names == 0)
        continue;

      for (size_t k = 0; k < WATCH_NSSAI_LEN && n_hndl < MAX_HANDLES; k++) {
        kpm_sub_data_t sub = gen_sub(WATCH_NSSAI[k], names, n_names);
        hndl[n_hndl] = report_sm_xapp_api(&n->id, KPM_RAN_FUNC_ID, &sub, sm_cb_kpm);
        free_kpm_sub_data(&sub);
        if (hndl[n_hndl].success)
          n_hndl++;
        else
          printf("[RELOC] KPM subscription for S-NSSAI (%u, %u) on %s nb_id %u failed\n",
                 WATCH_NSSAI[k].sst,
                 WATCH_NSSAI[k].sd,
                 node_name(n->id.type),
                 n->id.nb_id.nb_id);
      }
    }

    if (!metric_seen)
      printf("[RELOC] no connected node offers %s per UE (see the list above): not implemented in this RAN build, "
             "or misspelt\n",
             cfg.metric);
    if (cfg.guard != NULL && !guard_seen)
      printf("[RELOC] no connected node offers the guard %s: the trigger can never fire\n", cfg.guard);
    if (cucp == NULL)
      printf("[RELOC] no gNB-CU-CP with E2SM-RC connected: can watch, cannot trigger\n");
  }

  const int64_t t0 = time_now_us();
  int64_t fired_at = 0;
  while (!stop && n_hndl > 0) {
    usleep(100 * 1000);
    const int64_t now = time_now_us();

    bool send = false;
    uint32_t ue = 0;
    double val = 0;
    pthread_mutex_lock(&mtx);
    if (pending && !fired) {
      send = true;
      ue = pending_ue;
      val = pending_val;
      fired = true;
      pending = false;
    }
    pthread_mutex_unlock(&mtx);

    if (send) {
      printf("[RELOC] UE %u: %s = %.3f, %s %g on %d reports in a row -> gNB-CU-UP Change\n",
             ue,
             cfg.metric,
             val,
             cfg.lt ? "below" : "above",
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
      printf("[RELOC] nothing crossed the line within %d s, giving up\n", cfg.timeout_s);
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
