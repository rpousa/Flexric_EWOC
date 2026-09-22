/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this
 * file except in compliance with the License. You may obtain a copy of the
 * License at:
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * Non-destructive slice xApp .
 *
 * Control logic per matched E2 node:
 *
 *   dst slice exists?
 *     YES → ASSOC only             (slice already there, just move the UE)
 *     NO  → ADD dst slice → ASSOC  (create it first, then move the UE)
 *
 * No existing slice is ever removed. This is safe to run repeatedly
 * against a live RAN without disturbing other UEs.
 *
 * On handovers:
 *   The ASSOC moves the UE's MAC scheduling to the new PRB range instantly
 *   (intra-gNB, sub-ms). It does NOT by itself trigger an inter-CUUP
 *   handover. For that, the CU-CP must observe the slice label change and
 *   initiate a Bearer Context Modification on E1AP. OAI does not wire this
 *   up automatically; a companion E2SM-RC xApp or a CU-CP patch is needed.
 *
 * Usage:
 *   ./xapp_slice_assoc --sst 1 --sd 0x000001 --rnti 0x1234 \
 *       --target-slice 5 [--algo STATIC]                    \
 *       [--pos-low 0] [--pos-high 12]                       \
 *       -- <flexric args>
 */

#include "../../../../src/xApp/e42_xapp_api.h"
#include "../../../../src/util/alg_ds/alg/defer.h"
#include "../../../../src/util/time_now_us.h"
#include "../../../../src/sm/slice_sm/slice_sm_id.h"
#include "../../../../src/sm/rc_sm/rc_sm_id.h"
#include "../../../../src/sm/rc_sm/ie/rc_data_ie.h"
#include "../../../../src/sm/rc_sm/rc_sm_id.h"
#include "../../../../src/sm/rc_sm/ie/ir/ran_param_struct.h"
#include "../../../../src/sm/rc_sm/ie/ir/ran_param_list.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>

/* ─────────────────────────────────────────────────────────────────────────────
 * CLI parameters
 * ───────────────────────────────────────────────────────────────────────────*/
typedef struct {
  /* NSSAI — node filter + slice label carrier */
  uint8_t  sst;             /* Slice/Service Type          (default: 1)        */
  uint32_t sd;              /* Slice Differentiator 24-bit (default: 0xFFFFFF) */

  /* UE */
  uint16_t rnti;            /* 0 = auto-learn from first indication             */

  /* Target slice (the only slice this xApp touches) */
  uint32_t dst_slice_id;

  /* Scheduler — only used if we need to CREATE the slice */
  slice_algorithm_e algo;

  /* STATIC */
  uint32_t pos_low;
  uint32_t pos_high;

  /* NVS */
  nvs_slice_conf_e nvs_conf;
  float            mbps_rsvd;
  float            mbps_ref;
  float            pct_rsvd;

  /* EDF */
  int deadline;
  int guaranteed_prbs;

  /* Indication period */
  const char* period;
} xapp_params_t;

/* ─────────────────────────────────────────────────────────────────────────────
 * Shared state between callback and main
 * ───────────────────────────────────────────────────────────────────────────*/
static _Atomic uint16_t g_observed_rnti  = 0;  /* RNTI from indication        */
static _Atomic bool     g_dst_exists     = false; /* dst slice already present */
static _Atomic bool     g_indication_rdy = false; /* first indication done     */

/* ─────────────────────────────────────────────────────────────────────────────
 * Helpers
 * ───────────────────────────────────────────────────────────────────────────*/

/* NSSAI-tagged label: "nssai_<sst>_<sd_hex>" or "nssai_<sst>_none" */
static void make_slice_label(const xapp_params_t* p, char* buf, size_t n)
{
  if (p->sd == 0xFFFFFF)
    snprintf(buf, n, "nssai_%u_none", p->sst);
  else
    snprintf(buf, n, "nssai_%u_%06X", p->sst, p->sd);
}

/*
 * Node NSSAI filter.
 * Compile with -DE2_NODE_HAS_NSSAI when your FlexRIC build exposes n->nssai[].
 * Without that flag every node is accepted (safe fallback for older builds).
 */
static bool node_matches_nssai(const e2_node_connected_xapp_t* n,
                               uint8_t sst, uint32_t sd)
{
#ifdef E2_NODE_HAS_NSSAI
  for (size_t i = 0; i < n->len_nssai; ++i) {
    if (n->nssai[i].sst != sst) continue;
    if (sd == 0xFFFFFF || n->nssai[i].sd == sd) return true;
  }
  return false;
#else
  (void)n; (void)sst; (void)sd;
  return true;
#endif
}

/* Scan DL slice list for a given ID */
static bool slice_id_exists_in_dl(const slice_ind_msg_t* msg, uint32_t id)
{
  const ul_dl_slice_conf_t* dl = &msg->slice_conf.dl;
  for (uint32_t i = 0; i < dl->len_slices; ++i)
    if (dl->slices[i].id == id) return true;
  return false;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Indication callback
 * ───────────────────────────────────────────────────────────────────────────*/
typedef struct { const xapp_params_t* p; } cb_ctx_t;

static void sm_cb_slice(sm_ag_if_rd_t const* rd, void* ctx_v)
{
  assert(rd != NULL);
  assert(rd->type     == INDICATION_MSG_AGENT_IF_ANS_V0);
  assert(rd->ind.type == SLICE_STATS_V0);

  const xapp_params_t*   p   = ((cb_ctx_t*)ctx_v)->p;
  const slice_ind_msg_t* msg = &rd->ind.slice.msg;

  printf("[IND] latency=%ld μs  dl_slices=%u  ues=%u\n",
         time_now_us() - msg->tstamp,
         msg->slice_conf.dl.len_slices,
         msg->ue_slice_conf.len_ue_slice);

  /* First indication only: snapshot existing slices and check for dst */
  if (!atomic_load(&g_indication_rdy)) {
    bool exists = slice_id_exists_in_dl(msg, p->dst_slice_id);
    atomic_store(&g_dst_exists, exists);

    printf("[IND] Active DL slices:\n");
    for (uint32_t i = 0; i < msg->slice_conf.dl.len_slices; ++i) {
      const fr_slice_t* s = &msg->slice_conf.dl.slices[i];
      printf("[IND]   id=%-3u  label=%.*s%s\n",
             s->id,
             (int)s->len_label, s->label ? s->label : "",
             s->id == p->dst_slice_id ? "  ← target (exists)" : "");
    }
    if (!exists)
      printf("[IND]   (target slice id=%u not present — will be created)\n",
             p->dst_slice_id);
  }

  /* Capture RNTI — honours explicit --rnti filter, uses CAS to store once */
  for (uint32_t i = 0; i < msg->ue_slice_conf.len_ue_slice; ++i) {
    uint16_t rnti = msg->ue_slice_conf.ues[i].rnti;
    if (p->rnti != 0 && rnti != p->rnti) continue;
    uint16_t expected = 0;
    if (atomic_compare_exchange_strong(&g_observed_rnti, &expected, rnti))
      printf("[IND] Captured RNTI 0x%04X\n", rnti);
    break;
  }

  atomic_store(&g_indication_rdy, true);
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Slice control builders
 * ───────────────────────────────────────────────────────────────────────────*/

/*
 * ADD a single DL slice with the NSSAI label.
 * Called only when the slice does not already exist.
 */
static void fill_add_slice(slice_conf_t* add, const xapp_params_t* p)
{
  assert(add && p);

  /* DL scheduler header */
  ul_dl_slice_conf_t* dl  = &add->dl;
  const char*         pf  = "PF";
  dl->len_sched_name = strlen(pf);
  dl->sched_name     = malloc(dl->len_sched_name);
  assert(dl->sched_name);
  memcpy(dl->sched_name, pf, dl->len_sched_name);

  if (p->algo == SLICE_ALG_SM_V0_NONE) {
    /* NONE means "reset" — unusual here but kept for completeness */
    dl->len_slices = 0;
    printf("[CTRL] RESET DL slices (algo=NONE)\n");
  } else {
    dl->len_slices = 1;
    dl->slices     = calloc(1, sizeof(fr_slice_t));
    assert(dl->slices);

    fr_slice_t* s = &dl->slices[0];
    s->id = p->dst_slice_id;

    /* NSSAI label — the signal to CU-CP for CUUP selection on E1 */
    char label[32];
    make_slice_label(p, label, sizeof(label));
    s->len_label = strlen(label);
    s->label     = malloc(s->len_label);
    assert(s->label);
    memcpy(s->label, label, s->len_label);

    const char* sched = "PF";
    s->len_sched = strlen(sched);
    s->sched     = malloc(s->len_sched);
    assert(s->sched);
    memcpy(s->sched, sched, s->len_sched);

    s->params.type = p->algo;

    switch (p->algo) {
      case SLICE_ALG_SM_V0_STATIC:
        s->params.u.sta.pos_low  = p->pos_low;
        s->params.u.sta.pos_high = p->pos_high;
        printf("[CTRL] ADD STATIC DL slice id=%u label=%s pos=[%u,%u]\n",
               s->id, label, p->pos_low, p->pos_high);
        break;

      case SLICE_ALG_SM_V0_NVS:
        s->params.u.nvs.conf = p->nvs_conf;
        if (p->nvs_conf == SLICE_SM_NVS_V0_RATE) {
          s->params.u.nvs.u.rate.u1.mbps_required  = p->mbps_rsvd;
          s->params.u.nvs.u.rate.u2.mbps_reference = p->mbps_ref;
          printf("[CTRL] ADD NVS(RATE) DL slice id=%u label=%s "
                 "mbps_req=%.2f mbps_ref=%.2f\n",
                 s->id, label, p->mbps_rsvd, p->mbps_ref);
        } else {
          s->params.u.nvs.u.capacity.u.pct_reserved = p->pct_rsvd;
          printf("[CTRL] ADD NVS(CAP) DL slice id=%u label=%s pct=%.2f\n",
                 s->id, label, p->pct_rsvd);
        }
        break;

      case SLICE_ALG_SM_V0_EDF:
        s->params.u.edf.deadline        = p->deadline;
        s->params.u.edf.guaranteed_prbs = p->guaranteed_prbs;
        printf("[CTRL] ADD EDF DL slice id=%u label=%s deadline=%d prbs=%d\n",
               s->id, label, p->deadline, p->guaranteed_prbs);
        break;

      default:
        assert(0 && "Unhandled algorithm");
    }
  }

  /* UL — round-robin, no explicit slices */
  ul_dl_slice_conf_t* ul  = &add->ul;
  const char*         rr  = "round_robin_ul";
  ul->len_sched_name = strlen(rr);
  ul->sched_name     = malloc(ul->len_sched_name);
  assert(ul->sched_name);
  memcpy(ul->sched_name, rr, ul->len_sched_name);
  ul->len_slices = 0;
}

/* Associate a UE to the target slice */
static void fill_assoc(ue_slice_conf_t* assoc, const xapp_params_t* p)
{
  assert(assoc && p);

  uint16_t rnti = (p->rnti != 0) ? p->rnti
                                  : (uint16_t)atomic_load(&g_observed_rnti);
  if (rnti == 0)
    fprintf(stderr, "[WARN] RNTI=0 — no UE observed yet; ASSOC will be a no-op\n");

  assoc->len_ue_slice = 1;
  assoc->ues          = calloc(1, sizeof(ue_slice_assoc_t));
  assert(assoc->ues);
  assoc->ues[0].rnti  = rnti;
  assoc->ues[0].dl_id = p->dst_slice_id;

  printf("[CTRL] ASSOC rnti=0x%04X → DL slice id=%u\n", rnti, p->dst_slice_id);
  printf("[NOTE] This moves MAC scheduling immediately (intra-gNB, sub-ms).\n");
  printf("[NOTE] Inter-CUUP handover requires CU-CP to act on the slice label\n");
  printf("[NOTE] via E1AP Bearer Context Modification (not automatic in OAI).\n");
}

static slice_ctrl_req_data_t build_req(slice_ctrl_msg_e type,
                                       const xapp_params_t* p)
{
  slice_ctrl_req_data_t req = {0};
  switch (type) {
    case SLICE_CTRL_SM_V0_ADD:
      req.msg.type = SLICE_CTRL_SM_V0_ADD;
      fill_add_slice(&req.msg.u.add_mod_slice, p);
      break;
    case SLICE_CTRL_SM_V0_UE_SLICE_ASSOC:
      req.msg.type = SLICE_CTRL_SM_V0_UE_SLICE_ASSOC;
      fill_assoc(&req.msg.u.ue_slice, p);
      break;
    default:
      assert(0 && "DEL not used in this xApp");
  }
  return req;
}

// added for RC control 
/* ── RC control builder: E2SM-RC Control Style 2, S-NSSAI RAN params ─────── */
static rc_ctrl_req_data_t build_rc_cuup_steer(uint16_t rnti,
                                              uint8_t  sst,
                                              uint32_t sd)
{
  rc_ctrl_req_data_t req = {0};

  req.hdr.ric_style_type      = 2;            /* Radio Bearer Control        */
  req.hdr.ctrl_act_id         = 1;            /* DRB setup/modification      */
  req.hdr.ue_id.type          = GNB_UE_ID;
  req.hdr.ue_id.gnb.ran_ue_id = rnti;

  req.msg.style     = 2;
  req.msg.action_id = 1;

  /* One RAN parameter: S-NSSAI (id 21) as a STRUCTURE of SST(22)/SD(23) */
  req.msg.len_ran_param = 1;
  req.msg.ran_param     = calloc(1, sizeof(seq_ran_param_t));
  assert(req.msg.ran_param);

  seq_ran_param_t* p    = &req.msg.ran_param[0];
  p->ran_param_id       = 21;
  p->ran_param_val.type = STRUCTURE_RAN_PARAMETER_VAL_TYPE;

  p->ran_param_val.strct.len       = (sd == 0xFFFFFF) ? 1 : 2;
  p->ran_param_val.strct.ran_param = calloc(p->ran_param_val.strct.len,
                                            sizeof(seq_ran_param_t));
  assert(p->ran_param_val.strct.ran_param);

  /* SST (22) */
  seq_ran_param_t* sst_p = &p->ran_param_val.strct.ran_param[0];
  sst_p->ran_param_id                        = 22;
  sst_p->ran_param_val.type                  = ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE;
  sst_p->ran_param_val.flag_false.type       = OCTET_STRING_RAN_PARAMETER_VAL;
  sst_p->ran_param_val.flag_false.octet.len  = 1;
  sst_p->ran_param_val.flag_false.octet.buf  = malloc(1);
  sst_p->ran_param_val.flag_false.octet.buf[0] = sst;

  /* SD (23) — only when configured */
  if (sd != 0xFFFFFF) {
    seq_ran_param_t* sd_p = &p->ran_param_val.strct.ran_param[1];
    sd_p->ran_param_id                        = 23;
    sd_p->ran_param_val.type                  = ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE;
    sd_p->ran_param_val.flag_false.type       = OCTET_STRING_RAN_PARAMETER_VAL;
    sd_p->ran_param_val.flag_false.octet.len  = 3;
    sd_p->ran_param_val.flag_false.octet.buf  = malloc(3);
    sd_p->ran_param_val.flag_false.octet.buf[0] = (sd >> 16) & 0xFF;
    sd_p->ran_param_val.flag_false.octet.buf[1] = (sd >>  8) & 0xFF;
    sd_p->ran_param_val.flag_false.octet.buf[2] =  sd        & 0xFF;
  }
  return req;
}

static void trigger_cuup_steer(e2_node_connected_xapp_t* node,
                               const xapp_params_t* p, uint16_t rnti)
{
  rc_ctrl_req_data_t rc = build_rc_cuup_steer(rnti, p->sst, p->sd);
  printf("[RC] E2SM-RC Style2 → CUUP steer rnti=0x%04X sst=%u sd=0x%06X\n",
         rnti, p->sst, p->sd);
  control_sm_xapp_api(&node->id, SM_RC_ID, &rc);
  free_rc_ctrl_req_data(&rc);   /* verify the exact name in your FlexRIC build */
}

// end of RC control

/* ─────────────────────────────────────────────────────────────────────────────
 * Argument parsing
 * ───────────────────────────────────────────────────────────────────────────*/
static void usage(const char* prog)
{
  printf(
    "Usage: %s [options] [-- flexric-options]\n"
    "\n"
    "NSSAI (node filter + slice label):\n"
    "  --sst <uint8>        Slice/Service Type          (default: 1)\n"
    "  --sd  <uint24>       Slice Differentiator hex    (default: 0xFFFFFF = any)\n"
    "\n"
    "UE:\n"
    "  --rnti <uint16>      RNTI to move (0 = auto-learn, default: 0)\n"
    "\n"
    "Slice:\n"
    "  --target-slice <id>  DL slice ID to assoc the UE to (default: 5)\n"
    "                       If it doesn't exist it will be created.\n"
    "                       No existing slice is ever deleted.\n"
    "\n"
    "Scheduler (used only if target slice is created):\n"
    "  --algo <name>        STATIC|NVS|EDF|NONE  (default: STATIC)\n"
    "  --pos-low  <uint>    First PRB             (default: 0)\n"
    "  --pos-high <uint>    Last  PRB             (default: 12)\n"
    "  --nvs-conf <name>    RATE|CAPACITY         (default: RATE)\n"
    "  --mbps-rsvd <float>                        (default: 0.2)\n"
    "  --mbps-ref  <float>                        (default: 10.0)\n"
    "  --pct-rsvd  <float>                        (default: 0.7)\n"
    "  --deadline  <ms>                           (default: 20)\n"
    "  --guaranteed-prbs <n>                      (default: 10)\n"
    "\n"
    "  --period <str>       Indication period     (default: \"5_ms\")\n"
    "  --help\n"
    "\n"
    "Handover note:\n"
    "  ASSOC moves MAC scheduling instantly within the gNB.\n"
    "  Inter-CUUP handover requires the CU-CP to read the slice label\n"
    "  (nssai_<sst>_<sd>) and trigger E1AP Bearer Context Modification.\n"
    "  This is not automatic in OAI; a companion E2SM-RC xApp is needed.\n"
    "\n", prog);
}

static slice_algorithm_e parse_algo(const char* s)
{
  if (strcasecmp(s, "STATIC") == 0) return SLICE_ALG_SM_V0_STATIC;
  if (strcasecmp(s, "NVS")    == 0) return SLICE_ALG_SM_V0_NVS;
  if (strcasecmp(s, "EDF")    == 0) return SLICE_ALG_SM_V0_EDF;
  if (strcasecmp(s, "NONE")   == 0) return SLICE_ALG_SM_V0_NONE;
  fprintf(stderr, "Unknown algo '%s'\n", s); exit(EXIT_FAILURE);
}

static nvs_slice_conf_e parse_nvs_conf(const char* s)
{
  if (strcasecmp(s, "RATE")     == 0) return SLICE_SM_NVS_V0_RATE;
  if (strcasecmp(s, "CAPACITY") == 0) return SLICE_SM_NVS_V0_CAPACITY;
  fprintf(stderr, "Unknown NVS conf '%s'\n", s); exit(EXIT_FAILURE);
}

static int parse_args(int argc, char* argv[], xapp_params_t* p)
{
  p->sst             = 1;
  p->sd              = 0xFFFFFF;
  p->rnti            = 0;
  p->dst_slice_id    = 5;
  p->algo            = SLICE_ALG_SM_V0_STATIC;
  p->pos_low         = 0;
  p->pos_high        = 12;
  p->nvs_conf        = SLICE_SM_NVS_V0_RATE;
  p->mbps_rsvd       = 0.2f;
  p->mbps_ref        = 10.0f;
  p->pct_rsvd        = 0.7f;
  p->deadline        = 20;
  p->guaranteed_prbs = 10;
  p->period          = "5_ms";

  static const struct option opts[] = {
    {"sst",             required_argument, 0, 's'},
    {"sd",              required_argument, 0, 'd'},
    {"rnti",            required_argument, 0, 'r'},
    {"target-slice",    required_argument, 0, 't'},
    {"algo",            required_argument, 0, 'a'},
    {"pos-low",         required_argument, 0, 'l'},
    {"pos-high",        required_argument, 0, 'h'},
    {"nvs-conf",        required_argument, 0, 'n'},
    {"mbps-rsvd",       required_argument, 0, 'm'},
    {"mbps-ref",        required_argument, 0, 'M'},
    {"pct-rsvd",        required_argument, 0, 'p'},
    {"deadline",        required_argument, 0, 'D'},
    {"guaranteed-prbs", required_argument, 0, 'g'},
    {"period",          required_argument, 0, 'P'},
    {"help",            no_argument,       0, '?'},
    {0, 0, 0, 0}
  };

  int flexric_start = argc;
  for (int i = 1; i < argc; ++i)
    if (strcmp(argv[i], "--") == 0) { flexric_start = i + 1; argc = i; break; }

  int opt, idx = 0;
  while ((opt = getopt_long(argc, argv, "", opts, &idx)) != -1) {
    switch (opt) {
      case 's': p->sst             = (uint8_t) strtoul(optarg, NULL, 0); break;
      case 'd': p->sd              = (uint32_t)strtoul(optarg, NULL, 0); break;
      case 'r': p->rnti            = (uint16_t)strtoul(optarg, NULL, 0); break;
      case 't': p->dst_slice_id    = (uint32_t)strtoul(optarg, NULL, 0); break;
      case 'a': p->algo            = parse_algo(optarg);                  break;
      case 'l': p->pos_low         = (uint32_t)strtoul(optarg, NULL, 0); break;
      case 'h': p->pos_high        = (uint32_t)strtoul(optarg, NULL, 0); break;
      case 'n': p->nvs_conf        = parse_nvs_conf(optarg);             break;
      case 'm': p->mbps_rsvd       = strtof(optarg, NULL);               break;
      case 'M': p->mbps_ref        = strtof(optarg, NULL);               break;
      case 'p': p->pct_rsvd        = strtof(optarg, NULL);               break;
      case 'D': p->deadline        = (int)strtol(optarg, NULL, 0);       break;
      case 'g': p->guaranteed_prbs = (int)strtol(optarg, NULL, 0);       break;
      case 'P': p->period          = optarg;                              break;
      case '?': usage(argv[0]); exit(EXIT_SUCCESS);
      default:  usage(argv[0]); exit(EXIT_FAILURE);
    }
  }

  if (p->algo != SLICE_ALG_SM_V0_NONE && p->pos_low >= p->pos_high) {
    fprintf(stderr, "Error: --pos-low (%u) must be < --pos-high (%u)\n",
            p->pos_low, p->pos_high);
    exit(EXIT_FAILURE);
  }

  char label[32];
  make_slice_label(p, label, sizeof(label));
  printf("[PARAMS] sst=%u sd=0x%06X label=%s rnti=0x%04X dst_slice=%u "
         "algo=%d period=%s pos=[%u,%u]\n",
         p->sst, p->sd, label, p->rnti, p->dst_slice_id,
         p->algo, p->period, p->pos_low, p->pos_high);

  return flexric_start;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * main
 * ───────────────────────────────────────────────────────────────────────────*/
int main(int argc, char* argv[])
{
  xapp_params_t params = {0};
  int flexric_start = parse_args(argc, argv, &params);

  /* Rebuild argv for FlexRIC */
  int    fr_argc = 1 + (argc - flexric_start);
  char** fr_argv = calloc(fr_argc, sizeof(char*));
  assert(fr_argv);
  fr_argv[0] = argv[0];
  for (int i = flexric_start; i < argc; ++i)
    fr_argv[1 + i - flexric_start] = argv[i];

  fr_args_t args = init_fr_args(fr_argc, fr_argv);
  free(fr_argv);

  init_xapp_api(&args);
  sleep(1);

  e2_node_arr_xapp_t nodes = e2_nodes_xapp_api();
  defer({ free_e2_node_arr_xapp(&nodes); });
  assert(nodes.len > 0);
  printf("[MAIN] Connected E2 nodes: %d\n", nodes.len);

  sm_ans_xapp_t* handles = calloc(nodes.len, sizeof(sm_ans_xapp_t));
  assert(handles);
  cb_ctx_t cb_ctx = { .p = &params };
  int controlled = 0;

  for (size_t i = 0; i < nodes.len; ++i) {
    e2_node_connected_xapp_t* n = &nodes.n[i];

    /* NSSAI node filter */
    if (!node_matches_nssai(n, params.sst, params.sd)) {
      printf("[MAIN] Node %zu skipped (sst=%u sd=0x%06X not matched)\n",
             i, params.sst, params.sd);
      handles[i].success = false;
      continue;
    }
    printf("[MAIN] Node %zu matched (sst=%u sd=0x%06X)\n",
           i, params.sst, params.sd);

    /* Reset per-node state */
    atomic_store(&g_observed_rnti,  0);
    atomic_store(&g_dst_exists,     false);
    atomic_store(&g_indication_rdy, false);

    /* Subscribe */
    handles[i] = report_sm_xapp_api(&n->id, SM_SLICE_ID,
                                    (void*)params.period, sm_cb_slice);
    assert(handles[i].success == true);

    /* Wait for first indication (non-blocking poll, 10 ms steps) */
    printf("[MAIN] Waiting for first SLICE indication from node %zu...\n", i);
    while (!atomic_load(&g_indication_rdy))
      usleep(10000);

    bool dst_exists = atomic_load(&g_dst_exists);

    if (dst_exists) {
      /* ── Slice exists: ASSOC only ──────────────────────────────────────── */
      printf("[MAIN] Slice id=%u already exists → ASSOC only (no ADD)\n",
             params.dst_slice_id);
    } else {
      /* ── Slice missing: ADD then ASSOC ────────────────────────────────── */
      printf("[MAIN] Slice id=%u not found → ADD then ASSOC\n",
             params.dst_slice_id);
      slice_ctrl_req_data_t req_add = build_req(SLICE_CTRL_SM_V0_ADD, &params);
      control_sm_xapp_api(&n->id, SM_SLICE_ID, &req_add);
      free_slice_ctrl_msg(&req_add.msg);
      sleep(1);   /* allow the RAN to instantiate the slice before ASSOC */
    }

    /* ── Always: ASSOC ────────────────────────────────────────────────────── */
    slice_ctrl_req_data_t req_assoc = build_req(SLICE_CTRL_SM_V0_UE_SLICE_ASSOC,
                                                &params);
    control_sm_xapp_api(&n->id, SM_SLICE_ID, &req_assoc);
    free_slice_ctrl_msg(&req_assoc.msg);
    sleep(1);

    controlled++;
  }

  if (controlled == 0)
    printf("[WARN] No nodes matched NSSAI sst=%u sd=0x%06X\n",
           params.sst, params.sd);
  else
    printf("[MAIN] Done — controlled %d node(s)\n", controlled);

  for (int i = 0; i < nodes.len; ++i)
    if (handles[i].success)
      rm_report_sm_xapp_api(handles[i].u.handle);

  free(handles);
  sleep(1);

  while (try_stop_xapp_api() == false)
    usleep(1000);

  printf("[MAIN] xApp finished successfully\n");
  return EXIT_SUCCESS;
}