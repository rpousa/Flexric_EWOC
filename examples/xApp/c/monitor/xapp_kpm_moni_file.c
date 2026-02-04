/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BAS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

#include "../../../../src/xApp/e42_xapp_api.h"
#include "../../../../src/util/alg_ds/alg/defer.h"
#include "../../../../src/util/time_now_us.h"
#include "../../../../src/util/alg_ds/ds/lock_guard/lock_guard.h"
#include "../../../../src/util/e.h"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>


FILE *kpm_moni_file;

static
uint64_t const period_ms = 1000;

static
pthread_mutex_t mtx;

// Structure to hold measurement values for CSV row
typedef struct {
  uint32_t gnb_cu_cp_ue_e1ap;
  uint32_t gnb_cu_ue_f1ap;
  uint64_t ran_ue_id;
  int32_t pdcp_vol_dl;
  int32_t pdcp_vol_ul;
  float rlc_delay_dl;
  float ue_thp_dl;
  float ue_thp_ul;
  int32_t prb_tot_dl;
  int32_t prb_tot_ul;
  bool has_data;
} kpm_meas_row_t;

static
void get_timestamp_iso8601(char* buf, size_t buf_size)
{
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  
  struct tm tm_info;
  localtime_r(&ts.tv_sec, &tm_info);
  
  // Format: 2025-11-12T08:51:18.536359
  size_t len = strftime(buf, buf_size, "%Y-%m-%dT%H:%M:%S", &tm_info);
  snprintf(buf + len, buf_size - len, ".%06ld", ts.tv_nsec / 1000);
}

static
void extract_ue_ids(ue_id_e2sm_t ue_id, kpm_meas_row_t* row)
{
  switch (ue_id.type) {
    case GNB_UE_ID_E2SM:
      if (ue_id.gnb.gnb_cu_ue_f1ap_lst != NULL && ue_id.gnb.gnb_cu_ue_f1ap_lst_len > 0) {
        row->gnb_cu_ue_f1ap = ue_id.gnb.gnb_cu_ue_f1ap_lst[0];
      }
      if (ue_id.gnb.ran_ue_id != NULL) {
        row->ran_ue_id = *ue_id.gnb.ran_ue_id;
      }
      break;
      
    case GNB_DU_UE_ID_E2SM:
      row->gnb_cu_ue_f1ap = ue_id.gnb_du.gnb_cu_ue_f1ap;
      if (ue_id.gnb_du.ran_ue_id != NULL) {
        row->ran_ue_id = *ue_id.gnb_du.ran_ue_id;
      }
      break;
      
    case GNB_CU_UP_UE_ID_E2SM:
      row->gnb_cu_cp_ue_e1ap = ue_id.gnb_cu_up.gnb_cu_cp_ue_e1ap;
      if (ue_id.gnb_cu_up.ran_ue_id != NULL) {
        row->ran_ue_id = *ue_id.gnb_cu_up.ran_ue_id;
      }
      break;
      
    default:
      break;
  }
}

static
void extract_measurement_value(byte_array_t name, meas_record_lst_t meas_record, kpm_meas_row_t* row)
{
  if (cmp_str_ba("DRB.PdcpSduVolumeDL", name) == 0) {
    row->pdcp_vol_dl = meas_record.int_val;
  } else if (cmp_str_ba("DRB.PdcpSduVolumeUL", name) == 0) {
    row->pdcp_vol_ul = meas_record.int_val;
  } else if (cmp_str_ba("DRB.RlcSduDelayDl", name) == 0) {
    row->rlc_delay_dl = meas_record.real_val;
  } else if (cmp_str_ba("DRB.UEThpDl", name) == 0) {
    row->ue_thp_dl = meas_record.real_val;
  } else if (cmp_str_ba("DRB.UEThpUl", name) == 0) {
    row->ue_thp_ul = meas_record.real_val;
  } else if (cmp_str_ba("RRU.PrbTotDl", name) == 0) {
    row->prb_tot_dl = meas_record.int_val;
  } else if (cmp_str_ba("RRU.PrbTotUl", name) == 0) {
    row->prb_tot_ul = meas_record.int_val;
  }
}

static
void extract_kpm_measurements(kpm_ind_msg_format_1_t const* msg_frm_1, kpm_meas_row_t* row)
{
  assert(msg_frm_1->meas_info_lst_len > 0 && "Cannot correctly extract measurements");

  // UE Measurements per granularity period
  for (size_t j = 0; j < msg_frm_1->meas_data_lst_len; j++) {
    meas_data_lst_t const data_item = msg_frm_1->meas_data_lst[j];

    for (size_t z = 0; z < data_item.meas_record_len; z++) {
      meas_type_t const meas_type = msg_frm_1->meas_info_lst[z].meas_type;
      meas_record_lst_t const record_item = data_item.meas_record_lst[z];

      if (meas_type.type == NAME_MEAS_TYPE) {
        extract_measurement_value(meas_type.name, record_item, row);
      }
    }
    row->has_data = true;
  }
}

static
void write_csv_row(kpm_meas_row_t* row)
{
  if (!kpm_moni_file || !row->has_data) {
    return;
  }

  char timestamp[64];
  get_timestamp_iso8601(timestamp, sizeof(timestamp));

  fprintf(kpm_moni_file, "%s,%u,%u,%lu,%d,%d,%.2f,%.2f,%.2f,%d,%d\n",
          timestamp,
          row->gnb_cu_cp_ue_e1ap,
          row->gnb_cu_ue_f1ap,
          row->ran_ue_id,
          row->pdcp_vol_dl,
          row->pdcp_vol_ul,
          row->rlc_delay_dl,
          row->ue_thp_dl,
          row->ue_thp_ul,
          row->prb_tot_dl,
          row->prb_tot_ul);
  
  fflush(kpm_moni_file);
}

static
void sm_cb_kpm(sm_ag_if_rd_t const* rd)
{
  assert(rd != NULL);
  assert(rd->type == INDICATION_MSG_AGENT_IF_ANS_V0);
  assert(rd->ind.type == KPM_STATS_V3_0);

  // Reading Indication Message Format 3
  kpm_ind_data_t const* ind = &rd->ind.kpm.ind;
  kpm_ric_ind_hdr_format_1_t const* hdr_frm_1 = &ind->hdr.kpm_ric_ind_hdr_format_1;
  kpm_ind_msg_format_3_t const* msg_frm_3 = &ind->msg.frm_3;

  int64_t const now = time_now_us();
  static int counter = 1;
  
  {
    lock_guard(&mtx);

    printf("\n%7d KPM ind_msg latency = %ld [μs]\n", counter, now - hdr_frm_1->collectStartTime);

    // Reported list of measurements per UE
    for (size_t i = 0; i < msg_frm_3->ue_meas_report_lst_len; i++) {
      // Initialize measurement row
      kpm_meas_row_t row = {
        .gnb_cu_cp_ue_e1ap = 0,
        .gnb_cu_ue_f1ap = 0,
        .ran_ue_id = 0,
        .pdcp_vol_dl = 0,
        .pdcp_vol_ul = 0,
        .rlc_delay_dl = 0.0f,
        .ue_thp_dl = 0.0f,
        .ue_thp_ul = 0.0f,
        .prb_tot_dl = 0,
        .prb_tot_ul = 0,
        .has_data = false
      };

      // Extract UE ID
      ue_id_e2sm_t const ue_id_e2sm = msg_frm_3->meas_report_per_ue[i].ue_meas_report_lst;
      extract_ue_ids(ue_id_e2sm, &row);

      // Extract measurements
      extract_kpm_measurements(&msg_frm_3->meas_report_per_ue[i].ind_msg_format_1, &row);

      // Write CSV row
      write_csv_row(&row);
      
      // Also print to console for debugging
      printf("UE ID: e1ap=%u, f1ap=%u, ran_ue_id=%lu\n", 
             row.gnb_cu_cp_ue_e1ap, row.gnb_cu_ue_f1ap, row.ran_ue_id);
      printf("  PDCP Vol DL/UL: %d/%d kb\n", row.pdcp_vol_dl, row.pdcp_vol_ul);
      printf("  RLC Delay DL: %.2f μs\n", row.rlc_delay_dl);
      printf("  UE Thp DL/UL: %.2f/%.2f kbps\n", row.ue_thp_dl, row.ue_thp_ul);
      printf("  PRB Tot DL/UL: %d/%d PRBs\n", row.prb_tot_dl, row.prb_tot_ul);
    }
    counter++;
  }
}

static
test_info_lst_t filter_predicate(test_cond_type_e type, test_cond_e cond, int value)
{
  test_info_lst_t dst = {0};

  dst.test_cond_type = type;
  dst.S_NSSAI = TRUE_TEST_COND_TYPE;

  dst.test_cond = calloc(1, sizeof(test_cond_e));
  assert(dst.test_cond != NULL && "Memory exhausted");
  *dst.test_cond = EQUAL_TEST_COND;

  dst.test_cond_value = calloc(1, sizeof(test_cond_value_t));
  assert(dst.test_cond_value != NULL && "Memory exhausted");
  dst.test_cond_value->type = OCTET_STRING_TEST_COND_VALUE;

  dst.test_cond_value->octet_string_value = calloc(1, sizeof(byte_array_t));
  assert(dst.test_cond_value->octet_string_value != NULL && "Memory exhausted");
  const size_t len_nssai = 1;
  dst.test_cond_value->octet_string_value->len = len_nssai;
  dst.test_cond_value->octet_string_value->buf = calloc(len_nssai, sizeof(uint8_t));
  assert(dst.test_cond_value->octet_string_value->buf != NULL && "Memory exhausted");
  dst.test_cond_value->octet_string_value->buf[0] = value;

  return dst;
}

static
label_info_lst_t fill_kpm_label(void)
{
  label_info_lst_t label_item = {0};

  label_item.noLabel = ecalloc(1, sizeof(enum_value_e));
  *label_item.noLabel = TRUE_ENUM_VALUE;

  return label_item;
}

static
kpm_act_def_format_1_t fill_act_def_frm_1(ric_report_style_item_t const* report_item)
{
  assert(report_item != NULL);

  kpm_act_def_format_1_t ad_frm_1 = {0};

  size_t const sz = report_item->meas_info_for_action_lst_len;

  ad_frm_1.meas_info_lst_len = sz;
  ad_frm_1.meas_info_lst = calloc(sz, sizeof(meas_info_format_1_lst_t));
  assert(ad_frm_1.meas_info_lst != NULL && "Memory exhausted");

  for (size_t i = 0; i < sz; i++) {
    meas_info_format_1_lst_t* meas_item = &ad_frm_1.meas_info_lst[i];
    meas_item->meas_type.type = NAME_MEAS_TYPE;
    meas_item->meas_type.name = copy_byte_array(report_item->meas_info_for_action_lst[i].name);

    meas_item->label_info_lst_len = 1;
    meas_item->label_info_lst = ecalloc(1, sizeof(label_info_lst_t));
    meas_item->label_info_lst[0] = fill_kpm_label();
  }

  ad_frm_1.gran_period_ms = period_ms;
  ad_frm_1.cell_global_id = NULL;

#if defined KPM_V2_03 || defined KPM_V3_00
  ad_frm_1.meas_bin_range_info_lst_len = 0;
  ad_frm_1.meas_bin_info_lst = NULL;
#endif

  return ad_frm_1;
}

static
kpm_act_def_t fill_report_style_4(ric_report_style_item_t const* report_item)
{
  assert(report_item != NULL);
  assert(report_item->act_def_format_type == FORMAT_4_ACTION_DEFINITION);

  kpm_act_def_t act_def = {.type = FORMAT_4_ACTION_DEFINITION};

  act_def.frm_4.matching_cond_lst_len = 1;
  act_def.frm_4.matching_cond_lst = calloc(act_def.frm_4.matching_cond_lst_len, sizeof(matching_condition_format_4_lst_t));
  assert(act_def.frm_4.matching_cond_lst != NULL && "Memory exhausted");
  
  test_cond_type_e const type = S_NSSAI_TEST_COND_TYPE;
  test_cond_e const condition = GREATERTHAN_TEST_COND;
  int const value = 1;
  act_def.frm_4.matching_cond_lst[0].test_info_lst = filter_predicate(type, condition, value);

  act_def.frm_4.action_def_format_1 = fill_act_def_frm_1(report_item);

  return act_def;
}

typedef kpm_act_def_t (*fill_kpm_act_def)(ric_report_style_item_t const* report_item);

static
fill_kpm_act_def get_kpm_act_def[END_RIC_SERVICE_REPORT] = {
    NULL,
    NULL,
    NULL,
    fill_report_style_4,
    NULL,
};

static
kpm_sub_data_t gen_kpm_subs(kpm_ran_function_def_t const* ran_func)
{
  assert(ran_func != NULL);
  assert(ran_func->ric_event_trigger_style_list != NULL);

  kpm_sub_data_t kpm_sub = {0};

  assert(ran_func->ric_event_trigger_style_list[0].format_type == FORMAT_1_RIC_EVENT_TRIGGER);
  kpm_sub.ev_trg_def.type = FORMAT_1_RIC_EVENT_TRIGGER;
  kpm_sub.ev_trg_def.kpm_ric_event_trigger_format_1.report_period_ms = period_ms;

  kpm_sub.sz_ad = 1;
  kpm_sub.ad = calloc(kpm_sub.sz_ad, sizeof(kpm_act_def_t));
  assert(kpm_sub.ad != NULL && "Memory exhausted");

  ric_report_style_item_t* const report_item = &ran_func->ric_report_style_list[0];
  ric_service_report_e const report_style_type = report_item->report_style_type;
  *kpm_sub.ad = get_kpm_act_def[report_style_type](report_item);

  return kpm_sub;
}

static
bool eq_sm(sm_ran_function_t const* elem, int const id)
{
  if (elem->id == id)
    return true;

  return false;
}

static
size_t find_sm_idx(sm_ran_function_t* rf, size_t sz, bool (*f)(sm_ran_function_t const*, int const), int const id)
{
  for (size_t i = 0; i < sz; i++) {
    if (f(&rf[i], id))
      return i;
  }

  assert(0 != 0 && "SM ID could not be found in the RAN Function List");
}

int main(int argc, char* argv[])
{
  kpm_moni_file = fopen("kpm_monitoring_log.csv", "w");

  if (kpm_moni_file == NULL) {
    perror("Error opening file");
    return 1;
  }

  // Write CSV header
  fprintf(kpm_moni_file, "timestamp,gnb_cu_cp_ue_e1ap,gnb_cu_ue_f1ap,ran_ue_id,"
                         "DRB.PdcpSduVolumeDL,DRB.PdcpSduVolumeUL,DRB.RlcSduDelayDl,"
                         "DRB.UEThpDl,DRB.UEThpUl,RRU.PrbTotDl,RRU.PrbTotUl\n");
  fflush(kpm_moni_file);

  fr_args_t args = init_fr_args(argc, argv);

  // Init the xApp
  init_xapp_api(&args);
  sleep(1);

  e2_node_arr_xapp_t nodes = e2_nodes_xapp_api();
  defer({ free_e2_node_arr_xapp(&nodes); });

  assert(nodes.len > 0);

  printf("Connected E2 nodes = %d\n", nodes.len);

  pthread_mutexattr_t attr = {0};
  int rc = pthread_mutex_init(&mtx, &attr);
  assert(rc == 0);

  sm_ans_xapp_t* hndl = calloc(nodes.len, sizeof(sm_ans_xapp_t));
  assert(hndl != NULL);

  ////////////
  // START KPM
  ////////////
  int const KPM_ran_function = 2;

  for (size_t i = 0; i < nodes.len; ++i) {
    e2_node_connected_xapp_t* n = &nodes.n[i];

    size_t const idx = find_sm_idx(n->rf, n->len_rf, eq_sm, KPM_ran_function);
    assert(n->rf[idx].defn.type == KPM_RAN_FUNC_DEF_E && "KPM is not the received RAN Function");
    
    if (n->rf[idx].defn.kpm.ric_report_style_list != NULL) {
      kpm_sub_data_t kpm_sub = gen_kpm_subs(&n->rf[idx].defn.kpm);

      hndl[i] = report_sm_xapp_api(&n->id, KPM_ran_function, &kpm_sub, sm_cb_kpm);
      assert(hndl[i].success == true);

      free_kpm_sub_data(&kpm_sub);
    }
  }
  ////////////
  // END KPM
  ////////////

  xapp_wait_end_api();

  for (int i = 0; i < nodes.len; ++i) {
    if (hndl[i].success == true)
      rm_report_sm_xapp_api(hndl[i].u.handle);
  }
  free(hndl);

  // Stop the xApp
  while (try_stop_xapp_api() == false)
    usleep(1000);

  printf("Test xApp run SUCCESSFULLY\n");
  fclose(kpm_moni_file);
}