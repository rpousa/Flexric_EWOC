#include "not_handler_ric.h"

#include <assert.h>
#include <stdbool.h>

#include "e2_node.h"

#include "../lib/e2ap/e2ap_global_node_id_wrapper.h"
#include "../util/alg_ds/ds/lock_guard/lock_guard.h"
#include "../util/alg_ds/alg/alg.h"

#include "iApp/e42_iapp_api.h"

static
bool eq_global_e2_node_id_e2_node(void const* it, void const* val)
{
  e2_node_t* n = (e2_node_t*)it;
  global_e2_node_id_t* id = (global_e2_node_id_t*)val;

  return eq_global_e2_node_id(&n->id, id);
}

void notification_handle_ric(near_ric_t* ric, sctp_msg_t const* msg)
{
  assert(ric != NULL);
  assert(msg != NULL && msg->type == SCTP_MSG_NOTIFICATION);
  assert(msg->notif->sn_header.sn_type == SCTP_SHUTDOWN_EVENT && "Only shutdown event supported");

  global_e2_node_id_t* id = e2ap_rm_sock_addr_ric(&ric->ep, &msg->info);

  // GUARD: SCTP association shut down without a completed E2 SETUP
  // (e.g. cuup_e's flaky cross-host link). Nothing was ever registered
  // for this sockaddr, so there is nothing to remove. Ignore quietly.
  if (id == NULL) {
    printf("[E2AP]: SCTP_SHUTDOWN for unregistered association — ignoring\n");
    return;                       // note: BEFORE defer, so no free of NULL
  }

  defer( { free_global_e2_node_id(id);  free(id); } );

  {
  lock_guard(&ric->conn_e2_nodes_mtx);

  void* it  = seq_front(&ric->conn_e2_nodes);
  void* end = seq_end(&ric->conn_e2_nodes);

  it = find_if(&ric->conn_e2_nodes, it, end, id, eq_global_e2_node_id_e2_node);

  // Do NOT rely on assert() here — you build with NDEBUG.
  if (it == end) {
    printf("[E2AP]: E2 node for shutdown not in conn_e2_nodes — skipping\n");
    return;                       // defer still frees id correctly
  }

  e2_node_t* n = (e2_node_t*)it;
  free_e2_node(n);

  void* it_next = seq_next(&ric->conn_e2_nodes, it);
  seq_erase(&ric->conn_e2_nodes, it, it_next);
  }

  rm_e2_node_iapp_api(id);
}
