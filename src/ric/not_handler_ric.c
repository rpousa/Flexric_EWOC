#include "not_handler_ric.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

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

  // Look up (and remove) the sockaddr -> global_e2_node_id mapping for the
  // association that just shut down. This returns NULL if the association was
  // never fully registered (e.g. SCTP came up but no E2 SETUP completed —
  // typical for a flaky cross-host link that flaps before/at E2 SETUP).
  global_e2_node_id_t* id = e2ap_rm_sock_addr_ric(&ric->ep, &msg->info);

  // GUARD 1: association shut down without a completed E2 SETUP.
  // Nothing was ever registered for this sockaddr -> nothing to clean up.
  // Return BEFORE installing the defer so we never free a NULL/garbage id.
  if (id == NULL) {
    printf("[E2AP]: SCTP_SHUTDOWN for unregistered association — ignoring\n");
    return;
  }

  // From here on 'id' is valid and owned by us; free it on every exit path.
  defer( { free_global_e2_node_id(id);  free(id); } );

  // Remove the node from conn_e2_nodes (RIC-side registry) under the lock.
  // IMPORTANT: do NOT return from inside this block while the mutex is held.
  // Record whether we actually found/removed it, then act after releasing.
  bool found = false;
  {
    lock_guard(&ric->conn_e2_nodes_mtx);

    void* it  = seq_front(&ric->conn_e2_nodes);
    void* end = seq_end(&ric->conn_e2_nodes);

    it = find_if(&ric->conn_e2_nodes, it, end, id, eq_global_e2_node_id_e2_node);

    // GUARD 2: We build with NDEBUG, so assert(it != end) is compiled out.
    // Handle "not present" explicitly instead of dereferencing the sentinel.
    if (it != end) {
      found = true;

      // ASan does not like the memmove inside seq_erase_free, so free the
      // node explicitly, then erase the (now stale) slot.
      e2_node_t* n = (e2_node_t*)it;
      free_e2_node(n);

      void* it_next = seq_next(&ric->conn_e2_nodes, it);
      seq_erase(&ric->conn_e2_nodes, it, it_next);
    } else {
      printf("[E2AP]: E2 node for shutdown not in conn_e2_nodes — skipping\n");
    }
  } // <-- conn_e2_nodes_mtx released here on ALL paths (no early return above)

  // Only touch the iApp registration tree if the node was genuinely present.
  // This prevents rm_e2_node_iapp_api() from walking its RB-tree with a stale
  // id (the earlier MCC 13873 / NB_ID 12341 garbage crash).
  if (found)
    rm_e2_node_iapp_api(id);
}
