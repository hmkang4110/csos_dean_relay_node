#include "relay_mapping.h"

#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(relay_map, LOG_LEVEL_INF);

struct relay_session g_relay_sessions[MAX_RELAY_SESSIONS] = {0};

void relay_mapping_init(void)
{
    memset(g_relay_sessions, 0, sizeof(g_relay_sessions));
    for (int i = 0; i < MAX_RELAY_SESSIONS; i++) {
        g_relay_sessions[i].index = (uint8_t)i;
    }
}

struct relay_session *relay_mapping_find_empty(void)
{
    for (int i = 0; i < MAX_RELAY_SESSIONS; i++) {
        if (!g_relay_sessions[i].is_active) {
            return &g_relay_sessions[i];
        }
    }
    return NULL;
}

struct relay_session *relay_mapping_alloc(const struct dean_adv_report *report)
{
    struct relay_session *session = relay_mapping_find_empty();
    if (!session) {
        return NULL;
    }

    memset(session, 0, sizeof(*session));
    session->index = (uint8_t)(session - g_relay_sessions);
    session->is_active = true;
    memcpy(&session->adv_report, report, sizeof(*report));
    return session;
}

struct relay_session *relay_mapping_by_dean_conn(struct bt_conn *conn)
{
    for (int i = 0; i < MAX_RELAY_SESSIONS; i++) {
        if (g_relay_sessions[i].is_active && g_relay_sessions[i].conn_dean == conn) {
            return &g_relay_sessions[i];
        }
    }
    return NULL;
}

struct relay_session *relay_mapping_by_hub_conn(struct bt_conn *conn)
{
    for (int i = 0; i < MAX_RELAY_SESSIONS; i++) {
        if (g_relay_sessions[i].is_active && g_relay_sessions[i].conn_slimhub == conn) {
            return &g_relay_sessions[i];
        }
    }
    return NULL;
}

void relay_mapping_release(struct relay_session *session)
{
    if (!session) {
        return;
    }

    uint8_t keep_index = session->index;

    if (session->conn_dean) {
        bt_conn_unref(session->conn_dean);
        session->conn_dean = NULL;
    }

    if (session->conn_slimhub) {
        bt_conn_unref(session->conn_slimhub);
        session->conn_slimhub = NULL;
    }

    if (session->adv) {
        bt_le_ext_adv_stop(session->adv);
        bt_le_ext_adv_delete(session->adv);
        session->adv = NULL;
    }

    memset(session, 0, sizeof(*session));
    session->index = keep_index;
}
