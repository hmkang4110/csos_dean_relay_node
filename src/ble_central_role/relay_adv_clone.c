#include "relay_adv_clone.h"

#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/net/buf.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>

#include "inference_service.h"
#include "ble_relay_control.h"

LOG_MODULE_REGISTER(relay_adv, LOG_LEVEL_INF);

/* Vendor-specific HCI helpers for per-adv-set TX power */
static int hci_set_adv_tx_power(struct bt_le_ext_adv *adv, int8_t tx_dbm)
{
    struct bt_hci_cp_vs_write_tx_power_level *cp;
    struct net_buf *buf, *rsp = NULL;
    uint16_t handle = bt_le_ext_adv_get_index(adv);
    int err;

    buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL, sizeof(*cp));
    if (!buf) {
        return -ENOMEM;
    }

    cp = net_buf_add(buf, sizeof(*cp));
    cp->handle_type = BT_HCI_VS_LL_HANDLE_TYPE_ADV;
    cp->handle = sys_cpu_to_le16(handle);
    cp->tx_power_level = tx_dbm;

    err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL, buf, &rsp);
    if (rsp) {
        net_buf_unref(rsp);
    }
    return err;
}

static int hci_read_adv_tx_power(struct bt_le_ext_adv *adv, int8_t *out_dbm)
{
    struct bt_hci_cp_vs_read_tx_power_level *cp;
    struct bt_hci_rp_vs_read_tx_power_level *rp;
    struct net_buf *buf, *rsp = NULL;
    uint16_t handle = bt_le_ext_adv_get_index(adv);
    int err;

    buf = bt_hci_cmd_create(BT_HCI_OP_VS_READ_TX_POWER_LEVEL, sizeof(*cp));
    if (!buf) {
        return -ENOMEM;
    }

    cp = net_buf_add(buf, sizeof(*cp));
    cp->handle_type = BT_HCI_VS_LL_HANDLE_TYPE_ADV;
    cp->handle = sys_cpu_to_le16(handle);

    err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_TX_POWER_LEVEL, buf, &rsp);
    if (err) {
        return err;
    }

    rp = (void *)rsp->data;
    *out_dbm = (int8_t)rp->tx_power_level;
    net_buf_unref(rsp);
    return 0;
}

static size_t build_adv_payload(const struct relay_session *session,
                                struct bt_data *ad,
                                size_t ad_cap)
{
    size_t idx = 0;
    const uint8_t *uuid128_data = (const uint8_t *)session->adv_report.uuid128;
    size_t uuid128_len = session->adv_report.uuid128_count * 16;
    uint8_t uuid128_type = session->adv_report.uuid128_type ? session->adv_report.uuid128_type : BT_DATA_UUID128_ALL;
    const char *name = session->adv_report.name[0] ? session->adv_report.name : "DE&N";

    /* Fallback: ensure primary service UUID is present for scanners like bleak */
    if (uuid128_len == 0) {
        const struct bt_uuid_128 *inf = (const struct bt_uuid_128 *)BT_UUID_INFERENCE_SERVICE;
        uuid128_data = inf->val;
        uuid128_len = 16;
        uuid128_type = BT_DATA_UUID128_ALL;
    }

    if (idx < ad_cap) {
        ad[idx].type = BT_DATA_NAME_COMPLETE;
        ad[idx].data_len = strlen(name);
        ad[idx].data = (const uint8_t *)name;
        idx++;
    }

    if (session->adv_report.uuid16_count && idx < ad_cap) {
        ad[idx].type = BT_DATA_UUID16_ALL;
        ad[idx].data_len = session->adv_report.uuid16_count * sizeof(uint16_t);
        ad[idx].data = (const uint8_t *)session->adv_report.uuid16;
        idx++;
    }

    if (uuid128_len && idx < ad_cap) {
        ad[idx].type = uuid128_type;
        ad[idx].data_len = uuid128_len;
        ad[idx].data = uuid128_data;
        idx++;
    }

    if (session->adv_report.service_data_len && idx < ad_cap) {
        uint8_t type = session->adv_report.service_data_type;
        if (type != BT_DATA_SVC_DATA16 &&
            type != BT_DATA_SVC_DATA32 &&
            type != BT_DATA_SVC_DATA128) {
            type = BT_DATA_SVC_DATA16;
        }
        size_t min_len = (type == BT_DATA_SVC_DATA128) ? 16 :
                         (type == BT_DATA_SVC_DATA32) ? 4 : 2;
        if (session->adv_report.service_data_len >= min_len) {
            ad[idx].type = type;
            ad[idx].data_len = session->adv_report.service_data_len;
            ad[idx].data = session->adv_report.service_data;
            idx++;
        }
    }

    if (session->adv_report.mfg_data_len && idx < ad_cap) {
        ad[idx].type = BT_DATA_MANUFACTURER_DATA;
        ad[idx].data_len = session->adv_report.mfg_data_len;
        ad[idx].data = session->adv_report.mfg_data;
        idx++;
    }

    return idx;
}

int relay_adv_clone_start(struct relay_session *session)
{
    int err;
    struct bt_data ad[7];
    bt_addr_le_t ids[CONFIG_BT_ID_MAX];
    size_t id_count = ARRAY_SIZE(ids);
    char addr_str[BT_ADDR_LE_STR_LEN] = {0};
    char uuid_buf[40] = {0};
    struct bt_le_ext_adv_info adv_info;

    if (!session) {
        return -EINVAL;
    }

    if (session->adv) {
        err = bt_le_ext_adv_start(session->adv, BT_LE_EXT_ADV_START_DEFAULT);
        if (err == -EALREADY) {
            return 0;
        }
        return err;
    }

    /* Try to create a fresh identity using the DEAN MAC when possible. */
    bt_addr_le_t *addr = (bt_addr_le_t *)&session->adv_report.addr;
    const bool addr_is_rpa = bt_addr_le_is_rpa(addr);
    const bool addr_is_static = (addr->type == BT_ADDR_LE_RANDOM) &&
                                ((addr->a.val[5] & 0xC0) == 0xC0);

    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    /* Check if identity already exists for this MAC */
    if (!addr_is_rpa && addr_is_static) {
        bt_addr_le_t known_addrs[CONFIG_BT_ID_MAX];
        size_t known_cnt = ARRAY_SIZE(known_addrs);
        bt_id_get(known_addrs, &known_cnt);
        for (size_t i = 0; i < known_cnt; i++) {
            if (bt_addr_le_cmp(&known_addrs[i], addr) == 0) {
                session->adv_id = (int)i;
                LOG_INF("[ADV] found existing id=%d for DEAN MAC (%s)", session->adv_id, addr_str);
                break;
            }
        }
    }

    if (session->adv_id < 0) {
        session->adv_id = bt_id_create((addr_is_rpa || !addr_is_static) ? NULL : addr, NULL);
        if (session->adv_id == -EINVAL && addr_is_static) {
            /* Controller/stack may refuse cloning into a new id; retry with NULL. */
            session->adv_id = bt_id_create(NULL, NULL);
        }
    }

    if (session->adv_id == -ENOMEM) {
        bt_id_get(ids, &id_count);
        if (id_count <= 1) {
            LOG_ERR("[ADV] no identities available");
            return -ENOMEM;
        }
        /* avoid BT_ID_DEFAULT (0) since bt_id_reset forbids it */
        session->adv_id = 1 + (session->index % (id_count - 1));
        LOG_WRN("[ADV] identity pool full, reuse id=%d", session->adv_id);
    } else if (session->adv_id < 0) {
        LOG_ERR("[ADV] identity create failed (%d)", session->adv_id);
        return session->adv_id;
    }

    if (!addr_is_rpa && addr_is_static && session->adv_id != BT_ID_DEFAULT) {
        /* Do not reset if already present; just use existing identity */
        LOG_INF("[ADV] id=%d matches DEAN MAC (%s), use identity as-is",
                session->adv_id, addr_str);
    } else if (addr_is_rpa || !addr_is_static) {
        LOG_WRN("[ADV] target addr not cloneable; using relay identity");
    }

    struct bt_le_adv_param adv_param = {
        .id = (uint8_t)session->adv_id,
        .sid = session->index,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
        .options = BT_LE_ADV_OPT_EXT_ADV |
                //    BT_LE_ADV_OPT_NO_2M |      /* keep secondary on 1M for BlueZ/Bleak */
                   BT_LE_ADV_OPT_CONNECTABLE,
    };

    err = bt_le_ext_adv_create(&adv_param, NULL, &session->adv);
    if (err) {
        LOG_ERR("[ADV] create failed (%d)", err);
        return err;
    }

    /* Log the effective advertising address chosen by the controller for this set */
    if (bt_le_ext_adv_get_info(session->adv, &adv_info) == 0) {
        bt_addr_le_to_str(&adv_info.addr, addr_str, sizeof(addr_str));
        LOG_INF("[ADV] adv set=%p id=%d address on-air: %s", session->adv, session->adv_id, addr_str);
    } else {
        LOG_WRN("[ADV] failed to read adv address for set=%p", session->adv);
    }

    size_t ad_len = build_adv_payload(session, ad, ARRAY_SIZE(ad));
    /* Log first 128-bit UUID, if present */
    if (session->adv_report.uuid128_count > 0) {
        bin2hex((const uint8_t *)session->adv_report.uuid128[0], 16, uuid_buf, sizeof(uuid_buf));
        LOG_INF("[ADV] UUID128[0]=%s", uuid_buf);
    } else {
        LOG_INF("[ADV] UUID128[0]=<fallback or none>");
    }

    LOG_INF("[ADV] setting data: len=%zu name=\"%s\" svc16_cnt=%zu svc128_cnt=%zu mfg_len=%zu",
            ad_len,
            session->adv_report.name[0] ? session->adv_report.name : "DE&N",
            session->adv_report.uuid16_count,
            session->adv_report.uuid128_count,
            session->adv_report.mfg_data_len);

    err = bt_le_ext_adv_set_data(session->adv, ad, ad_len, NULL, 0);
    if (err) {
        LOG_ERR("[ADV] set data failed (%d) len=%zu name_len=%zu svc_len=%zu mfg_len=%zu",
                err,
                ad_len,
                strlen(session->adv_report.name),
                session->adv_report.service_data_len,
                session->adv_report.mfg_data_len);
        bt_le_ext_adv_delete(session->adv);
        session->adv = NULL;
        return err;
    }

    /* Boost TX power per request; uses adv handle index. */
    int8_t effective = 0;
    hci_set_adv_tx_power(session->adv, 20);
    if (hci_read_adv_tx_power(session->adv, &effective) == 0) {
        LOG_INF("[ADV] TX power set=20 dBm effective=%d dBm", effective);
    }

    err = bt_le_ext_adv_start(session->adv, BT_LE_EXT_ADV_START_DEFAULT);
    if (err) {
        LOG_ERR("[ADV] start failed (%d)", err);
        bt_le_ext_adv_delete(session->adv);
        session->adv = NULL;
        return err;
    }

    LOG_INF("[ADV] proxy adv started for session %d as %s", 
        session->index, session->adv_report.name);
    return 0;
}

void relay_adv_clone_stop(struct relay_session *session)
{
    if (!session || !session->adv) {
        return;
    }

    bt_le_ext_adv_stop(session->adv);
    bt_le_ext_adv_delete(session->adv);
    session->adv = NULL;
}
