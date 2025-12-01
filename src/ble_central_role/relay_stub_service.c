/* relay_stub_services.c
 *
 * 목적:
 *  - SLIMHUB 입장에서 DEAN node 와 동일한 BLE 서비스 / UUID 를 가진 것처럼 보이게 한다.
 *  - 실제 동작은 inference_service 만 사용하고,
 *    나머지 서비스는 "형식만 있는 stub" 으로 구현한다.
 */
#include "relay_stub_service.h"
#include "ble_relay_control.h"

#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

#include "ble.h"
#include "config_service.h"
#include "grideye_service.h"
#include "peripheral_service.h"
#include "env_service.h"
#include "sound_service.h"
// inference_service.h 는 별도 실제 구현 파일에서 사용

/* 공통 dummy 읽기 함수: attr->user_data 의 버퍼를 그대로 반환 */
static ssize_t dummy_read(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr,
                          void *buf, uint16_t len, uint16_t offset)
{
    const void *value = attr->user_data;

    /* attr->user_data 가 NULL 인 경우도 방어 */
    if (!value) {
        static uint8_t zero;
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &zero, sizeof(zero));
    }

    /* 여기서는 길이를 몰라서 "최대 len" 만큼만 읽어주고, offset 은 bt_gatt_attr_read 에 맡긴다. */
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, len);
}

/* 공통 dummy 쓰기 함수: 수신 데이터는 그대로 버리지만, 길이만큼 처리가 된 것처럼 리턴 */
static ssize_t dummy_write(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           const void *buf, uint16_t len,
                           uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(buf);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    /* 상위에서는 ATT 성공으로 인식하게 len 그대로 반환 */
    return len;
}

/* 공통 CCC 변경 콜백: 단순히 enable/disable 정보만 로컬 플래그에 저장 */
#define DEFINE_CCC_FLAG(name)          \
    static bool name;                  \
    static void name##_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) \
    {                                  \
        ARG_UNUSED(attr);              \
        name = (value == BT_GATT_CCC_NOTIFY); \
    }

/* 각 서비스별 notify enable 상태 플래그 */
DEFINE_CCC_FLAG(cfg_notify_enabled);
DEFINE_CCC_FLAG(env_notify_enabled);
DEFINE_CCC_FLAG(grideye_pred_notify_enabled);
DEFINE_CCC_FLAG(grideye_raw_notify_enabled);
DEFINE_CCC_FLAG(periph_notify_enabled);
DEFINE_CCC_FLAG(sound_pred_notify_enabled);
DEFINE_CCC_FLAG(sound_raw_notify_enabled);

/* ----------------- 1) CONFIG SERVICE (기본 설정/이름/위치 등) ----------------- */

static uint8_t  cfg_file_transfer_dummy[20];

device_config_t dean_device_conf = {
    .device_name = "RELAY_NODE",
    .location = "RELAY_LOC",
    .update_flag = 0,
};

static bt_gatt_attr_read_func_t name_read_cb(struct bt_conn *conn,
                                             const struct bt_gatt_attr *attr,
                                             void *buf, uint16_t len,
                                             uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, dean_device_conf.device_name,
                             strlen(dean_device_conf.device_name));
}

static bt_gatt_attr_write_func_t name_write_cb(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               const void *buf,
                                               uint16_t len,
                                               uint16_t offset,
                                               uint8_t flags)
{
    memset(dean_device_conf.device_name, 0, sizeof(dean_device_conf.device_name));
    memcpy(dean_device_conf.device_name, buf, len);

    ble_relay_send_write_to_dean(BT_UUID_CHRC_DEVICE_NAME, buf, len);

    return len;
}

static bt_gatt_attr_read_func_t location_read_cb(struct bt_conn *conn,
                                                 const struct bt_gatt_attr *attr,
                                                 void *buf, uint16_t len,
                                                 uint16_t offset)
{
    dean_device_conf.update_flag = 1;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, dean_device_conf.location,
                             strlen(dean_device_conf.location));
}

static bt_gatt_attr_write_func_t location_write_cb(struct bt_conn *conn,
                                                   const struct bt_gatt_attr *attr,
                                                   const void *buf,
                                                   uint16_t len,
                                                   uint16_t offset,
                                                   uint8_t flags)
{
    dean_device_conf.update_flag = 1;
    memset(dean_device_conf.location, 0, sizeof(dean_device_conf.location));
    memcpy(dean_device_conf.location, buf, len);

    ble_relay_send_write_to_dean(BT_UUID_CHRC_LOCATION, buf, len);

    return len;
}

void process_file_transfer_write(const void *buf, uint16_t len);

static bt_gatt_attr_write_func_t file_write_cb(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               const void *buf,
                                               uint16_t len,
                                               uint16_t offset,
                                               uint8_t flags)
{
    // process_file_transfer_write(buf, len);
    ble_relay_send_write_to_dean(BT_UUID_CHRC_FILE_TRANSFER, buf, len);
    return len;
}
void process_file_transfer_write(const void *buf, uint16_t len)
{
    int ret;
    int retry_cnt = 0;

    struct ble_file_transfer_data_packet packet = {0};
    struct ble_file_transfer_ack_packet ack_packet = {0};

    memcpy(&packet, buf, len);

    // switch (packet.cmd)
    // {
    // case BLE_FILE_TRANSFER_CMD_START:
    // {
    //     memset(current_file_name, 0, sizeof(current_file_name));
    //     snprintf(current_file_name, packet.size + 6, "/SD:/%s", packet.data);
    //     printk("File transfer start: %s\n", current_file_name);

    //     /* changed lines from here */
    //     /* 기존 파일 삭제 → 새 파일 생성 준비 (덮어쓰기 효과) */
    //     if (!k_mutex_lock(&sdcard_mutex, K_SECONDS(3)))
    //     {
    //         if (!get_disk_status())
    //         {
    //             ret = fs_unlink(current_file_name);
    //             if (ret == 0)
    //             {
    //                 printk("Existing file removed: %s\n", current_file_name);
    //             }
    //         }
    //         k_mutex_unlock(&sdcard_mutex);
    //     }
    //     else
    //     {
    //         printk("Failed to lock sdcard mutex\n");
    //     }
    //     /* changed lines to here */

    //     struct ble_file_transfer_ack_packet ack_packet;
    //     ack_packet.cmd = BLE_FILE_TRANSFER_CMD_START;
    //     ack_packet.seq = 0;

    //     bt_config_file_transfer(&ack_packet, sizeof(ack_packet));
    //     break;
    // }
    // case BLE_FILE_TRANSFER_CMD_DATA:
    // {
    //     struct fs_file_t fp;
    //     fs_file_t_init(&fp);

    //     if (!k_mutex_lock(&sdcard_mutex, K_SECONDS(3)))
    //     {
    //         if (!get_disk_status())
    //         {
    //             ret = fs_open(&fp, current_file_name, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
    //             if (ret < 0)
    //             {
    //                 printk("Failed to open file: %s\n", current_file_name);
    //                 ack_packet.cmd = BLE_FILE_TRANSFER_CMD_FAILED;
    //                 ack_packet.seq = packet.seq;
    //                 bt_config_file_transfer(&ack_packet, sizeof(ack_packet));
    //                 break;
    //             }
    //             ret = fs_write(&fp, packet.data, packet.size);
    //             if (ret < 0)
    //             {
    //                 printk("Failed to write file: %s\n", current_file_name);
    //                 fs_close(&fp);
    //                 ack_packet.cmd = BLE_FILE_TRANSFER_CMD_FAILED;
    //                 ack_packet.seq = packet.seq;
    //                 bt_config_file_transfer(&ack_packet, sizeof(ack_packet));
    //                 break;
    //             }
    //             fs_close(&fp);
    //         }
    //         k_mutex_unlock(&sdcard_mutex);
    //         ack_packet.cmd = BLE_FILE_TRANSFER_CMD_DATA;
    //         ack_packet.seq = packet.seq;
    //         bt_config_file_transfer(&ack_packet, sizeof(ack_packet));
    //         break;
    //     }
    //     else
    //     {
    //         printk("Failed to lock sdcard mutex\n");
    //         ack_packet.cmd = BLE_FILE_TRANSFER_CMD_FAILED;
    //         ack_packet.seq = packet.seq;
    //         bt_config_file_transfer(&ack_packet, sizeof(ack_packet));
    //         break;
    //     }
    // }
    // case BLE_FILE_TRANSFER_CMD_END:
    // {
    //     printk("File transfer end: %s\n", current_file_name);
    //     ack_packet.cmd = BLE_FILE_TRANSFER_CMD_END;
    //     ack_packet.seq = 0;
    //     bt_config_file_transfer(&ack_packet, sizeof(ack_packet));
    //     inference_msgq_packet_data_t packet;
    //     packet.type = INFERENCE_MSGQ_TYPE_IFDATA_LOAD;
    //     k_msgq_put(&inference_msgq, &packet, K_NO_WAIT);
    //     break;
    // }
    // case BLE_FILE_TRANSFER_CMD_REMOVE:
    // {
    //     memset(current_file_name, 0, sizeof(current_file_name));
    //     snprintf(current_file_name, packet.size + 6, "/SD:/%s", packet.data);
    //     printk("File remove request: %s\n", current_file_name);

    //     ret = fs_unlink(current_file_name);
    //     if (ret < 0)
    //     {
    //         printk("Failed to remove file: %s\n", current_file_name);
    //         ack_packet.cmd = BLE_FILE_TRANSFER_CMD_FAILED;
    //         ack_packet.seq = 0;
    //         bt_config_file_transfer(&ack_packet, sizeof(ack_packet));
    //         break;
    //     }
    //     ack_packet.cmd = BLE_FILE_TRANSFER_CMD_REMOVE;
    //     ack_packet.seq = 0;
    //     bt_config_file_transfer(&ack_packet, sizeof(ack_packet));
    //     break;
    // }
    // }
}

BT_GATT_SERVICE_DEFINE(config_svr,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CONFIG_SERVICE),

    /* 원래 FILE_TRANSFER 특성:
     *  - 실제 DEAN node 에서는 파일 전송용이었지만,
     *  - 여기서는 단순 dummy 버퍼 + notify 용도로만 사용
     */
    BT_GATT_CHARACTERISTIC(BT_UUID_CHRC_FILE_TRANSFER,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           NULL, file_write_cb,
                           cfg_file_transfer_dummy),
    BT_GATT_CCC(cfg_notify_enabled_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* DEVICE_NAME */
    BT_GATT_CHARACTERISTIC(BT_UUID_CHRC_DEVICE_NAME,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           name_read_cb, name_write_cb,
                           dean_device_conf.device_name),

    /* DEVICE_LOCATION */
    BT_GATT_CHARACTERISTIC(BT_UUID_CHRC_LOCATION,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           location_read_cb, location_write_cb,
                           dean_device_conf.location)
);

/* ----------------- 2) ENVIRONMENT SERVICE (환경값 dummy) ----------------- */

/* SLIMHUB 가 ENV 서비스의 notify 를 enable 할 수 있도록
 * 최소한 하나의 "send" 특성만 제공 (필요 시 나머지 확장 가능)
 */
static uint8_t env_dummy_data[8];

BT_GATT_SERVICE_DEFINE(env_svr,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_ENV_SERVICE),

    BT_GATT_CHARACTERISTIC(BT_UUID_CHRC_ENV_SEND,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           dummy_read, NULL,
                           env_dummy_data),
    BT_GATT_CCC(env_notify_enabled_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* ----------------- 3) GRIDEYE SERVICE (열화상 관련 dummy) ----------------- */

/* prediction / raw 두 개 다 DEAN node 에서 쓰므로 그대로 흉내 */
static uint8_t grideye_pred_dummy[8];
static uint8_t grideye_raw_dummy[64];
static bool grideye_notify_enabled;
static void ccc_cfg_grideye_changed(const struct bt_gatt_attr *attr,
                                    uint16_t value)
{
    grideye_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static bt_gatt_attr_write_func_t grideye_write_cb(struct bt_conn *conn, 
                                                  const struct bt_gatt_attr *attr,
                                                  const void *buf,
                                                  uint16_t len,
                                                  uint16_t offset,
                                                  uint8_t flags)
{
    // grideye_prediction_callback(buf, len);
    ble_relay_send_write_to_dean(BT_UUID_CHRC_GRIDEYE_PREDICTION, buf, len);
    return len;
}

BT_GATT_SERVICE_DEFINE(grideye_svr,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_GRIDEYE_SERVICE),

    /* prediction */
    BT_GATT_CHARACTERISTIC(BT_UUID_CHRC_GRIDEYE_PREDICTION,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           dummy_read, grideye_write_cb,
                           grideye_pred_dummy),
    BT_GATT_CCC(ccc_cfg_grideye_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* raw data */
    BT_GATT_CHARACTERISTIC(BT_UUID_CHRC_GRIDEYE_RAW_STREAMING,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           dummy_read, NULL,
                           grideye_raw_dummy),
    BT_GATT_CCC(NULL,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* ----------------- 4) PERIPHERAL SERVICE (카운터/제어 dummy) ----------------- */

static uint8_t periph_counter_dummy[4];
static uint8_t periph_ctrl_dummy[4];
static bool peripheral_action_notify_enabled;
static void ccc_cfg_peripheral_action_changed(const struct bt_gatt_attr *attr,
                                uint16_t value)
{
    peripheral_action_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

BT_GATT_SERVICE_DEFINE(peripheral_svr,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_peripheral_ACTION_SERVICE),

    BT_GATT_CHARACTERISTIC(BT_UUID_CHRC_peripheral_ACTION_DATA,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           dummy_read, NULL,
                           periph_counter_dummy),
    BT_GATT_CCC(ccc_cfg_peripheral_action_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* ----------------- 5) SOUND SERVICE (소리 추론 결과 / raw dummy) ----------------- */

static uint8_t sound_pred_dummy[8];
static uint8_t sound_raw_dummy[32];
bool model_notify_enabled;
static void ccc_cfg_sound_changed_event(const struct bt_gatt_attr *attr, uint16_t value)
{
    model_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

BT_GATT_SERVICE_DEFINE(sound_svr,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_SOUND_SERVICE),

    BT_GATT_CHARACTERISTIC(BT_UUID_SOUND_SERVICE,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           dummy_read, NULL,
                           sound_pred_dummy),
    BT_GATT_CCC(ccc_cfg_sound_changed_event,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // BT_GATT_CHARACTERISTIC(BT_UUID_CHRC_SOUND_RAWDATA,
    //                        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
    //                        BT_GATT_PERM_READ,
    //                        dummy_read, NULL,
    //                        sound_raw_dummy),
    // BT_GATT_CCC(sound_raw_notify_enabled_ccc_changed,
    //             BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* ----------------- 6) UBINOS / PAAR 서비스는 필요 시 나중에 추가 ----------------- */
/* 현재 SLIMHUB 기준으로 사용하지 않는다면 굳이 올릴 필요는 없음.
 * 필요해지면 ubinos_service.h 에 정의된 UUID 로 위와 동일한 패턴으로
 * BT_GATT_SERVICE_DEFINE(...) 하나 더 만들어주면 된다.
 */

