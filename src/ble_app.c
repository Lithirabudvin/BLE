/*
 * BLE core: enable stack, advertise, track the connection.
 * Shared by the audio streaming service and the OTA (SMP) service.
 *
 * Advertising packets are limited to 31 bytes, so:
 *   adv data      : flags (3) + audio service UUID128 (18)      = 21 bytes
 *   scan response : SMP service UUID128 (18) + name (2 + len)
 * The name therefore has to be 11 characters or fewer.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>

#include "ble_app.h"
#include "ble_audio.h"
#include "ota.h"

static struct k_work_delayable tune_work;
static int tune_step;

LOG_MODULE_REGISTER(ble_app, LOG_LEVEL_INF);

static struct bt_conn *current_conn;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_AUDIO_SERVICE_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, OTA_SMP_UUID_VAL),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static int start_advertising(void)
{
    return bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad),
                           sd, ARRAY_SIZE(sd));
}

static void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
    ARG_UNUSED(conn);
    LOG_INF("MTU updated: TX=%u RX=%u", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
    .att_mtu_updated = mtu_updated,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    LOG_INF("Connected");
    current_conn = bt_conn_ref(conn);

    /* Ask for the fastest link settings; the central may still refuse.
     * Faster links help both audio streaming and OTA transfer speed. */
    tune_step = 0;
    k_work_reschedule(&tune_work, K_SECONDS(2));
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    LOG_INF("Disconnected (reason %u)", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
}




static void tune_fn(struct k_work *w)
{
    ARG_UNUSED(w);
    if (!current_conn) {
        return;
    }

    int e = 0;

    switch (tune_step++) {
    case 0: e = bt_conn_le_data_len_update(current_conn, BT_LE_DATA_LEN_PARAM_MAX); break;
    case 1: e = bt_conn_le_phy_update(current_conn, BT_CONN_LE_PHY_PARAM_2M); break;
    case 2: e = bt_conn_le_param_update(current_conn, BT_LE_CONN_PARAM(6, 12, 0, 400)); break;
    default: return;
    }
    if (e) {
        LOG_WRN("Link tuning step %d failed: %d", tune_step - 1, e);
    }
    k_work_reschedule(&tune_work, K_MSEC(700));
}

static void recycled(void)
{
    int err = start_advertising();

    if (err) {
        LOG_ERR("Re-advertising failed: %d", err);
    } else {
        LOG_INF("Advertising restarted");
    }
}

static void le_phy_updated(struct bt_conn *conn,
                           struct bt_conn_le_phy_info *param)
{
    ARG_UNUSED(conn);
    LOG_INF("PHY: TX %u RX %u (1=1M, 2=2M)", param->tx_phy, param->rx_phy);
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
                             uint16_t latency, uint16_t timeout)
{
    ARG_UNUSED(conn);
    LOG_INF("Conn params: interval %u.%02u ms, latency %u, timeout %u ms",
            (interval * 5U) / 4U, ((interval * 5U) % 4U) * 25U,
            latency, timeout * 10U);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected        = connected,
    .disconnected     = disconnected,
    .recycled         = recycled,
    .le_phy_updated   = le_phy_updated,
    .le_param_updated = le_param_updated,
};

struct bt_conn *ble_app_get_conn(void)
{
    return current_conn;
}

int ble_app_init(void)
{
    k_work_init_delayable(&tune_work, tune_fn);

    int err = bt_enable(NULL);
    LOG_INF("bt_enable: %d", err);
    if (err) {
        return err;
    }

    bt_gatt_cb_register(&gatt_callbacks);

    err = start_advertising();
    LOG_INF("adv_start: %d", err);
    return err;
}