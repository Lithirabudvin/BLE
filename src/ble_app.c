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

/*
 * The tune sequence above runs ONCE per connection and then stops. That's
 * fine for MTU/DLE/PHY, which only need to be set once -- but the central
 * (Android in particular) can re-open connection-parameter negotiation on
 * its OWN schedule at any later point in the connection, independent of
 * anything we request. Each log has shown the interval drifting back up
 * minutes after our one-shot request already succeeded -- that's the
 * central doing this, not our firmware failing to ask.
 *
 * This watchdog re-asks for the fast interval every time le_param_updated()
 * reports something slower than we want, for as long as the connection
 * lasts, instead of giving up after the first attempt.
 */
#define DESIRED_INTERVAL_MAX_UNITS  6   /* 6 * 1.25ms = 7.5 ms */

static struct k_work_delayable retune_work;

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

/*
 * ATT MTU exchange. The MTU can only be negotiated ONCE per connection,
 * and it stays at the 23-byte default forever if neither side asks for
 * more. Many BLE centrals (test tools, nRF Connect for Desktop, some
 * generic scanners) never request a bigger MTU on their own -- so the
 * peripheral has to ask. Requesting it here, immediately on connect,
 * also wins the race against a central that only exchanges MTU later
 * (e.g. only when you click a button in a desktop tool), since a second
 * exchange attempt after one has already completed is simply rejected.
 *
 * At MTU=23 a notification can carry only 18 payload bytes, so a 1920
 * byte audio block needs ~107 notifications -- far more than fit in the
 * time budget, which is what caused the "PCM queue full" drops. Once
 * this succeeds the sender in ble_audio.c automatically starts using
 * the bigger payload (it reads bt_gatt_get_mtu() per packet already).
 */
static struct bt_gatt_exchange_params mtu_exchange_params;

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_exchange_params *params)
{
    ARG_UNUSED(params);

    if (err) {
        LOG_WRN("MTU exchange failed (err %u), staying at %u bytes",
                err, bt_gatt_get_mtu(conn));
    } else {
        LOG_INF("MTU exchange accepted, now %u bytes", bt_gatt_get_mtu(conn));
    }
}

static void request_mtu_exchange(struct bt_conn *conn)
{
    mtu_exchange_params.func = mtu_exchange_cb;

    int err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);

    if (err) {
        /* -EALREADY here just means the central already exchanged MTU
         * before we asked (e.g. during service discovery) -- harmless,
         * whatever value resulted is what we're stuck with. */
        LOG_WRN("MTU exchange request failed: %d", err);
    }
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    LOG_INF("Connected");
    current_conn = bt_conn_ref(conn);

    /* Ask for a bigger MTU right away -- see request_mtu_exchange(). */
    request_mtu_exchange(conn);

    /* Data length / PHY / connection interval tuning, run as a short
     * sequence with a small gap between steps rather than all at once,
     * since a BLE controller can only have one such procedure active
     * per connection at a time. Starts almost immediately now (used to
     * wait a full 2 s first) so audio has less time to overrun the
     * queue before the link is tuned. */
    tune_step = 0;
    k_work_reschedule(&tune_work, K_MSEC(300));
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    LOG_INF("Disconnected (reason %u)", reason);

    k_work_cancel_delayable(&tune_work);
    k_work_cancel_delayable(&retune_work);

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
    k_work_reschedule(&tune_work, K_MSEC(500));
}

static void retune_fn(struct k_work *w)
{
    ARG_UNUSED(w);

    if (!current_conn) {
        return;
    }

    /* Same pinned request tune_fn's step 2 used -- ask again, since the
     * central has drifted away from it on its own. */
    int e = bt_conn_le_param_update(current_conn, BT_LE_CONN_PARAM(6, 6, 0, 400));

    if (e) {
        LOG_WRN("Re-tune param update failed: %d", e);
    } else {
        LOG_INF("Central drifted off the fast interval -- re-requested it");
    }
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

    if (interval > DESIRED_INTERVAL_MAX_UNITS) {
        /* Give the central a few seconds in case this is part of its own
         * settling process, then ask again. Runs for as long as the
         * connection lasts -- unlike the one-shot tune sequence, this
         * keeps correcting drift for the whole session. */
        k_work_reschedule(&retune_work, K_SECONDS(3));
    }
}

static void le_data_len_updated(struct bt_conn *conn,
                                struct bt_conn_le_data_len_info *info)
{
    ARG_UNUSED(conn);
    LOG_INF("DLE: TX %u B/%u us, RX %u B/%u us",
            info->tx_max_len, info->tx_max_time,
            info->rx_max_len, info->rx_max_time);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected           = connected,
    .disconnected        = disconnected,
    .recycled            = recycled,
    .le_phy_updated      = le_phy_updated,
    .le_param_updated    = le_param_updated,
    .le_data_len_updated = le_data_len_updated,
};

struct bt_conn *ble_app_get_conn(void)
{
    return current_conn;
}

int ble_app_init(void)
{
    k_work_init_delayable(&tune_work, tune_fn);
    k_work_init_delayable(&retune_work, retune_fn);

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