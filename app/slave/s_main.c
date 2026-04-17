/*
 * Slave (Implant) - Implantable Device Application
 *
 * Ultra-low power operation: kimagu 2023.10.13
 *   1. MCU in Stop 2 mode (~2 uA)
 *   2. CC2500 in WOR mode (periodic RX sniff at 2.4 GHz)
 *   3. CC2500 GDO0 -> EXTI wakes MCU on beacon detection
 *   4. CC1101 is powered down until communication session
 *
 * Communication flow (Slave never initiates):
 *   1. CC2500 WOR detects wake-up beacon -> GDO0 EXTI -> MCU wakes
 *   2. Parse beacon, remember SESS + assigned MICS CHANNEL (chosen by Master
 *      via LBT before the beacon was sent)
 *   3. CC2500 sends wake-up ACK echoing DEVICE_ID+SESS+CHANNEL
 *   4. CC1101 tunes directly to the assigned MICS channel (no scanning)
 *   5. CC1101 responds to Master commands filtered by SESS + DEVICE_ID
 *   6. CC1101 powers down, CC2500 re-enters WOR, MCU -> Stop 2
 *
 * MCU: STM32U575 Q-series
 * Radios: CC2500 (2.4 GHz WOR wake-up), CC1101 (400 MHz MICS data)
 */

#include "cc1101.h"
#include "cc2500.h"
#include "protocol.h"
#include "stm32u575_lowpower.h"
#include <string.h>

#define AUTH_NONCE_LEN 4U
#define AUTH_TAG_LEN   4U
#define AUTH_ARG_LEN   (AUTH_NONCE_LEN + AUTH_TAG_LEN)
#define STREAM_CFG_SUBCMD 0x01U
#define STREAM_LOGICAL_DATA_MAX 17U
#define STREAM_FEC_COPIES 5U

/* ---------- low-power tuning knobs ---------- */
#define SLAVE_RTC_WAKEUP_INTERVAL_S   300U  /* raise to reduce periodic wakeups */
#define SLAVE_MICS_RX_WINDOW_MS       50U   /* lower to reduce active RX duty */

/* ---------- power trace (debugger-readable) ---------- */
typedef struct {
    uint32_t stop2_entries;
    uint32_t wakeup_exti_count;
    uint32_t wakeup_rtc_count;
    uint32_t wakeup_ok_count;
    uint32_t wakeup_fail_count;
    uint32_t mics_session_count;
    uint32_t mics_timeout_count;
    uint32_t last_stop2_enter_ms;
    uint32_t last_stop2_exit_ms;
    uint32_t last_session_start_ms;
    uint32_t last_session_end_ms;
} Slave_PowerTrace;

volatile Slave_PowerTrace g_slave_pwr_trace = {0};

/* ---------- hardware handles ---------- */
static CC1101_HandleTypeDef g_cc1101;
static CC2500_HandleTypeDef g_cc2500;
static LP_HandleTypeDef     g_lp;

/* These would be defined by CubeMX-generated code */
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern RTC_HandleTypeDef hrtc;

/* ---------- device identity ---------- */
static const uint8_t g_my_id[PROTO_DEVICE_ID_LEN] = {0x00, 0x00, 0x00, 0x01};

/* ---------- authorised Master identity (factory-paired) ----------
 * This Slave accepts wake-up / COMMAND frames ONLY if they carry this
 * MASTER_ID in the payload. Provisioned at the factory during pairing
 * with a specific Programmer. Rejecting frames from unauthorised
 * Masters mitigates the trivial spoof attack where an attacker who
 * learns the DEVICE_ID could otherwise wake the implant at will.
 *
 * NOTE: This is a pre-shared identifier, not a cryptographic key.
 * Strong authentication (HMAC / signatures / nonce-based freshness)
 * must be layered on top at the application level if the threat
 * model requires it. */
static const uint8_t g_authorized_master_id[PROTO_MASTER_ID_LEN] =
    {0xAA, 0xBB, 0xCC, 0xDD};

/* ---------- session state ---------- */
static volatile bool g_wakeup_flag       = false;
static volatile bool g_cc1101_gdo0_flag  = false; /* set in EXTI ISR when CC1101 GDO0 asserts */
static uint8_t g_seq = 0U;
static uint8_t g_session_id     = PROTO_SESS_UNASSIGNED; /* assigned by Master wake-up */
static uint8_t g_active_channel = PROTO_CHANNEL_UNASSIGNED; /* MICS channel carried in beacon */
static uint8_t g_last_rx_seq = 0U;
static bool    g_last_rx_seq_valid = false;
static uint8_t g_session_nonce[AUTH_NONCE_LEN] = {0};
static bool    g_stream_enabled = false;
static uint8_t g_stream_interval_ms = 20U;
static uint8_t g_stream_remaining_frames = 0U;
static uint8_t g_stream_seq = 0U;

static const uint8_t g_auth_key[16] = {
    0x3A, 0x5C, 0x19, 0xE7, 0xA2, 0x4D, 0x77, 0x10,
    0x91, 0x2B, 0xC4, 0x6E, 0x58, 0xFD, 0x03, 0xAB
};

/* ---------- simulated sensor data ---------- */
static uint8_t g_sensor_data[16] = {
    0x01, 0x02, 0x03, 0x04,  /* placeholder sensor values */
    0x05, 0x06, 0x07, 0x08,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};

/* ---------- forward declarations ---------- */
static void Slave_InitHardware(void);
static void Slave_EnterDeepSleep(void);
static int  Slave_HandleWakeUp(void);
static int  Slave_HandleMICSSession(void);
static int  Slave_ProcessCommand(const Proto_Packet *req, Proto_Packet *resp);
static void Slave_SendResponse(const Proto_Packet *resp, uint8_t channel);
static uint32_t Slave_Mac32(const uint8_t *buf, uint8_t len);
static bool Slave_VerifyPollAuth(const Proto_Packet *req);
static uint8_t Slave_BuildPollDataBody(uint8_t *out, uint8_t out_size);
static bool Slave_VerifyCmdAuthAndStoreNonce(const Proto_Packet *req);
static uint8_t Slave_BuildStreamDataBody(uint8_t *out, uint8_t out_size);
static bool Slave_SendStreamFragments(uint8_t channel);

/* ========================================================================== */
/* EXTI callback - CC2500 GDO0 wake-up interrupt                              */
/* ========================================================================== */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == g_cc2500.gdo0_pin) {
        g_wakeup_flag = true;
    }
    if (GPIO_Pin == g_cc1101.gdo0_pin) {
        g_cc1101_gdo0_flag = true;
    }
}

/* ========================================================================== */
/* Hardware initialization                                                     */
/* ========================================================================== */

static void Slave_InitHardware(void)
{
    CC1101_DWT_DelayInit();

    /* CC1101 handle (400 MHz MICS) - initially not used */
    g_cc1101.hspi      = &hspi1;
    g_cc1101.cs_port   = GPIOA;
    g_cc1101.cs_pin    = GPIO_PIN_4;
    g_cc1101.miso_port = GPIOA;
    g_cc1101.miso_pin  = GPIO_PIN_6;
    g_cc1101.gdo0_port = GPIOB;
    g_cc1101.gdo0_pin  = GPIO_PIN_0;

    /* CC2500 handle (2.4 GHz wake-up) */
    g_cc2500.hspi      = &hspi2;
    g_cc2500.cs_port   = GPIOB;
    g_cc2500.cs_pin    = GPIO_PIN_12;
    g_cc2500.miso_port = GPIOB;
    g_cc2500.miso_pin  = GPIO_PIN_14;
    g_cc2500.gdo0_port = GPIOB;
    g_cc2500.gdo0_pin  = GPIO_PIN_1;

    /* Low-power handle */
    g_lp.hspi_cc1101 = &hspi1;
    g_lp.hspi_cc2500 = &hspi2;
    /* CC2500 GDO0 (PB1 / EXTI1): deep-sleep beacon wakeup */
    g_lp.gdo0_port   = g_cc2500.gdo0_port;
    g_lp.gdo0_pin    = g_cc2500.gdo0_pin;
    g_lp.gdo0_irqn   = EXTI1_IRQn;
    /* CC1101 GDO0 (PB0 / EXTI0): MICS session RX wakeup from Stop 2 */
    g_lp.cc1101_gdo0_port = g_cc1101.gdo0_port;
    g_lp.cc1101_gdo0_pin  = g_cc1101.gdo0_pin;
    g_lp.cc1101_gdo0_irqn = EXTI0_IRQn;
    g_lp.hrtc         = &hrtc;

    LP_Init(&g_lp);

    /* Use lowest voltage scaling for minimum power */
    LP_SetVoltageScaling_LowPower();

    /* Initialize CC2500 for WOR wake-up mode */
    CC2500_InitWakeUp26MHz(&g_cc2500);
}

static uint32_t Slave_Mac32(const uint8_t *buf, uint8_t len)
{
    uint32_t h = 2166136261UL;
    for (uint8_t i = 0U; i < sizeof(g_auth_key); i++) {
        h ^= g_auth_key[i];
        h *= 16777619UL;
    }
    for (uint8_t i = 0U; i < len; i++) {
        h ^= buf[i];
        h *= 16777619UL;
    }
    return h;
}

static bool Slave_VerifyCmdAuthAndStoreNonce(const Proto_Packet *req)
{
    if (req->payload_len < (uint8_t)(PROTO_CMD_ARGS_OFFSET + AUTH_ARG_LEN)) {
        return false;
    }

    const uint8_t *nonce = &req->payload[PROTO_CMD_ARGS_OFFSET];
    const uint8_t *tag_b = &req->payload[PROTO_CMD_ARGS_OFFSET + AUTH_NONCE_LEN];

    uint8_t mac_input[1U + PROTO_DEVICE_ID_LEN + PROTO_MASTER_ID_LEN + AUTH_NONCE_LEN];
    mac_input[0] = g_session_id;
    memcpy(&mac_input[1], g_my_id, PROTO_DEVICE_ID_LEN);
    memcpy(&mac_input[1 + PROTO_DEVICE_ID_LEN], g_authorized_master_id, PROTO_MASTER_ID_LEN);
    memcpy(&mac_input[1 + PROTO_DEVICE_ID_LEN + PROTO_MASTER_ID_LEN], nonce, AUTH_NONCE_LEN);

    uint32_t expect = Slave_Mac32(mac_input, sizeof(mac_input));
    uint32_t got = ((uint32_t)tag_b[0]) |
                   ((uint32_t)tag_b[1] << 8) |
                   ((uint32_t)tag_b[2] << 16) |
                   ((uint32_t)tag_b[3] << 24);
    if (expect != got) {
        return false;
    }

    memcpy(g_session_nonce, nonce, AUTH_NONCE_LEN);
    return true;
}

static uint8_t Slave_BuildPollDataBody(uint8_t *out, uint8_t out_size)
{
    uint8_t body_len = (uint8_t)sizeof(g_sensor_data);
    uint8_t total = (uint8_t)(AUTH_ARG_LEN + body_len);
    if ((out == NULL) || (out_size < total)) {
        return 0U;
    }

    memcpy(out, g_session_nonce, AUTH_NONCE_LEN);

    uint8_t mac_input[1U + PROTO_DEVICE_ID_LEN + PROTO_MASTER_ID_LEN + AUTH_NONCE_LEN];
    mac_input[0] = g_session_id;
    memcpy(&mac_input[1], g_my_id, PROTO_DEVICE_ID_LEN);
    memcpy(&mac_input[1 + PROTO_DEVICE_ID_LEN], g_authorized_master_id, PROTO_MASTER_ID_LEN);
    memcpy(&mac_input[1 + PROTO_DEVICE_ID_LEN + PROTO_MASTER_ID_LEN], g_session_nonce, AUTH_NONCE_LEN);
    uint32_t tag = Slave_Mac32(mac_input, sizeof(mac_input));
    out[4] = (uint8_t)(tag & 0xFFU);
    out[5] = (uint8_t)((tag >> 8) & 0xFFU);
    out[6] = (uint8_t)((tag >> 16) & 0xFFU);
    out[7] = (uint8_t)((tag >> 24) & 0xFFU);

    memcpy(&out[AUTH_ARG_LEN], g_sensor_data, body_len);
    return total;
}

static uint8_t Slave_BuildStreamDataBody(uint8_t *out, uint8_t out_size)
{
    const uint8_t stream_meta_len = 1U; /* stream_seq */
    uint8_t body_len = (uint8_t)sizeof(g_sensor_data);
    uint8_t total = (uint8_t)(AUTH_ARG_LEN + stream_meta_len + body_len);
    if ((out == NULL) || (out_size < total)) {
        return 0U;
    }

    memcpy(out, g_session_nonce, AUTH_NONCE_LEN);

    uint8_t mac_input[1U + PROTO_DEVICE_ID_LEN + PROTO_MASTER_ID_LEN + AUTH_NONCE_LEN];
    mac_input[0] = g_session_id;
    memcpy(&mac_input[1], g_my_id, PROTO_DEVICE_ID_LEN);
    memcpy(&mac_input[1 + PROTO_DEVICE_ID_LEN], g_authorized_master_id, PROTO_MASTER_ID_LEN);
    memcpy(&mac_input[1 + PROTO_DEVICE_ID_LEN + PROTO_MASTER_ID_LEN], g_session_nonce, AUTH_NONCE_LEN);
    uint32_t tag = Slave_Mac32(mac_input, sizeof(mac_input));
    out[4] = (uint8_t)(tag & 0xFFU);
    out[5] = (uint8_t)((tag >> 8) & 0xFFU);
    out[6] = (uint8_t)((tag >> 16) & 0xFFU);
    out[7] = (uint8_t)((tag >> 24) & 0xFFU);

    out[AUTH_ARG_LEN] = g_stream_seq++;
    memcpy(&out[AUTH_ARG_LEN + stream_meta_len], g_sensor_data, body_len);
    return total;
}

static bool Slave_SendStreamFragments(uint8_t channel)
{
    uint8_t logical[STREAM_LOGICAL_DATA_MAX];
    logical[0] = g_stream_seq++;
    logical[1] = PROTO_APP_TYPE_WAVEFORM;
    logical[2] = PROTO_WAVEFORM_ECG;
    logical[3] = (uint8_t)sizeof(g_sensor_data); /* raw sample block length */
    memcpy(&logical[4], g_sensor_data, sizeof(g_sensor_data));

    uint8_t logical_len = (uint8_t)(4U + sizeof(g_sensor_data));
    if (logical_len > STREAM_LOGICAL_DATA_MAX) {
        return false;
    }

    for (uint8_t copy_idx = 0U; copy_idx < STREAM_FEC_COPIES; copy_idx++) {
        uint8_t body[AUTH_ARG_LEN + 4U + STREAM_LOGICAL_DATA_MAX];
        memcpy(body, g_session_nonce, AUTH_NONCE_LEN);
        uint8_t mac_input[1U + PROTO_DEVICE_ID_LEN + PROTO_MASTER_ID_LEN + AUTH_NONCE_LEN];
        mac_input[0] = g_session_id;
        memcpy(&mac_input[1], g_my_id, PROTO_DEVICE_ID_LEN);
        memcpy(&mac_input[1 + PROTO_DEVICE_ID_LEN], g_authorized_master_id, PROTO_MASTER_ID_LEN);
        memcpy(&mac_input[1 + PROTO_DEVICE_ID_LEN + PROTO_MASTER_ID_LEN], g_session_nonce, AUTH_NONCE_LEN);
        uint32_t tag = Slave_Mac32(mac_input, sizeof(mac_input));
        body[4] = (uint8_t)(tag & 0xFFU);
        body[5] = (uint8_t)((tag >> 8) & 0xFFU);
        body[6] = (uint8_t)((tag >> 16) & 0xFFU);
        body[7] = (uint8_t)((tag >> 24) & 0xFFU);
        body[AUTH_ARG_LEN] = logical[0]; /* stream_seq */
        body[AUTH_ARG_LEN + 1U] = copy_idx;
        body[AUTH_ARG_LEN + 2U] = STREAM_FEC_COPIES;
        body[AUTH_ARG_LEN + 3U] = logical_len;
        memcpy(&body[AUTH_ARG_LEN + 4U], logical, logical_len);

        Proto_Packet pkt;
        pkt.fcf = Proto_FCF_Make(MICS_FCF_TYPE_DATA, false, true);
        pkt.seq = g_seq++;
        pkt.sess = g_session_id;
        pkt.payload_len = Proto_BuildDataPayload(MICS_DTYPE_ECG_DELTA,
                                                 g_my_id,
                                                 g_authorized_master_id,
                                                 body,
                                                 (uint8_t)(AUTH_ARG_LEN + 4U + logical_len),
                                                 pkt.payload,
                                                 sizeof(pkt.payload));
        if (pkt.payload_len == 0U) {
            return false;
        }
        Slave_SendResponse(&pkt, channel);
    }

    return true;
}

/* ========================================================================== */
/* Deep sleep entry (main idle state)                                          */
/* ========================================================================== */

static void Slave_EnterDeepSleep(void)
{
    g_slave_pwr_trace.stop2_entries++;
    g_slave_pwr_trace.last_stop2_enter_ms = HAL_GetTick();

    /* Ensure CC1101 is in sleep mode */
    CC1101_EnterSleep(&g_cc1101);

    /* If CC2500 was put into SPWD during MICS session, wake it first
     * so WOR strobes/register accesses are valid. */
    (void)CC2500_WakeFromSleep(&g_cc2500);

    /* Put CC2500 into WOR mode for wake-up monitoring */
    CC2500_EnterWOR(&g_cc2500);

    /* Configure RTC for periodic self-check */
    LP_ConfigureRTCWakeUp(&g_lp, SLAVE_RTC_WAKEUP_INTERVAL_S);

    /* Enter Stop 2 - MCU draws ~2 uA
     * Wake-up sources:
     *   - CC2500 GDO0 EXTI (beacon received)
     *   - RTC wake-up timer (periodic self-check)
     */
    LP_WakeupSource src = LP_EnterStop2(&g_lp);

    /* --- MCU wakes up here --- */
    g_slave_pwr_trace.last_stop2_exit_ms = HAL_GetTick();

    if (src & LP_WAKEUP_EXTI_GDO0) {
        g_wakeup_flag = true;
        g_slave_pwr_trace.wakeup_exti_count++;
    }

    if (src & LP_WAKEUP_RTC_ALARM) {
        g_slave_pwr_trace.wakeup_rtc_count++;
        /* Periodic self-check: battery level, sensor status, etc.
         * If nothing to do, go back to sleep immediately */
        g_sensor_data[8] = (uint8_t)(HAL_GetTick() & 0xFFU);  /* timestamp */
    }
}

/* ========================================================================== */
/* Phase 1: Handle CC2500 wake-up beacon                                       */
/* ========================================================================== */

static int Slave_HandleWakeUp(void)
{
    uint8_t rx_buf[CC2500_PKT_MAX_LEN];
    uint8_t rx_len = 0U;

    /* 1) Read the received beacon packet from CC2500 */
    CC2500_Status st = CC2500_ReadPacket(&g_cc2500, rx_buf, &rx_len, sizeof(rx_buf));
    if (st != CC2500_OK) {
        return -1;
    }

    /* 2) Parse MAC PDU */
    Proto_Packet beacon;
    if (Proto_ParsePacket(rx_buf, rx_len, &beacon) < 0) {
        return -2;
    }

    /* 3) Validate FCF: must be COMMAND type, ACK_REQ set */
    if (Proto_FCF_Type(beacon.fcf) != MICS_FCF_TYPE_COMMAND) {
        return -3;
    }
    if (!Proto_FCF_IsAckReq(beacon.fcf)) {
        return -3;
    }

    /* 4) Validate payload layout (13 B):
     *    [0]     DTYPE_COMMAND  [1]      CMD_WAKEUP_REQ
     *    [2..5]  DEVICE_ID      [6..9]   MASTER_ID
     *    [10]    SESS           [11]     CHANNEL      [12] flags
     */
    if (beacon.payload_len < PROTO_BEACON_PAYLOAD_LEN) {
        return -4;
    }
    if (beacon.payload[PROTO_BEACON_DTYPE_OFFSET] != MICS_DTYPE_COMMAND) {
        return -5;
    }
    if (beacon.payload[PROTO_BEACON_CMD_OFFSET] != CMD_WAKEUP_REQ) {
        return -6;
    }

    /* 5) Verify this beacon is addressed to us (or broadcast) */
    const uint8_t *target_id = &beacon.payload[PROTO_BEACON_DEVID_OFFSET];
    if (!Proto_MatchDeviceID(target_id, g_my_id)) {
        return -7;
    }

    /* 6) Verify MASTER_ID - reject beacons from unauthorised Masters.
     *    Without this check, anyone learning the Slave's DEVICE_ID could
     *    wake the implant and hold a session. */
    if (!Proto_VerifyPayloadMasterID(&beacon,
                                     PROTO_BEACON_MASTERID_OFFSET,
                                     g_authorized_master_id)) {
        return -8;  /* silent drop - unknown Master */
    }

    /* 7) Remember the SESS assigned by Master */
    uint8_t assigned_sess = beacon.payload[PROTO_BEACON_SESS_OFFSET];
    if ((assigned_sess == PROTO_SESS_UNASSIGNED) ||
        (assigned_sess == PROTO_SESS_BROADCAST)) {
        return -9; /* invalid SESS allocation */
    }

    /* 8) Remember the MICS channel the Master pre-selected via LBT.
     *    This removes the need for the Slave to scan channels 0..9 on 400 MHz,
     *    which saves significant battery on the implant. */
    uint8_t assigned_channel = beacon.payload[PROTO_BEACON_CHANNEL_OFFSET];
    if (assigned_channel >= CC1101_MICS_NUM_CHANNELS) {
        return -10; /* Master must pre-select a valid MICS channel */
    }

    g_session_id     = assigned_sess;
    g_active_channel = assigned_channel;
    g_last_rx_seq_valid = false;
    memset(g_session_nonce, 0, sizeof(g_session_nonce));

    /* 9) Build and send wake-up ACK: echo DEVICE_ID + MASTER_ID +
     *    SESS + CHANNEL so Master can verify bilateral binding. */
    Proto_Packet ack;
    ack.fcf  = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, false, false);
    ack.seq  = g_seq++;
    ack.sess = PROTO_SESS_BROADCAST;  /* session not yet bilaterally confirmed */
    ack.payload_len = Proto_BuildBeaconPayload(CMD_WAKEUP_ACK,
                                               g_my_id,
                                               g_authorized_master_id,
                                               g_session_id,
                                               g_active_channel,
                                               0U,
                                               ack.payload,
                                               sizeof(ack.payload));
    if (ack.payload_len == 0U) {
        return -11;
    }

    uint8_t tx_buf[CC2500_PKT_MAX_LEN];
    uint8_t tx_len = Proto_BuildPacket(&ack, tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return -12;
    }

    st = CC2500_SendPacketDirect(&g_cc2500, CC2500_WAKEUP_CHANNEL,
                                 tx_buf, tx_len, 100U);
    if (st != CC2500_OK) {
        return -13;
    }

    return 0;
}

/* ========================================================================== */
/* Phase 2: MICS data session                                                  */
/* ========================================================================== */

/* Build a COMMAND-frame response carrying
 *   [DTYPE, sub_cmd, g_my_id, g_authorized_master_id, args...] */
static void slave_build_cmd_response(Proto_Packet *resp,
                                     uint8_t sub_cmd,
                                     const uint8_t *args, uint8_t args_len)
{
    resp->fcf = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, false, false);
    resp->payload_len = Proto_BuildCommandPayload(sub_cmd, g_my_id,
                                                  g_authorized_master_id,
                                                  args, args_len,
                                                  resp->payload,
                                                  sizeof(resp->payload));
}

static int Slave_ProcessCommand(const Proto_Packet *req, Proto_Packet *resp)
{
    /* Session is already validated by caller; just echo SESS */
    resp->seq  = req->seq;
    resp->sess = g_session_id;

    /* All inbound COMMAND packets: payload = [DTYPE, SUBCMD, DEVID[4], args] */
    if (req->payload_len < PROTO_CMD_HEADER_LEN) {
        uint8_t st = PROTO_STATUS_ERR_UNKNOWN_CMD;
        slave_build_cmd_response(resp, CMD_DATA_ACK, &st, 1U);
        return 0;
    }

    uint8_t sub_cmd = req->payload[PROTO_CMD_SUBCMD_OFFSET];

    switch (sub_cmd) {
    case CMD_POLL_REQ:
    {
        if (!Slave_VerifyCmdAuthAndStoreNonce(req)) {
            uint8_t st = PROTO_STATUS_ERR_ID_MISMATCH;
            slave_build_cmd_response(resp, CMD_DATA_ACK, &st, 1U);
            break;
        }

        uint8_t auth_body[AUTH_ARG_LEN + sizeof(g_sensor_data)];
        uint8_t auth_body_len = Slave_BuildPollDataBody(auth_body, sizeof(auth_body));
        if (auth_body_len == 0U) {
            uint8_t st = PROTO_STATUS_ERR_BUSY;
            slave_build_cmd_response(resp, CMD_DATA_ACK, &st, 1U);
            break;
        }

        /* Respond with DATA-type frame:
         *   [DTYPE_ECG_DELTA, g_my_id, g_authorized_master_id, sensor...]
         * DEVICE_ID proves "I am the implant you woke up";
         * MASTER_ID echo proves "I am still bound to this authorised Master". */
        resp->fcf = Proto_FCF_Make(MICS_FCF_TYPE_DATA, false, false);
        resp->payload_len = Proto_BuildDataPayload(MICS_DTYPE_ECG_DELTA,
                                                   g_my_id,
                                                   g_authorized_master_id,
                                                   auth_body,
                                                   auth_body_len,
                                                   resp->payload,
                                                   sizeof(resp->payload));
        break;
    }

    case CMD_STREAM_START:
    {
        if (!Slave_VerifyCmdAuthAndStoreNonce(req)) {
            uint8_t st = PROTO_STATUS_ERR_ID_MISMATCH;
            slave_build_cmd_response(resp, CMD_STREAM_ACK, &st, 1U);
            break;
        }

        if (req->payload_len < (uint8_t)(PROTO_CMD_ARGS_OFFSET + AUTH_ARG_LEN + 2U)) {
            uint8_t st = PROTO_STATUS_ERR_UNKNOWN_CMD;
            slave_build_cmd_response(resp, CMD_STREAM_ACK, &st, 1U);
            break;
        }

        g_stream_interval_ms = req->payload[PROTO_CMD_ARGS_OFFSET + AUTH_ARG_LEN];
        g_stream_remaining_frames = req->payload[PROTO_CMD_ARGS_OFFSET + AUTH_ARG_LEN + 1U];
        if (g_stream_interval_ms == 0U) {
            g_stream_interval_ms = 20U;
        }
        g_stream_enabled = (g_stream_remaining_frames > 0U);
        g_stream_seq = 0U;

        uint8_t st = PROTO_STATUS_OK;
        slave_build_cmd_response(resp, CMD_STREAM_ACK, &st, 1U);
        break;
    }

    case CMD_STREAM_STOP:
    {
        g_stream_enabled = false;
        g_stream_remaining_frames = 0U;
        uint8_t st = PROTO_STATUS_OK;
        slave_build_cmd_response(resp, CMD_STREAM_ACK, &st, 1U);
        break;
    }

    case CMD_STREAM_NACK:
    {
        /* Optional/low-frequency path: default ACK-only profile keeps
         * behavior simple and power-efficient. We only acknowledge receipt. */
        uint8_t st = PROTO_STATUS_OK;
        slave_build_cmd_response(resp, CMD_STREAM_ACK, &st, 1U);
        break;
    }

    case CMD_DATA_WRITE:
    {
        /* DATA_WRITE args profile (runtime stream config):
         *   args[0] = STREAM_CFG_SUBCMD (0x01)
         *   args[1] = interval_ms
         *   args[2] = frame_count
         */
        uint8_t status = PROTO_STATUS_ERR_UNKNOWN_CMD;
        if (req->payload_len >= (uint8_t)(PROTO_CMD_ARGS_OFFSET + 3U)) {
            const uint8_t *args = &req->payload[PROTO_CMD_ARGS_OFFSET];
            if (args[0] == STREAM_CFG_SUBCMD) {
                uint8_t iv = args[1];
                uint8_t fc = args[2];
                g_stream_interval_ms = (iv == 0U) ? 20U : iv;
                g_stream_remaining_frames = fc;
                status = PROTO_STATUS_OK;
            }
        }
        slave_build_cmd_response(resp, CMD_DATA_ACK, &status, 1U);
        break;
    }

    case CMD_SLEEP_CMD:
        slave_build_cmd_response(resp, CMD_SLEEP_ACK, NULL, 0U);
        break;

    case CMD_KEEPALIVE:
        slave_build_cmd_response(resp, CMD_KEEPALIVE_ACK, NULL, 0U);
        break;

    default:
    {
        uint8_t st = PROTO_STATUS_ERR_UNKNOWN_CMD;
        slave_build_cmd_response(resp, CMD_DATA_ACK, &st, 1U);
        break;
    }
    }

    return 0;
}

static void Slave_SendResponse(const Proto_Packet *resp, uint8_t channel)
{
    uint8_t tx_buf[CC1101_PKT_MAX_LEN];
    uint8_t tx_len = Proto_BuildPacket(resp, tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return;
    }

    /* Slave responds on the channel Master selected (Master already cleared
     * it via LBT). Use SendPacketLBT with threshold=-120 dBm so the CCA
     * always passes (effectively direct send). */
    (void)CC1101_SendPacketLBT(&g_cc1101, channel,
                               tx_buf, tx_len,
                               -120, 1U, 200U);
}

/* Validate and dispatch one received MICS command packet.
 * Returns true if the session should terminate (SLEEP_CMD received). */
static bool Slave_HandleRxPacket(const uint8_t *rx_buf, uint8_t rx_len)
{
    Proto_Packet req_pkt, resp_pkt;

    if (Proto_ParsePacket(rx_buf, rx_len, &req_pkt) < 0) {
        return false;
    }
    if (!Proto_MatchSession(req_pkt.sess, g_session_id)) {
        return false;
    }
    if (g_last_rx_seq_valid && !Proto_IsSeqNewer(req_pkt.seq, g_last_rx_seq)) {
        return false; /* replay/duplicate/out-of-order */
    }
    g_last_rx_seq = req_pkt.seq;
    g_last_rx_seq_valid = true;

    if (Proto_FCF_Type(req_pkt.fcf) != MICS_FCF_TYPE_COMMAND) {
        return false;
    }
    if ((req_pkt.payload_len < PROTO_CMD_HEADER_LEN) ||
        (req_pkt.payload[PROTO_CMD_DTYPE_OFFSET] != MICS_DTYPE_COMMAND)) {
        return false;
    }

    /* Triple-verify the handoff from 2.4 GHz wake-up:
     *   - SESS (already matched above)
     *   - DEVICE_ID in payload offset 2..5
     *   - MASTER_ID in payload offset 6..9
     * DEVICE_ID proves the Master still knows whom it woke up.
     * MASTER_ID proves this is the same authorised Master that
     * initiated the wake-up (and not a concurrent Master guessing
     * the SESS). Silently drop unauthenticated COMMAND frames so
     * the attacker gains no side-channel signal. */
    if (!Proto_VerifyPayloadDeviceID(&req_pkt,
                                     PROTO_CMD_DEVID_OFFSET,
                                     g_my_id)) {
        return false;
    }
    if (!Proto_VerifyPayloadMasterID(&req_pkt,
                                     PROTO_CMD_MASTERID_OFFSET,
                                     g_authorized_master_id)) {
        return false;
    }

    Slave_ProcessCommand(&req_pkt, &resp_pkt);
    Slave_SendResponse(&resp_pkt, g_active_channel);

    return ((req_pkt.payload_len >= PROTO_CMD_HEADER_LEN) &&
            (req_pkt.payload[PROTO_CMD_SUBCMD_OFFSET] == CMD_SLEEP_CMD));
}

static int Slave_HandleMICSSession(void)
{
    uint8_t rx_buf[CC1101_PKT_MAX_LEN];
    uint8_t rx_len = 0U;
    bool session_active = true;

    g_slave_pwr_trace.mics_session_count++;
    g_slave_pwr_trace.last_session_start_ms = HAL_GetTick();

    /* Guard: a valid channel must have been assigned in the wake-up beacon */
    if (g_active_channel >= CC1101_MICS_NUM_CHANNELS) {
        return -1;
    }

    /* Initialize CC1101 for MICS communication */
    CC1101_WakeFromSleep(&g_cc1101);
    CC1101_InitMICSLike26MHz(&g_cc1101);

    /* CC2500 is not needed during active 400 MHz session.
     * Put it into deep sleep to cut parallel radio current. */
    (void)CC2500_EnterSleep(&g_cc2500);

    while (session_active) {
        if (g_stream_enabled) {
            /*
             * Stream mode with Stop 2 + ms-precision RTC wakeup.
             *
             * Problem with HAL_GetTick(): SysTick stops in Stop 2, so ms
             * timing cannot rely on it.  HAL_GetTick() and busy-polling are
             * therefore replaced here too.
             *
             * Solution — LP_ConfigureRTCWakeUp_ms():
             *   RTCCLK/16 = 32768/16 = 2048 Hz → ~488 µs/tick.
             *   stream_interval_ms=20 → count=41 ticks → 41/2048 ≈ 20 ms.
             *   MCU sleeps in Stop 2 for exactly one stream period per loop
             *   iteration instead of spinning at 4 MHz for 50 ms windows.
             *
             * Wakeup semantics per iteration:
             *   RTC fires (g_cc1101_gdo0_flag==false): stream interval elapsed
             *     → send stream fragments; CC1101 auto-returns to RX (MCSM1
             *     TXOFF→RX) so no explicit CC1101_EnterRx is needed mid-session.
             *   CC1101 GDO0 fires (g_cc1101_gdo0_flag==true): command arrived
             *     mid-stream → read and process; both events can fire in the
             *     same wakeup (RTC during TX completion window).
             *
             * Session timeout: accumulated elapsed ms replaces HAL_GetTick().
             */
            uint32_t stream_elapsed_ms = 0U;

            while (g_stream_enabled && session_active) {
                LP_ConfigureRTCWakeUp_ms(&g_lp, (uint32_t)g_stream_interval_ms);

                if (CC1101_EnterRx(&g_cc1101, g_active_channel) != CC1101_OK) {
                    session_active = false;
                    break;
                }

                g_cc1101_gdo0_flag = false;
                (void)LP_EnterStop2(&g_lp);
                /* --- ISRs have fired; SPI restored --- */

                /* Advance software timeout counter by one stream period */
                stream_elapsed_ms += (uint32_t)g_stream_interval_ms;
                if (stream_elapsed_ms > PROTO_SESSION_TIMEOUT_MS) {
                    g_slave_pwr_trace.mics_timeout_count++;
                    session_active = false;
                    break;
                }

                if (!g_cc1101_gdo0_flag) {
                    /* Woke due to RTC: stream interval elapsed → transmit */
                    if (Slave_SendStreamFragments(g_active_channel)) {
                        if (g_stream_remaining_frames > 0U) {
                            g_stream_remaining_frames--;
                            if (g_stream_remaining_frames == 0U) {
                                g_stream_enabled = false;
                            }
                        }
                    }
                }

                if (g_cc1101_gdo0_flag) {
                    /* CC1101 received a command during stream */
                    g_cc1101_gdo0_flag = false;
                    if (CC1101_ReadPacket(&g_cc1101,
                                         rx_buf, &rx_len,
                                         sizeof(rx_buf)) == CC1101_OK) {
                        bool end = Slave_HandleRxPacket(rx_buf, rx_len);
                        if (end) {
                            session_active = false;
                        } else {
                            stream_elapsed_ms = 0U; /* valid traffic resets timeout */
                        }
                    }
                }
            }

        } else {
            /*
             * Idle command-wait mode: MCU enters Stop 2 with CC1101 in RX.
             * CC1101 GDO0 EXTI (asserts on CRC-OK packet) wakes the MCU to
             * read the FIFO — no busy-poll at 4 MHz between exchanges.
             * RTC alarm fires if no valid packet arrives within the session
             * timeout, replacing HAL_GetTick() which freezes in Stop 2.
             *
             * Architecture:
             *   1. Set RTC alarm = PROTO_SESSION_TIMEOUT_S
             *   2. CC1101_EnterRx → radio stays in RX while MCU sleeps
             *   3. LP_EnterStop2 → ~2 µA until GDO0 EXTI or RTC fires
             *   4a. CC1101 GDO0 (g_cc1101_gdo0_flag): read packet, process,
             *       reset RTC alarm for next timeout window
             *   4b. RTC alarm: session timed out, exit
             */
            LP_ConfigureRTCWakeUp(&g_lp, PROTO_SESSION_TIMEOUT_MS / 1000U);

            if (CC1101_EnterRx(&g_cc1101, g_active_channel) != CC1101_OK) {
                /* Radio failed to enter RX — abort session */
                break;
            }

            g_cc1101_gdo0_flag = false;
            (void)LP_EnterStop2(&g_lp);
            /* --- MCU wakes here (ISRs have already fired) --- */

            if (g_cc1101_gdo0_flag) {
                /* CC1101 received a packet: ISR callback set the flag. */
                g_cc1101_gdo0_flag = false;

                if (CC1101_ReadPacket(&g_cc1101,
                                     rx_buf, &rx_len,
                                     sizeof(rx_buf)) == CC1101_OK) {
                    bool end = Slave_HandleRxPacket(rx_buf, rx_len);
                    if (end) {
                        session_active = false;
                    }
                    /* On any valid traffic: RTC is reset at the top of the
                     * next iteration via LP_ConfigureRTCWakeUp. */
                }
            } else {
                /* No CC1101 packet flag set → woke due to RTC timeout
                 * (or a spurious wakeup from an unrelated source). */
                g_slave_pwr_trace.mics_timeout_count++;
                session_active = false;
            }
        }
    }

    LP_DisableRTCWakeUp(&g_lp);

    /* Invalidate session */
    g_session_id     = PROTO_SESS_UNASSIGNED;
    g_active_channel = PROTO_CHANNEL_UNASSIGNED;
    g_last_rx_seq_valid = false;
    memset(g_session_nonce, 0, sizeof(g_session_nonce));
    g_stream_enabled = false;
    g_stream_remaining_frames = 0U;
    g_slave_pwr_trace.last_session_end_ms = HAL_GetTick();
    return 0;
}

/* ========================================================================== */
/* main                                                                        */
/* ========================================================================== */

int main(void)
{
    HAL_Init();

    /* SystemClock_Config() - generated by CubeMX */
    /* MX_GPIO_Init(), MX_SPI1_Init(), MX_SPI2_Init(), MX_RTC_Init() */

    Slave_InitHardware();

    /* Put CC1101 to sleep at startup (not needed until communication) */
    CC1101_EnterSleep(&g_cc1101);

    while (1) {
        /* Enter deep sleep - this is the primary state
         * MCU + CC1101 in deep sleep
         * CC2500 in WOR mode, sniffing for wake-up beacons
         */
        Slave_EnterDeepSleep();

        /* Check if woken by CC2500 beacon */
        if (g_wakeup_flag) {
            g_wakeup_flag = false;

            /* Handle the wake-up beacon - parses and stores SESS */
            int rc = Slave_HandleWakeUp();

            if (rc == 0) {
                g_slave_pwr_trace.wakeup_ok_count++;
                /* Wake-up successful - enter MICS data session */
                Slave_HandleMICSSession();
            } else {
                g_slave_pwr_trace.wakeup_fail_count++;
            }

            /* Session done or wake-up failed - return to deep sleep */
            CC1101_EnterSleep(&g_cc1101);
        }

        /* If woken by RTC, the self-check was done in Slave_EnterDeepSleep.
         * Loop back to sleep. */
    }
}
