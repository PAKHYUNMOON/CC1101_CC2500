/*
 * Master (Programmer) - External Device Application
 * Ultra-low power operation: kimagu 2023.10.13
 *   1. MCU in Stop 2 mode (~2 uA)
 *   2. CC2500 in WOR mode (periodic RX sniff at 2.4 GHz)
 *   3. CC2500 GDO0 -> EXTI wakes MCU on beacon detection
 *   4. CC1101 is powered down until communication session
 *
 * Communication flow (LBT-first, same-channel retry):
 *   0. (internal) CC1101 runs LBT over MICS ch 0..9, picks a free channel N
 *   1. CC2500 sends wake-up beacon (2.4 GHz) carrying
 *      DEVICE_ID + MASTER_ID + SESS + CHANNEL=N
 *   2. CC2500 waits for wake-up ACK (Slave echoes DEVICE_ID + MASTER_ID +
 *      SESS + CHANNEL)
 *   3. CC1101 sends POLL / DATA commands on channel N with same-channel LBT
 *      retries (never hops - Slave is camped on N)
 *   4. CC1101 receives responses filtered by SESS + DEVICE_ID + MASTER_ID
 *   5. CC1101 sends SLEEP command to implant
 *   6. Both radios return to idle/sleep
 *
 * If same-channel LBT fails despite retries, the outer session-restart loop
 * re-runs step 0 and picks a new free channel.
 *
 * MCU: STM32U575 Q-series
 * Radios: CC2500 (2.4 GHz wake-up), CC1101 (400 MHz MICS data)
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
#define STREAM_USE_OPTIONAL_NACK 0U
#define STREAM_NACK_MIN_GAP 2U

/* ---------- hardware handles ---------- */
static CC1101_HandleTypeDef g_cc1101;
static CC2500_HandleTypeDef g_cc2500;
static LP_HandleTypeDef     g_lp;

/* These would be defined by CubeMX-generated code */
extern SPI_HandleTypeDef hspi1;  /* CC1101 SPI */
extern SPI_HandleTypeDef hspi2;  /* CC2500 SPI */
extern RTC_HandleTypeDef hrtc;

/* ---------- identity (factory-provisioned) ----------
 * g_master_id   : this Master's 4-byte authenticator. Sent on every beacon /
 *                 COMMAND / DATA frame. Slaves are paired to a specific
 *                 g_master_id at factory and reject frames that do not carry
 *                 the expected ID. This is a pre-shared identifier, NOT a
 *                 cryptographic key.
 * g_target_id   : DEVICE_ID of the implant we currently talk to.
 */
static const uint8_t g_master_id[PROTO_MASTER_ID_LEN] =
    {0xAA, 0xBB, 0xCC, 0xDD};
static uint8_t g_target_id[PROTO_DEVICE_ID_LEN] = {0x00, 0x00, 0x00, 0x01};

/* ---------- session state ---------- */
static uint8_t g_seq = 0U;
static uint8_t g_active_channel = 0U;
static uint8_t g_session_id     = PROTO_SESS_UNASSIGNED;
static uint8_t g_next_sess      = 1U;   /* rolling session ID generator */
static uint8_t g_last_rx_seq = 0U;
static bool    g_last_rx_seq_valid = false;
static uint8_t g_poll_nonce[AUTH_NONCE_LEN] = {0};
static uint8_t g_stream_interval_ms = 20U;
static uint8_t g_stream_frame_count = 4U;

static const uint8_t g_auth_key[16] = {
    0x3A, 0x5C, 0x19, 0xE7, 0xA2, 0x4D, 0x77, 0x10,
    0x91, 0x2B, 0xC4, 0x6E, 0x58, 0xFD, 0x03, 0xAB
};

/* ---------- LBT retry policy (Patch B) ----------
 * Once Master has pre-selected a channel via LBT and advertised it in the
 * 2.4 GHz wake-up beacon, the Slave is expected to reply on that exact
 * channel. If the channel becomes busy (another MICS user enters) we must
 * NOT hop - that would desync the Slave. Instead we retry CCA on the same
 * channel a bounded number of times with a short random back-off. If CCA
 * keeps failing we abort the TX and let the outer session-restart loop
 * re-run wake-up (with a fresh LBT channel selection).
 */
#define MASTER_SAME_CH_RETRIES      3U   /* total CCA attempts per TX */
#define MASTER_SAME_CH_BACKOFF_MS   5U   /* base back-off between CCA attempts */
#define MASTER_SESSION_RESTARTS     2U   /* outer wake-up retries on CCA fail */

/* ---------- forward declarations ---------- */
static void Master_InitHardware(void);
static int  Master_WakeUpImplant(void);
static int  Master_PollImplantData(uint8_t *resp_buf, uint8_t *resp_len);
static int  Master_WriteToImplant(const uint8_t *data, uint8_t len);
static int  Master_SendSleepCommand(void);
static int  Master_CommunicationSession(void);
static int  Master_StreamFromImplant(uint8_t interval_ms, uint8_t frame_count);
static int  Master_ConfigureStreamViaDataWrite(uint8_t interval_ms, uint8_t frame_count);
static int  Master_SendStreamNack(uint8_t missing_from_seq);
static CC1101_Status Master_TxLBT_SameChannel(const uint8_t *tx_buf,
                                              uint8_t tx_len);
static uint32_t Master_Mac32(const uint8_t *buf, uint8_t len);
static void Master_BuildPollAuthArgs(uint8_t sess, uint8_t out[AUTH_ARG_LEN]);
static bool Master_VerifyPollDataAuth(const Proto_Packet *pkt, uint8_t body_off);
static bool Master_VerifyStreamDataAuth(const Proto_Packet *pkt, uint8_t body_off);

/* ========================================================================== */
/* Hardware initialization                                                     */
/* ========================================================================== */

static void Master_InitHardware(void)
{
    /* Initialize DWT for microsecond delays */
    CC1101_DWT_DelayInit();

    /* CC1101 handle (400 MHz MICS) */
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
    g_lp.gdo0_port   = g_cc2500.gdo0_port;
    g_lp.gdo0_pin    = g_cc2500.gdo0_pin;
    g_lp.gdo0_irqn   = EXTI1_IRQn;
    g_lp.hrtc         = &hrtc;

    LP_Init(&g_lp);

    /* Initialize CC2500 in wake-up mode */
    CC2500_InitWakeUp26MHz(&g_cc2500);

    /* Initialize CC1101 in MICS mode */
    CC1101_InitMICSLike26MHz(&g_cc1101);
}

/* ========================================================================== */
/* Allocate next session ID (skips reserved values)                            */
/* ========================================================================== */
static uint8_t Master_AllocateSession(void)
{
    uint8_t s = g_next_sess++;
    if (g_next_sess == PROTO_SESS_BROADCAST) {
        g_next_sess = 1U; /* skip 0x00 and 0xFF */
    }
    if (s == PROTO_SESS_UNASSIGNED) {
        s = 1U;
    }
    return s;
}

static uint32_t Master_Mac32(const uint8_t *buf, uint8_t len)
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

static void Master_BuildPollAuthArgs(uint8_t sess, uint8_t out[AUTH_ARG_LEN])
{
    uint32_t t = HAL_GetTick() ^ (((uint32_t)g_seq) << 24) ^ ((uint32_t)sess << 16);
    g_poll_nonce[0] = (uint8_t)(t & 0xFFU);
    g_poll_nonce[1] = (uint8_t)((t >> 8) & 0xFFU);
    g_poll_nonce[2] = (uint8_t)((t >> 16) & 0xFFU);
    g_poll_nonce[3] = (uint8_t)((t >> 24) & 0xFFU);

    memcpy(out, g_poll_nonce, AUTH_NONCE_LEN);

    uint8_t mac_input[1U + PROTO_DEVICE_ID_LEN + PROTO_MASTER_ID_LEN + AUTH_NONCE_LEN];
    mac_input[0] = sess;
    memcpy(&mac_input[1], g_target_id, PROTO_DEVICE_ID_LEN);
    memcpy(&mac_input[1 + PROTO_DEVICE_ID_LEN], g_master_id, PROTO_MASTER_ID_LEN);
    memcpy(&mac_input[1 + PROTO_DEVICE_ID_LEN + PROTO_MASTER_ID_LEN], g_poll_nonce, AUTH_NONCE_LEN);

    uint32_t tag = Master_Mac32(mac_input, sizeof(mac_input));
    out[4] = (uint8_t)(tag & 0xFFU);
    out[5] = (uint8_t)((tag >> 8) & 0xFFU);
    out[6] = (uint8_t)((tag >> 16) & 0xFFU);
    out[7] = (uint8_t)((tag >> 24) & 0xFFU);
}

static bool Master_VerifyPollDataAuth(const Proto_Packet *pkt, uint8_t body_off)
{
    if (pkt->payload_len < (uint8_t)(body_off + AUTH_ARG_LEN)) {
        return false;
    }

    const uint8_t *nonce = &pkt->payload[body_off];
    const uint8_t *tag_b = &pkt->payload[body_off + AUTH_NONCE_LEN];

    if (memcmp(nonce, g_poll_nonce, AUTH_NONCE_LEN) != 0) {
        return false;
    }

    uint8_t mac_input[1U + PROTO_DEVICE_ID_LEN + PROTO_MASTER_ID_LEN + AUTH_NONCE_LEN];
    mac_input[0] = g_session_id;
    memcpy(&mac_input[1], g_target_id, PROTO_DEVICE_ID_LEN);
    memcpy(&mac_input[1 + PROTO_DEVICE_ID_LEN], g_master_id, PROTO_MASTER_ID_LEN);
    memcpy(&mac_input[1 + PROTO_DEVICE_ID_LEN + PROTO_MASTER_ID_LEN], nonce, AUTH_NONCE_LEN);

    uint32_t expect = Master_Mac32(mac_input, sizeof(mac_input));
    uint32_t got = ((uint32_t)tag_b[0]) |
                   ((uint32_t)tag_b[1] << 8) |
                   ((uint32_t)tag_b[2] << 16) |
                   ((uint32_t)tag_b[3] << 24);
    return expect == got;
}

static bool Master_VerifyStreamDataAuth(const Proto_Packet *pkt, uint8_t body_off)
{
    return Master_VerifyPollDataAuth(pkt, body_off);
}

static int Master_SendStreamNack(uint8_t missing_from_seq)
{
    uint8_t nack_args[1] = { missing_from_seq };
    Proto_Packet nack_pkt;
    uint8_t tx_buf[CC1101_PKT_MAX_LEN];

    nack_pkt.fcf  = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, true, false);
    nack_pkt.seq  = g_seq++;
    nack_pkt.sess = g_session_id;
    nack_pkt.payload_len = Proto_BuildCommandPayload(CMD_STREAM_NACK,
                                                     g_target_id,
                                                     g_master_id,
                                                     nack_args, (uint8_t)sizeof(nack_args),
                                                     nack_pkt.payload,
                                                     sizeof(nack_pkt.payload));
    if (nack_pkt.payload_len == 0U) {
        return -1;
    }
    uint8_t tx_len = Proto_BuildPacket(&nack_pkt, tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return -1;
    }
    return (Master_TxLBT_SameChannel(tx_buf, tx_len) == CC1101_OK) ? 0 : -2;
}

static int Master_ConfigureStreamViaDataWrite(uint8_t interval_ms, uint8_t frame_count)
{
    uint8_t cfg[3];
    cfg[0] = STREAM_CFG_SUBCMD;
    cfg[1] = interval_ms;
    cfg[2] = frame_count;
    return Master_WriteToImplant(cfg, (uint8_t)sizeof(cfg));
}

static int Master_StreamFromImplant(uint8_t interval_ms, uint8_t frame_count)
{
    if (frame_count == 0U) {
        return 0;
    }

    uint8_t start_args[AUTH_ARG_LEN + 2U];
    Master_BuildPollAuthArgs(g_session_id, start_args);
    start_args[AUTH_ARG_LEN] = interval_ms;
    start_args[AUTH_ARG_LEN + 1U] = frame_count;

    Proto_Packet start_pkt;
    uint8_t tx_buf[CC1101_PKT_MAX_LEN];
    uint8_t rx_buf[CC1101_PKT_MAX_LEN];
    uint8_t rx_len = 0U;

    start_pkt.fcf  = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, true, false);
    start_pkt.seq  = g_seq++;
    start_pkt.sess = g_session_id;
    start_pkt.payload_len = Proto_BuildCommandPayload(CMD_STREAM_START,
                                                      g_target_id,
                                                      g_master_id,
                                                      start_args, (uint8_t)sizeof(start_args),
                                                      start_pkt.payload,
                                                      sizeof(start_pkt.payload));
    if (start_pkt.payload_len == 0U) {
        return -1;
    }
    uint8_t tx_len = Proto_BuildPacket(&start_pkt, tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return -1;
    }
    if (Master_TxLBT_SameChannel(tx_buf, tx_len) != CC1101_OK) {
        return -2;
    }

    /* Wait command ACK for stream start */
    if (CC1101_WaitAndReadPacket(&g_cc1101, g_active_channel,
                                 rx_buf, &rx_len, sizeof(rx_buf),
                                 PROTO_MICS_RESP_TIMEOUT_MS) != CC1101_OK) {
        return -3;
    }

    Proto_Packet ack;
    if (Proto_ParsePacket(rx_buf, rx_len, &ack) < 0) {
        return -4;
    }
    if (!Proto_MatchSession(ack.sess, g_session_id)) {
        return -5;
    }
    if (g_last_rx_seq_valid && !Proto_IsSeqNewer(ack.seq, g_last_rx_seq)) {
        return -6;
    }
    g_last_rx_seq = ack.seq;
    g_last_rx_seq_valid = true;

    if (Proto_FCF_Type(ack.fcf) != MICS_FCF_TYPE_COMMAND) {
        return -7;
    }
    if (!Proto_VerifyPayloadDeviceID(&ack, PROTO_CMD_DEVID_OFFSET, g_target_id) ||
        !Proto_VerifyPayloadMasterID(&ack, PROTO_CMD_MASTERID_OFFSET, g_master_id)) {
        return -8;
    }
    if (ack.payload_len < (PROTO_CMD_ARGS_OFFSET + 1U) ||
        ack.payload[PROTO_CMD_SUBCMD_OFFSET] != CMD_STREAM_ACK ||
        ack.payload[PROTO_CMD_ARGS_OFFSET] != PROTO_STATUS_OK) {
        return -9;
    }

    /* Receive pushed stream DATA frames */
    uint8_t received = 0U;
    bool stream_seq_valid = false;
    uint8_t last_stream_seq = 0U;
    while (received < frame_count) {
        if (CC1101_WaitAndReadPacket(&g_cc1101, g_active_channel,
                                     rx_buf, &rx_len, sizeof(rx_buf),
                                     PROTO_MICS_RESP_TIMEOUT_MS) != CC1101_OK) {
            return -10;
        }
        Proto_Packet data_pkt;
        if (Proto_ParsePacket(rx_buf, rx_len, &data_pkt) < 0) {
            continue;
        }
        if (!Proto_MatchSession(data_pkt.sess, g_session_id)) {
            continue;
        }
        if (g_last_rx_seq_valid && !Proto_IsSeqNewer(data_pkt.seq, g_last_rx_seq)) {
            continue;
        }
        g_last_rx_seq = data_pkt.seq;
        g_last_rx_seq_valid = true;

        if (Proto_FCF_Type(data_pkt.fcf) != MICS_FCF_TYPE_DATA) {
            continue;
        }
        if (!Proto_VerifyPayloadDeviceID(&data_pkt, PROTO_DATA_DEVID_OFFSET, g_target_id) ||
            !Proto_VerifyPayloadMasterID(&data_pkt, PROTO_DATA_MASTERID_OFFSET, g_master_id)) {
            continue;
        }
        if (!Master_VerifyStreamDataAuth(&data_pkt, PROTO_DATA_BODY_OFFSET)) {
            continue;
        }
        /* DATA body: NONCE(4) + TAG(4) + stream_seq(1) + sensor... */
        if (data_pkt.payload_len < (uint8_t)(PROTO_DATA_BODY_OFFSET + AUTH_ARG_LEN + 1U)) {
            continue;
        }
        uint8_t stream_seq = data_pkt.payload[PROTO_DATA_BODY_OFFSET + AUTH_ARG_LEN];
        if (stream_seq_valid) {
            uint8_t expected = (uint8_t)(last_stream_seq + 1U);
            if (stream_seq != expected) {
#if STREAM_USE_OPTIONAL_NACK
                uint8_t gap = (uint8_t)(stream_seq - expected);
                if (gap >= STREAM_NACK_MIN_GAP) {
                    (void)Master_SendStreamNack(expected);
                }
#endif
            }
        }
        last_stream_seq = stream_seq;
        stream_seq_valid = true;
        received++;
    }

    return 0;
}

/* ========================================================================== */
/* Same-channel LBT helper (Patch B)                                            */
/*                                                                              */
/* Retries CCA on g_active_channel up to MASTER_SAME_CH_RETRIES times with a    */
/* small back-off between attempts. Unlike CC1101_SendPacketLBT_Agile this      */
/* NEVER hops to a different channel, so the Slave stays in sync with the       */
/* channel it was told to camp on in the wake-up beacon.                        */
/* Returns CC1101_OK on successful TX, CC1101_ERR_CCA if every attempt failed.  */
/* ========================================================================== */
static CC1101_Status Master_TxLBT_SameChannel(const uint8_t *tx_buf,
                                              uint8_t tx_len)
{
    CC1101_Status st = CC1101_ERR_CCA;
    for (uint8_t attempt = 0U; attempt < MASTER_SAME_CH_RETRIES; attempt++) {
        st = CC1101_SendPacketLBT(&g_cc1101,
                                  g_active_channel,
                                  tx_buf, tx_len,
                                  CC1101_MICS_LBT_THRESHOLD_DBM,
                                  CC1101_MICS_LBT_LISTEN_MS,
                                  200U);
        if (st == CC1101_OK) {
            return CC1101_OK;
        }
        if (st != CC1101_ERR_CCA) {
            /* Non-CCA failure (SPI / HW) - retry won't help */
            return st;
        }
        /* Pseudo-random back-off: mix seq counter with attempt index to
         * de-correlate colliding Masters. Bounded: [MASTER_SAME_CH_BACKOFF_MS
         * .. MASTER_SAME_CH_BACKOFF_MS + 7]. */
        uint32_t jitter = ((uint32_t)g_seq + attempt) & 0x07U;
        HAL_Delay(MASTER_SAME_CH_BACKOFF_MS + jitter);
    }
    return st;   /* CC1101_ERR_CCA */
}

/* ========================================================================== */
/* Phase 1: Wake-up via CC2500 (2.4 GHz)                                      */
/* ========================================================================== */

static int Master_WakeUpImplant(void)
{
    uint8_t tx_buf[CC2500_PKT_MAX_LEN];
    uint8_t rx_buf[CC2500_PKT_MAX_LEN];
    uint8_t rx_len = 0U;

    /* 1) Run LBT on CC1101 *before* sending the beacon so we can tell the
     *    Slave which MICS channel to tune to. This eliminates Slave-side
     *    channel scanning and its battery cost. */
    int8_t free_ch = CC1101_FindFreeChannel(&g_cc1101,
                                            CC1101_MICS_LBT_THRESHOLD_DBM,
                                            CC1101_MICS_LBT_LISTEN_MS);
    if (free_ch < 0) {
        return -1;   /* all MICS channels busy - retry later */
    }
    g_active_channel = (uint8_t)free_ch;

    /* 2) Propose a new SESS for this session */
    uint8_t proposed_sess = Master_AllocateSession();

    /* 3) Build beacon MAC PDU:
     *    FCF  = COMMAND | ACK_REQ
     *    SESS = BROADCAST (session not yet established)
     *    PAYLOAD[0]    = DTYPE_COMMAND
     *    PAYLOAD[1]    = CMD_WAKEUP_REQ
     *    PAYLOAD[2..5] = target DEVICE_ID
     *    PAYLOAD[6..9] = MASTER_ID (authenticator)
     *    PAYLOAD[10]   = proposed SESS
     *    PAYLOAD[11]   = assigned MICS channel (from LBT)
     *    PAYLOAD[12]   = flags
     */
    Proto_Packet beacon_pkt;
    beacon_pkt.fcf  = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, true, false);
    beacon_pkt.seq  = g_seq++;
    beacon_pkt.sess = PROTO_SESS_BROADCAST;
    beacon_pkt.payload_len = Proto_BuildBeaconPayload(CMD_WAKEUP_REQ,
                                                      g_target_id,
                                                      g_master_id,
                                                      proposed_sess,
                                                      g_active_channel,
                                                      0U,
                                                      beacon_pkt.payload,
                                                      sizeof(beacon_pkt.payload));
    if (beacon_pkt.payload_len == 0U) {
        return -2;
    }

    uint8_t tx_len = Proto_BuildPacket(&beacon_pkt, tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return -2;
    }

    /* 4) Repeatedly transmit the beacon so at least one WOR window catches it */
    CC2500_Status st = CC2500_SendWakeUpBeacon(&g_cc2500,
                                               tx_buf, tx_len,
                                               PROTO_WAKEUP_BEACON_REPEAT);
    if (st != CC2500_OK) {
        return -3;
    }

    /* 5) Wait for wake-up ACK on the same channel */
    st = CC2500_WaitAndReadPacket(&g_cc2500,
                                  CC2500_WAKEUP_CHANNEL,
                                  rx_buf, &rx_len,
                                  sizeof(rx_buf),
                                  PROTO_WAKEUP_ACK_TIMEOUT_MS);
    if (st != CC2500_OK) {
        return -4;
    }

    /* 6) Parse and verify ACK:
     *    - FCF type = COMMAND
     *    - payload[1]    = CMD_WAKEUP_ACK
     *    - payload[2..5] = our target DEVICE_ID
     *    - payload[6..9] = echoed MASTER_ID (must match g_master_id)
     *    - payload[10]   = echoed SESS (must match proposed_sess)
     *    - payload[11]   = echoed CHANNEL (must match g_active_channel)
     */
    Proto_Packet ack_pkt;
    if (Proto_ParsePacket(rx_buf, rx_len, &ack_pkt) < 0) {
        return -5;
    }
    if (Proto_FCF_Type(ack_pkt.fcf) != MICS_FCF_TYPE_COMMAND) {
        return -6;
    }
    if (ack_pkt.payload_len < PROTO_BEACON_PAYLOAD_LEN) {
        return -7;
    }
    if (ack_pkt.payload[PROTO_BEACON_CMD_OFFSET] != CMD_WAKEUP_ACK) {
        return -8;
    }
    if (!Proto_MatchDeviceID_Strict(&ack_pkt.payload[PROTO_BEACON_DEVID_OFFSET],
                                    g_target_id)) {
        return -9;
    }
    /* MASTER_ID echo proves the Slave really received OUR wake-up beacon
     * (and not a concurrent Master that happens to use the same DEVICE_ID). */
    if (!Proto_VerifyPayloadMasterID(&ack_pkt,
                                     PROTO_BEACON_MASTERID_OFFSET,
                                     g_master_id)) {
        return -10;
    }
    if (ack_pkt.payload[PROTO_BEACON_SESS_OFFSET] != proposed_sess) {
        return -11;
    }
    if (ack_pkt.payload[PROTO_BEACON_CHANNEL_OFFSET] != g_active_channel) {
        return -12;  /* Slave didn't acknowledge the assigned channel */
    }

    /* 7) Session established */
    g_session_id = proposed_sess;
    g_last_rx_seq_valid = false;
    return 0;
}

/* ========================================================================== */
/* Phase 2: Data communication via CC1101 (400 MHz MICS)                       */
/* ========================================================================== */

static int Master_PollImplantData(uint8_t *resp_buf, uint8_t *resp_len)
{
    Proto_Packet poll_pkt;
    uint8_t tx_buf[CC1101_PKT_MAX_LEN];
    uint8_t rx_buf[CC1101_PKT_MAX_LEN];
    uint8_t rx_len = 0U;

    /* Build POLL request:
     *   FCF  = COMMAND | ACK_REQ
     *   SESS = established session
     *   payload = [DTYPE_COMMAND, CMD_POLL_REQ, DEVICE_ID[4], MASTER_ID[4]]
     *   DEVICE_ID + MASTER_ID jointly prove to Slave that this Master is the
     *   same authenticated peer that handed off identity during the 2.4 GHz
     *   wake-up handshake.
     */
    poll_pkt.fcf  = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, true, false);
    poll_pkt.seq  = g_seq++;
    poll_pkt.sess = g_session_id;
    uint8_t auth_args[AUTH_ARG_LEN];
    Master_BuildPollAuthArgs(g_session_id, auth_args);
    poll_pkt.payload_len = Proto_BuildCommandPayload(CMD_POLL_REQ,
                                                     g_target_id,
                                                     g_master_id,
                                                     auth_args, AUTH_ARG_LEN,
                                                     poll_pkt.payload,
                                                     sizeof(poll_pkt.payload));
    if (poll_pkt.payload_len == 0U) {
        return -1;
    }

    uint8_t tx_len = Proto_BuildPacket(&poll_pkt, tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return -1;
    }

    /* Same-channel LBT: retry CCA on the pre-announced channel, do NOT hop.
     * If every retry fails, caller restarts the session (new LBT channel). */
    CC1101_Status st = Master_TxLBT_SameChannel(tx_buf, tx_len);
    if (st != CC1101_OK) {
        return -2;
    }

    /* Wait for response on the same channel */
    st = CC1101_WaitAndReadPacket(&g_cc1101,
                                  g_active_channel,
                                  rx_buf, &rx_len,
                                  sizeof(rx_buf),
                                  PROTO_MICS_RESP_TIMEOUT_MS);
    if (st != CC1101_OK) {
        return -3;
    }

    /* Parse response */
    Proto_Packet resp_pkt;
    if (Proto_ParsePacket(rx_buf, rx_len, &resp_pkt) < 0) {
        return -4;
    }
    if (!Proto_MatchSession(resp_pkt.sess, g_session_id)) {
        return -5;
    }
    if (g_last_rx_seq_valid && !Proto_IsSeqNewer(resp_pkt.seq, g_last_rx_seq)) {
        return -6; /* replay/duplicate/out-of-order */
    }
    g_last_rx_seq = resp_pkt.seq;
    g_last_rx_seq_valid = true;

    /* Verify DEVICE_ID + MASTER_ID in payload.
     * Triple-check (SESS + DEVICE_ID + MASTER_ID) confirms the response
     * originates from the Slave we woke up AND is bound to this Master. */
    uint8_t ftype = Proto_FCF_Type(resp_pkt.fcf);
    uint8_t devid_off, masterid_off, body_off;

    if (ftype == MICS_FCF_TYPE_DATA) {
        devid_off    = PROTO_DATA_DEVID_OFFSET;
        masterid_off = PROTO_DATA_MASTERID_OFFSET;
        body_off     = PROTO_DATA_BODY_OFFSET;
    } else if (ftype == MICS_FCF_TYPE_COMMAND) {
        devid_off    = PROTO_CMD_DEVID_OFFSET;
        masterid_off = PROTO_CMD_MASTERID_OFFSET;
        body_off     = PROTO_CMD_ARGS_OFFSET;
    } else {
        return -7;
    }

    if (!Proto_VerifyPayloadDeviceID(&resp_pkt, devid_off, g_target_id)) {
        return -8;  /* Device ID mismatch - possible spoofer or wrong Slave */
    }
    if (!Proto_VerifyPayloadMasterID(&resp_pkt, masterid_off, g_master_id)) {
        return -9;  /* Slave is echoing a different Master's ID - reject */
    }

    /* Copy application body to caller */
    if (resp_pkt.payload_len < body_off) {
        return -10;
    }
    if (ftype == MICS_FCF_TYPE_DATA && !Master_VerifyPollDataAuth(&resp_pkt, body_off)) {
        return -11;
    }
    if ((resp_buf != NULL) && (resp_len != NULL)) {
        uint8_t data_off = body_off;
        if (ftype == MICS_FCF_TYPE_DATA) {
            data_off = (uint8_t)(data_off + AUTH_ARG_LEN);
        }
        uint8_t n = (uint8_t)(resp_pkt.payload_len - data_off);
        memcpy(resp_buf, &resp_pkt.payload[data_off], n);
        *resp_len = n;
    }

    return 0;
}

static int Master_WriteToImplant(const uint8_t *data, uint8_t len)
{
    Proto_Packet write_pkt;
    uint8_t tx_buf[CC1101_PKT_MAX_LEN];
    uint8_t rx_buf[CC1101_PKT_MAX_LEN];
    uint8_t rx_len = 0U;

    /* Command payload overhead = DTYPE + SUBCMD + DEVID[4] + MASTERID[4] = 10 B */
    if ((data == NULL) || (len == 0U) ||
        (len > (PROTO_MAX_PAYLOAD - PROTO_CMD_HEADER_LEN))) {
        return -1;
    }

    /* Build DATA_WRITE packet with DEVICE_ID + MASTER_ID handoff */
    write_pkt.fcf  = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, true, false);
    write_pkt.seq  = g_seq++;
    write_pkt.sess = g_session_id;
    write_pkt.payload_len = Proto_BuildCommandPayload(CMD_DATA_WRITE,
                                                      g_target_id,
                                                      g_master_id,
                                                      data, len,
                                                      write_pkt.payload,
                                                      sizeof(write_pkt.payload));
    if (write_pkt.payload_len == 0U) {
        return -2;
    }

    uint8_t tx_len = Proto_BuildPacket(&write_pkt, tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return -2;
    }

    /* Same-channel LBT retry (never hop - Slave is camped on this channel) */
    CC1101_Status st = Master_TxLBT_SameChannel(tx_buf, tx_len);
    if (st != CC1101_OK) {
        return -3;
    }

    /* Wait for DATA_ACK */
    st = CC1101_WaitAndReadPacket(&g_cc1101,
                                  g_active_channel,
                                  rx_buf, &rx_len,
                                  sizeof(rx_buf),
                                  PROTO_MICS_RESP_TIMEOUT_MS);
    if (st != CC1101_OK) {
        return -4;
    }

    Proto_Packet ack_pkt;
    if (Proto_ParsePacket(rx_buf, rx_len, &ack_pkt) < 0) {
        return -5;
    }
    if (!Proto_MatchSession(ack_pkt.sess, g_session_id)) {
        return -6;
    }
    if (g_last_rx_seq_valid && !Proto_IsSeqNewer(ack_pkt.seq, g_last_rx_seq)) {
        return -7;
    }
    g_last_rx_seq = ack_pkt.seq;
    g_last_rx_seq_valid = true;
    if (Proto_FCF_Type(ack_pkt.fcf) != MICS_FCF_TYPE_COMMAND) {
        return -8;
    }
    /* Verify DEVICE_ID + MASTER_ID handoff in ACK */
    if (!Proto_VerifyPayloadDeviceID(&ack_pkt, PROTO_CMD_DEVID_OFFSET,
                                     g_target_id)) {
        return -8;
    }
    if (!Proto_VerifyPayloadMasterID(&ack_pkt, PROTO_CMD_MASTERID_OFFSET,
                                     g_master_id)) {
        return -9;
    }
    if (ack_pkt.payload_len < (PROTO_CMD_ARGS_OFFSET + 1U)) {
        return -10;
    }
    if (ack_pkt.payload[PROTO_CMD_SUBCMD_OFFSET] != CMD_DATA_ACK) {
        return -11;
    }
    if (ack_pkt.payload[PROTO_CMD_ARGS_OFFSET] != PROTO_STATUS_OK) {
        return -12;
    }

    return 0;
}

/* ========================================================================== */
/* Phase 3: End session                                                        */
/* ========================================================================== */

static int Master_SendSleepCommand(void)
{
    Proto_Packet sleep_pkt;
    uint8_t tx_buf[CC1101_PKT_MAX_LEN];
    uint8_t rx_buf[CC1101_PKT_MAX_LEN];
    uint8_t rx_len = 0U;

    sleep_pkt.fcf  = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, false, false);
    sleep_pkt.seq  = g_seq++;
    sleep_pkt.sess = g_session_id;
    sleep_pkt.payload_len = Proto_BuildCommandPayload(CMD_SLEEP_CMD,
                                                      g_target_id,
                                                      g_master_id,
                                                      NULL, 0U,
                                                      sleep_pkt.payload,
                                                      sizeof(sleep_pkt.payload));
    if (sleep_pkt.payload_len == 0U) {
        return -1;
    }

    uint8_t tx_len = Proto_BuildPacket(&sleep_pkt, tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return -1;
    }

    /* Same-channel LBT retry (no hop) */
    CC1101_Status st = Master_TxLBT_SameChannel(tx_buf, tx_len);
    if (st != CC1101_OK) {
        return -2;
    }

    /* Optionally wait for SLEEP_ACK */
    st = CC1101_WaitAndReadPacket(&g_cc1101,
                                  g_active_channel,
                                  rx_buf, &rx_len,
                                  sizeof(rx_buf),
                                  PROTO_SLEEP_CONFIRM_TIMEOUT_MS);
    /* Even if ACK times out, implant will auto-sleep */
    (void)st;

    /* Invalidate session */
    g_session_id = PROTO_SESS_UNASSIGNED;
    g_last_rx_seq_valid = false;

    return 0;
}

/* ========================================================================== */
/* Full communication session                                                  */
/* ========================================================================== */

static int Master_CommunicationSession(void)
{
    int rc = -1;
    uint8_t data_buf[PROTO_MAX_PAYLOAD];
    uint8_t data_len = 0U;

    /* Outer session-restart loop (Patch B).
     *
     * If same-channel LBT keeps failing on the MICS channel we announced
     * during wake-up (another user has occupied it after our CCA), the
     * Slave is camped on a dead channel. Re-run wake-up from scratch:
     * LBT will pick a NEW free channel, the beacon will advertise it,
     * and the Slave will retune. Bounded by MASTER_SESSION_RESTARTS.
     */
    for (uint8_t attempt = 0U; attempt <= MASTER_SESSION_RESTARTS; attempt++) {
        /* Start each retry from a clean session state. */
        g_session_id = PROTO_SESS_UNASSIGNED;

        /* Phase 1: Wake up implant via 2.4 GHz (with fresh LBT each pass) */
        rc = Master_WakeUpImplant();
        if (rc != 0) {
            continue; /* try again from wake-up - Slave may have missed beacon */
        }

        /* Phase 2: Poll data via 400 MHz MICS */
        rc = Master_PollImplantData(data_buf, &data_len);
        if (rc == -2) {
            /* CCA failure on announced channel: the channel was occupied
             * after our LBT. Restart the session so we re-run LBT and
             * pick a different channel. Slave will session-timeout or be
             * re-woken with the new channel. */
            g_session_id = PROTO_SESS_UNASSIGNED;
            continue;
        }
        if (rc != 0) {
            /* Non-CCA failure (parse, auth, timeout) - end session cleanly */
            Master_SendSleepCommand();
            break;
        }

        /* Process received data (application-specific) */
        /* ... */

        /* Runtime stream control path:
         * 1) push stream config via DATA_WRITE
         * 2) trigger stream start with configured values */
        (void)Master_ConfigureStreamViaDataWrite(g_stream_interval_ms, g_stream_frame_count);
        (void)Master_StreamFromImplant(g_stream_interval_ms, g_stream_frame_count);

        /* Phase 3: End session - send implant back to sleep */
        Master_SendSleepCommand();
        rc = 0;
        break;
    }

    /* Put radios to sleep regardless of outcome */
    CC1101_EnterSleep(&g_cc1101);
    CC2500_EnterSleep(&g_cc2500);

    return rc;
}

/* ========================================================================== */
/* main                                                                        */
/* ========================================================================== */

int main(void)
{
    HAL_Init();

    /* SystemClock_Config() - generated by CubeMX */
    /* MX_GPIO_Init(), MX_SPI1_Init(), MX_SPI2_Init(), MX_RTC_Init() */

    Master_InitHardware();

    while (1) {
        /*
         * Master (Programmer) is typically triggered by user action
         * (e.g., button press or host command).
         * Here we show a simple polling loop; in practice this would be
         * event-driven.
         */

        int result = Master_CommunicationSession();

        if (result == 0) {
            /* Session successful */
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  /* LED on */
        } else {
            /* Session failed */
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        }

        /* Wait before next session (or wait for user trigger) */
        HAL_Delay(5000U);
    }
}
