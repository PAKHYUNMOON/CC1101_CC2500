#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * MICS Implant Communication Protocol
 *
 * Dual-radio architecture:
 *   CC2500 (2.4 GHz) : Wake-up beacon / ACK only
 *   CC1101 (400 MHz) : MICS-band data communication with LBT
 *
 * Roles:
 *   Master (Programmer)  - Initiates all communication; performs LBT
 *   Slave  (Implant)     - Responds only; never self-initiates
 *
 * -------------------------------------------------------------------------
 * Unified MAC PDU (CC1101 전 구간 및 CC2500 Proto_BuildPacket 구간 공통):
 *
 *   +------+------+------+------+------------------+------+
 *   | LEN  | FCF  | SEQ  | SESS | PAYLOAD          | CRC  |
 *   | 1B   | 1B   | 1B   | 1B   | 0..58B (no FEC)  | 2B   |
 *   |      |      |      |      | 0..105B (w/FEC)  |      |
 *   +------+------+------+------+------------------+------+
 *         └─────── MAC HEADER (3B) ─────┘
 *   LEN and CRC are handled by CC1101/CC2500 hardware
 *   (PKTCTRL0.LENGTH_CONFIG=variable, CRC_EN=1).
 *
 *   Maximum LEN field = 61 (FCF+SEQ+SESS+PAYLOAD bytes after LEN; = PKTLEN).
 *   With PKTLEN=61 → PAYLOAD up to 58B (non-FEC).
 *   With FEC enabled and extended buffering → up to 105B application data.
 *
 *   DEVICE_ID is NOT carried in the MAC header.  During an established
 *   session, SESS (1B) uniquely identifies the link.  DEVICE_ID is carried
 *   INSIDE the PAYLOAD of the Wake-up beacon / ACK only, because SESS is
 *   not yet assigned at wake-up time.
 * ========================================================================== */

/* ---------- PDU sizing ---------- */
#define PROTO_HDR_SIZE               3U      /* FCF + SEQ + SESS */
#define PROTO_PKT_MAX_LEN            61U     /* must match CC1101/CC2500 PKTLEN */
#define PROTO_MAX_PAYLOAD            (PROTO_PKT_MAX_LEN - PROTO_HDR_SIZE)  /* 58 */
#define PROTO_MAX_PAYLOAD_FEC        105U    /* with FEC (application-layer) */

/* ==========================================================================
 * FCF (Frame Control Field) - 1 byte
 *
 *   Bit 7  (0x80) : MICS_FCF_ACK_REQ       ACK 요청 플래그
 *   Bit 6  (0x40) : MICS_FCF_FRAGMENT      Fragment 패킷 플래그
 *   Bit 5  (0x20) : Reserved
 *   Bit 4  (0x10) : Reserved
 *   Bit 3-0(0x0F) : Frame type
 * ========================================================================== */
#define MICS_FCF_ACK_REQ             0x80U
#define MICS_FCF_FRAGMENT            0x40U
#define MICS_FCF_RESERVED_5          0x20U
#define MICS_FCF_RESERVED_4          0x10U
#define MICS_FCF_TYPE_MASK           0x0FU

/* ---------- Frame types (Bit 3-0 of FCF) ---------- */
#define MICS_FCF_TYPE_DATA           0x00U   /* sensor data, telemetry */
#define MICS_FCF_TYPE_ACK            0x01U   /* acknowledgement frame  */
#define MICS_FCF_TYPE_COMMAND        0x02U   /* control command frame  */
/* 0x03 ~ 0x0F : reserved for future use */

/* ==========================================================================
 * SESS (Session ID) - 1 byte
 *
 *   Assigned by Master during the Wake-up handshake.
 *   Valid range: 0x01 ~ 0xFE  (254 concurrent sessions per Master)
 * ========================================================================== */
#define PROTO_SESS_UNASSIGNED        0x00U   /* used before session setup */
#define PROTO_SESS_BROADCAST         0xFFU   /* broadcast (wake-up only)  */

/* ==========================================================================
 * Payload type codes
 *
 *   First byte of PAYLOAD when FCF type = DATA or COMMAND.
 *   Allows a single PDU structure to carry heterogeneous MICS content.
 * ========================================================================== */
#define MICS_DTYPE_ECG_DELTA         0x10U   /* ECG delta-compressed data */
#define MICS_DTYPE_BATTERY           0x20U   /* battery status            */
#define MICS_DTYPE_TEMP              0x30U   /* temperature data          */
#define MICS_DTYPE_STATUS_LOG        0x40U   /* device status log         */
#define MICS_DTYPE_COMMAND           0xA0U   /* control command           */
#define MICS_DTYPE_EMERGENCY         0xFFU   /* emergency data (priority) */

/* ---------- Stream logical-data application types ---------- */
#define PROTO_APP_TYPE_WAVEFORM      0x01U
#define PROTO_APP_TYPE_MEASUREMENT   0x02U
#define PROTO_APP_TYPE_EVENT         0x03U

/* ---------- Waveform kinds ---------- */
#define PROTO_WAVEFORM_ECG           0x01U
#define PROTO_WAVEFORM_EGM           0x02U
#define PROTO_WAVEFORM_SENSOR_RAW    0x03U

/* ---------- Measurement bitmap bits ---------- */
#define PROTO_MEAS_HR_BIT            0x01U
#define PROTO_MEAS_TEMP_BIT          0x02U
#define PROTO_MEAS_BATT_BIT          0x04U
#define PROTO_MEAS_IMPEDANCE_BIT     0x08U

/* ---------- Event codes ---------- */
#define PROTO_EVENT_STIM             0x01U
#define PROTO_EVENT_ABNORMAL         0x02U
#define PROTO_EVENT_WAKEUP           0x03U
#define PROTO_EVENT_FAULT            0x04U
#define PROTO_EVENT_LOW_BATTERY      0x05U

/* ==========================================================================
 * Command sub-codes
 *
 *   When payload[0] == MICS_DTYPE_COMMAND, payload[1] is the sub-command.
 *   These replace the old CMD_* codes that used to occupy the MAC header.
 * ========================================================================== */
#define CMD_WAKEUP_REQ               0x01U   /* Master -> Slave (beacon)    */
#define CMD_WAKEUP_ACK               0x02U   /* Slave  -> Master            */
#define CMD_POLL_REQ                 0x10U   /* Master -> Slave             */
#define CMD_POLL_RESP                0x11U   /* Slave  -> Master            */
#define CMD_DATA_WRITE               0x20U   /* Master -> Slave             */
#define CMD_DATA_ACK                 0x21U   /* Slave  -> Master            */
#define CMD_SLEEP_CMD                0x30U   /* Master -> Slave             */
#define CMD_SLEEP_ACK                0x31U   /* Slave  -> Master            */
#define CMD_KEEPALIVE                0x40U   /* Master -> Slave             */
#define CMD_KEEPALIVE_ACK            0x41U   /* Slave  -> Master            */
#define CMD_STREAM_START             0x50U   /* Master -> Slave             */
#define CMD_STREAM_STOP              0x51U   /* Master -> Slave             */
#define CMD_STREAM_ACK               0x52U   /* Slave  -> Master            */
#define CMD_STREAM_NACK              0x53U   /* Master -> Slave (optional)  */
#define CMD_SHIP_CMD                 0x60U   /* Master→Slave: authenticated ship/storage (Stop3+RTC+WKUP) */
#define CMD_SHIP_ACK                 0x61U   /* Slave→Master: SHIP accepted or status error */

/* ---------- Status codes (payload byte after command code) ---------- */
#define PROTO_STATUS_OK              0x00U
#define PROTO_STATUS_ERR_UNKNOWN_CMD 0x01U
#define PROTO_STATUS_ERR_CRC         0x02U
#define PROTO_STATUS_ERR_ID_MISMATCH 0x03U
#define PROTO_STATUS_ERR_BUSY        0x04U
#define PROTO_STATUS_ERR_TIMEOUT     0x05U

/* ==========================================================================
 * Device identity - used ONLY inside Wake-up beacon / ACK payload.
 * Once SESS is assigned, the 4-byte DEVICE_ID is no longer transmitted.
 * ========================================================================== */
#define PROTO_DEVICE_ID_LEN          4U
#define PROTO_BROADCAST_ID           {0xFFU, 0xFFU, 0xFFU, 0xFFU}

/* ==========================================================================
 * Master identity - authenticator for the Slave.
 *
 * Every 2.4 GHz wake-up beacon / ACK and every 400 MHz COMMAND / DATA
 * payload carries the 4-byte MASTER_ID so the Slave can reject frames
 * coming from an unauthorised Master.  The Slave stores its authorised
 * Master ID at factory-pairing time (see app/slave/main.c
 * g_authorized_master_id).  The Master stores its own g_master_id and
 * sends it on every outbound frame; the Slave echoes MASTER_ID on
 * replies for bilateral binding.
 *
 * This is a factory pre-shared identifier, NOT a cryptographic
 * authenticator.  Stronger authentication (HMAC / signature / replay
 * protection) must be layered on top at the application level if the
 * threat model requires it.
 * ========================================================================== */
#define PROTO_MASTER_ID_LEN          4U
#define PROTO_MASTER_ID_BROADCAST    {0xFFU, 0xFFU, 0xFFU, 0xFFU}

/* ==========================================================================
 * Wake-up beacon PAYLOAD layout (inside the 58B PAYLOAD field)
 *
 *   Offset 0     : MICS_DTYPE_COMMAND (0xA0)
 *   Offset 1     : CMD_WAKEUP_REQ (0x01) or CMD_WAKEUP_ACK (0x02)
 *   Offset 2..5  : DEVICE_ID   (target / responder Slave ID)
 *   Offset 6..9  : MASTER_ID   (authenticating Master ID; echoed in ACK)
 *   Offset 10    : SESS        (session ID proposed / echoed)
 *   Offset 11    : CHANNEL     (MICS channel 0..9 pre-selected by Master via LBT)
 *   Offset 12    : flags / reserved
 *                                                  total = 13 B
 *
 * FCF for wake-up = MICS_FCF_TYPE_COMMAND | MICS_FCF_ACK_REQ
 * The MAC-header SESS field is set to PROTO_SESS_BROADCAST during beacon;
 * assigned SESS is transported inside the payload.
 *
 * CHANNEL field is assigned by Master *before* the beacon is sent:
 *   1) Master runs CC1101_FindFreeChannel() over MICS ch 0..9
 *   2) Free channel number is embedded at offset 11 of the beacon payload
 *   3) Slave reads the channel and tunes CC1101 directly to it - no scanning
 *   4) Slave echoes the same channel + MASTER_ID in the wake-up ACK
 * ========================================================================== */
#define PROTO_BEACON_DTYPE_OFFSET    0U
#define PROTO_BEACON_CMD_OFFSET      1U
#define PROTO_BEACON_DEVID_OFFSET    2U
#define PROTO_BEACON_MASTERID_OFFSET 6U
#define PROTO_BEACON_SESS_OFFSET     10U
#define PROTO_BEACON_CHANNEL_OFFSET  11U
#define PROTO_BEACON_FLAGS_OFFSET    12U
#define PROTO_BEACON_PAYLOAD_LEN     13U

#define PROTO_CHANNEL_UNASSIGNED     0xFFU   /* no channel pre-selected */

/* ==========================================================================
 * 400 MHz COMMAND frame PAYLOAD layout (FCF type = MICS_FCF_TYPE_COMMAND)
 *
 *   Offset 0     : MICS_DTYPE_COMMAND (0xA0)
 *   Offset 1     : sub-command (CMD_POLL_REQ, CMD_DATA_WRITE, ...)
 *   Offset 2..5  : DEVICE_ID   (Implant identity, carried from 2.4 GHz wake-up)
 *   Offset 6..9  : MASTER_ID   (authenticating Master, carried from wake-up)
 *   Offset 10..  : command-specific arguments
 *                                                    header = 10 B
 *
 * Both DEVICE_ID and MASTER_ID are present on every 400 MHz COMMAND frame
 * regardless of direction:
 *   - Master → Slave: proves Master still knows the Slave it woke up AND
 *                     proves this is the same Master the Slave trusts
 *   - Slave  → Master: proves the responder is the same Slave that
 *                      accepted the wake-up, and echoes the MASTER_ID it
 *                      received so Master can verify round-trip binding
 * Both ends verify SESS + DEVICE_ID + MASTER_ID on every COMMAND packet.
 * ========================================================================== */
#define PROTO_CMD_DTYPE_OFFSET       0U
#define PROTO_CMD_SUBCMD_OFFSET      1U
#define PROTO_CMD_DEVID_OFFSET       2U
#define PROTO_CMD_MASTERID_OFFSET    6U
#define PROTO_CMD_ARGS_OFFSET        10U
#define PROTO_CMD_HEADER_LEN         10U      /* DTYPE + SUBCMD + DEVID[4] + MASTERID[4] */

/* ==========================================================================
 * 400 MHz DATA frame PAYLOAD layout (FCF type = MICS_FCF_TYPE_DATA)
 *
 *   Offset 0     : data type (MICS_DTYPE_ECG_DELTA, _BATTERY, _TEMP, ...)
 *   Offset 1..4  : DEVICE_ID  (Implant identity, proves DATA is from the
 *                              same Slave that was woken up)
 *   Offset 5..8  : MASTER_ID  (echoes the Master this session is bound to)
 *   Offset 9..   : actual telemetry / data body
 *                                                    header = 9 B
 * ========================================================================== */
#define PROTO_DATA_DTYPE_OFFSET      0U
#define PROTO_DATA_DEVID_OFFSET      1U
#define PROTO_DATA_MASTERID_OFFSET   5U
#define PROTO_DATA_BODY_OFFSET       9U
#define PROTO_DATA_HEADER_LEN        9U      /* DTYPE + DEVID[4] + MASTERID[4] */

/* ---------- timing constants ---------- */
#define PROTO_WAKEUP_BEACON_REPEAT   10U
#define PROTO_WAKEUP_ACK_TIMEOUT_MS  200U
#define PROTO_MICS_RESP_TIMEOUT_MS   500U
#define PROTO_SESSION_TIMEOUT_MS     10000U
#define PROTO_KEEPALIVE_INTERVAL_MS  5000U
#define PROTO_SLEEP_CONFIRM_TIMEOUT_MS 200U

/* ---------- UART log options -----------------------------------------------
 * 상태 상수 (값 직접 사용 금지 — 아래 이름으로만 참조):
 *   PROTO_UART_LOG_ENABLE   = 1  (로그 on)
 *   PROTO_UART_LOG_DISABLE  = 0  (로그 off)
 *
 * 설정 방법 (common/protocol.h 에서만 수정):
 *   모두 off    : 둘 다 PROTO_UART_LOG_DISABLE
 *   모두 on     : 둘 다 PROTO_UART_LOG_ENABLE
 *   Master만 on : MASTER=PROTO_UART_LOG_ENABLE,  SLAVE=PROTO_UART_LOG_DISABLE
 *   Slave만  on : MASTER=PROTO_UART_LOG_DISABLE, SLAVE=PROTO_UART_LOG_ENABLE
 * ----------------------------------------------------------------------- */
#define PROTO_UART_LOG_DISABLE        0   /* 로그 off 상태 상수 — 값 변경 금지 */
#define PROTO_UART_LOG_ENABLE         1   /* 로그 on  상태 상수 — 값 변경 금지 */

#define PROTO_MASTER_UART_LOG_ENABLE  PROTO_UART_LOG_DISABLE   /* Master 로그: ENABLE/DISABLE 선택 */
#define PROTO_SLAVE_UART_LOG_ENABLE   PROTO_UART_LOG_DISABLE   /* Slave  로그: ENABLE/DISABLE 선택 */

/* ==========================================================================
 * Proto_Packet - in-memory representation of one MAC PDU
 * ========================================================================== */
typedef struct {
    uint8_t fcf;                              /* Frame Control Field            */
    uint8_t seq;                              /* Sequence number (wraps 0..255) */
    uint8_t sess;                             /* Session ID                     */
    uint8_t payload[PROTO_MAX_PAYLOAD];       /* 0..58 bytes                    */
    uint8_t payload_len;
} Proto_Packet;

/* ==========================================================================
 * Builders / Parsers
 * ========================================================================== */

static inline uint8_t Proto_BuildPacket(const Proto_Packet *pkt,
                                        uint8_t *buf, uint8_t buf_size)
{
    if ((pkt == NULL) || (buf == NULL)) {
        return 0U;
    }
    if (pkt->payload_len > PROTO_MAX_PAYLOAD) {
        return 0U;
    }

    uint8_t total = (uint8_t)(PROTO_HDR_SIZE + pkt->payload_len);
    if (buf_size < total) {
        return 0U;
    }

    buf[0] = pkt->fcf;
    buf[1] = pkt->seq;
    buf[2] = pkt->sess;

    for (uint8_t i = 0U; i < pkt->payload_len; i++) {
        buf[PROTO_HDR_SIZE + i] = pkt->payload[i];
    }
    return total;
}

static inline int8_t Proto_ParsePacket(const uint8_t *buf, uint8_t len,
                                       Proto_Packet *pkt)
{
    if ((buf == NULL) || (pkt == NULL) || (len < PROTO_HDR_SIZE)) {
        return -1;
    }

    pkt->fcf  = buf[0];
    pkt->seq  = buf[1];
    pkt->sess = buf[2];
    pkt->payload_len = (uint8_t)(len - PROTO_HDR_SIZE);

    if (pkt->payload_len > PROTO_MAX_PAYLOAD) {
        return -2;
    }

    for (uint8_t i = 0U; i < pkt->payload_len; i++) {
        pkt->payload[i] = buf[PROTO_HDR_SIZE + i];
    }
    return 0;
}

/* ==========================================================================
 * FCF helpers
 * ========================================================================== */
static inline uint8_t Proto_FCF_Make(uint8_t frame_type,
                                     bool ack_req, bool fragment)
{
    uint8_t fcf = (uint8_t)(frame_type & MICS_FCF_TYPE_MASK);
    if (ack_req)  { fcf |= MICS_FCF_ACK_REQ; }
    if (fragment) { fcf |= MICS_FCF_FRAGMENT; }
    return fcf;
}

static inline uint8_t Proto_FCF_Type(uint8_t fcf)
{
    return (uint8_t)(fcf & MICS_FCF_TYPE_MASK);
}

static inline bool Proto_FCF_IsAckReq(uint8_t fcf)
{
    return (fcf & MICS_FCF_ACK_REQ) != 0U;
}

static inline bool Proto_FCF_IsFragment(uint8_t fcf)
{
    return (fcf & MICS_FCF_FRAGMENT) != 0U;
}

/* ==========================================================================
 * Session helpers
 * ========================================================================== */
static inline bool Proto_MatchSession(uint8_t a, uint8_t b)
{
    if ((a == PROTO_SESS_BROADCAST) || (b == PROTO_SESS_BROADCAST)) {
        return true;
    }
    return (a == b);
}

/* ==========================================================================
 * Device ID helpers (wake-up payload only)
 * ========================================================================== */
static inline bool Proto_MatchDeviceID(const uint8_t *id_a, const uint8_t *id_b)
{
    static const uint8_t broadcast[PROTO_DEVICE_ID_LEN] = PROTO_BROADCAST_ID;

    bool is_broadcast = true;
    for (uint8_t i = 0U; i < PROTO_DEVICE_ID_LEN; i++) {
        if (id_a[i] != broadcast[i]) { is_broadcast = false; break; }
    }
    if (is_broadcast) { return true; }

    for (uint8_t i = 0U; i < PROTO_DEVICE_ID_LEN; i++) {
        if (id_a[i] != id_b[i]) { return false; }
    }
    return true;
}

/* Strict comparison for authenticated paths (no broadcast wildcard). */
static inline bool Proto_MatchDeviceID_Strict(const uint8_t *id_a, const uint8_t *id_b)
{
    if ((id_a == NULL) || (id_b == NULL)) {
        return false;
    }
    for (uint8_t i = 0U; i < PROTO_DEVICE_ID_LEN; i++) {
        if (id_a[i] != id_b[i]) { return false; }
    }
    return true;
}

/* ==========================================================================
 * Master ID helpers (included in every wake-up / COMMAND / DATA frame)
 * ========================================================================== */
static inline bool Proto_MatchMasterID(const uint8_t *id_a, const uint8_t *id_b)
{
    static const uint8_t broadcast[PROTO_MASTER_ID_LEN] = PROTO_MASTER_ID_BROADCAST;

    bool is_broadcast = true;
    for (uint8_t i = 0U; i < PROTO_MASTER_ID_LEN; i++) {
        if (id_a[i] != broadcast[i]) { is_broadcast = false; break; }
    }
    if (is_broadcast) { return true; }

    for (uint8_t i = 0U; i < PROTO_MASTER_ID_LEN; i++) {
        if (id_a[i] != id_b[i]) { return false; }
    }
    return true;
}

/* Strict comparison for authenticated paths (no broadcast wildcard). */
static inline bool Proto_MatchMasterID_Strict(const uint8_t *id_a, const uint8_t *id_b)
{
    if ((id_a == NULL) || (id_b == NULL)) {
        return false;
    }
    for (uint8_t i = 0U; i < PROTO_MASTER_ID_LEN; i++) {
        if (id_a[i] != id_b[i]) { return false; }
    }
    return true;
}

/* ==========================================================================
 * Convenience: build / parse Wake-up beacon payload body
 *
 *   cmd        : CMD_WAKEUP_REQ (Master→Slave) or CMD_WAKEUP_ACK (Slave→Master)
 *   device_id  : target DEVICE_ID (4 B)
 *   master_id  : authenticating MASTER_ID (4 B, echoed in ACK)
 *   sess       : proposed / echoed SESS (0x01..0xFE)
 *   channel    : MICS channel 0..9 pre-selected by Master (PROTO_CHANNEL_UNASSIGNED
 *                if not yet assigned, not recommended)
 *   flags      : reserved
 * ========================================================================== */
static inline uint8_t Proto_BuildBeaconPayload(uint8_t cmd,
                                               const uint8_t *device_id,
                                               const uint8_t *master_id,
                                               uint8_t sess,
                                               uint8_t channel,
                                               uint8_t flags,
                                               uint8_t *out, uint8_t out_size)
{
    if ((device_id == NULL) || (master_id == NULL) || (out == NULL) ||
        (out_size < PROTO_BEACON_PAYLOAD_LEN)) {
        return 0U;
    }
    out[PROTO_BEACON_DTYPE_OFFSET]   = MICS_DTYPE_COMMAND;
    out[PROTO_BEACON_CMD_OFFSET]     = cmd;
    for (uint8_t i = 0U; i < PROTO_DEVICE_ID_LEN; i++) {
        out[PROTO_BEACON_DEVID_OFFSET + i] = device_id[i];
    }
    for (uint8_t i = 0U; i < PROTO_MASTER_ID_LEN; i++) {
        out[PROTO_BEACON_MASTERID_OFFSET + i] = master_id[i];
    }
    out[PROTO_BEACON_SESS_OFFSET]    = sess;
    out[PROTO_BEACON_CHANNEL_OFFSET] = channel;
    out[PROTO_BEACON_FLAGS_OFFSET]   = flags;
    return PROTO_BEACON_PAYLOAD_LEN;
}

/* ==========================================================================
 * Build a 400 MHz COMMAND-frame payload:
 *   [DTYPE_COMMAND][sub_cmd][DEVICE_ID[4]][MASTER_ID[4]][args...]
 * Returns the number of bytes written to `out`, or 0 on error.
 * ========================================================================== */
static inline uint8_t Proto_BuildCommandPayload(uint8_t sub_cmd,
                                                const uint8_t *device_id,
                                                const uint8_t *master_id,
                                                const uint8_t *args,
                                                uint8_t args_len,
                                                uint8_t *out, uint8_t out_size)
{
    uint8_t total = (uint8_t)(PROTO_CMD_HEADER_LEN + args_len);
    if ((device_id == NULL) || (master_id == NULL) ||
        (out == NULL) || (out_size < total)) {
        return 0U;
    }
    out[PROTO_CMD_DTYPE_OFFSET]  = MICS_DTYPE_COMMAND;
    out[PROTO_CMD_SUBCMD_OFFSET] = sub_cmd;
    for (uint8_t i = 0U; i < PROTO_DEVICE_ID_LEN; i++) {
        out[PROTO_CMD_DEVID_OFFSET + i] = device_id[i];
    }
    for (uint8_t i = 0U; i < PROTO_MASTER_ID_LEN; i++) {
        out[PROTO_CMD_MASTERID_OFFSET + i] = master_id[i];
    }
    if ((args != NULL) && (args_len > 0U)) {
        for (uint8_t i = 0U; i < args_len; i++) {
            out[PROTO_CMD_ARGS_OFFSET + i] = args[i];
        }
    }
    return total;
}

/* ==========================================================================
 * Build a 400 MHz DATA-frame payload:
 *   [dtype][DEVICE_ID[4]][MASTER_ID[4]][body...]
 * Returns the number of bytes written to `out`, or 0 on error.
 * ========================================================================== */
static inline uint8_t Proto_BuildDataPayload(uint8_t dtype,
                                             const uint8_t *device_id,
                                             const uint8_t *master_id,
                                             const uint8_t *body,
                                             uint8_t body_len,
                                             uint8_t *out, uint8_t out_size)
{
    uint8_t total = (uint8_t)(PROTO_DATA_HEADER_LEN + body_len);
    if ((device_id == NULL) || (master_id == NULL) ||
        (out == NULL) || (out_size < total)) {
        return 0U;
    }
    out[PROTO_DATA_DTYPE_OFFSET] = dtype;
    for (uint8_t i = 0U; i < PROTO_DEVICE_ID_LEN; i++) {
        out[PROTO_DATA_DEVID_OFFSET + i] = device_id[i];
    }
    for (uint8_t i = 0U; i < PROTO_MASTER_ID_LEN; i++) {
        out[PROTO_DATA_MASTERID_OFFSET + i] = master_id[i];
    }
    if ((body != NULL) && (body_len > 0U)) {
        for (uint8_t i = 0U; i < body_len; i++) {
            out[PROTO_DATA_BODY_OFFSET + i] = body[i];
        }
    }
    return total;
}

/* ==========================================================================
 * Verify that a received packet's payload carries a matching DEVICE_ID.
 * Caller must pass the correct offset for the frame type (COMMAND or DATA).
 * ========================================================================== */
static inline bool Proto_VerifyPayloadDeviceID(const Proto_Packet *pkt,
                                               uint8_t devid_offset,
                                               const uint8_t *expected_id)
{
    if ((pkt == NULL) || (expected_id == NULL)) {
        return false;
    }
    if (pkt->payload_len < (devid_offset + PROTO_DEVICE_ID_LEN)) {
        return false;
    }
    return Proto_MatchDeviceID_Strict(&pkt->payload[devid_offset], expected_id);
}

/* ==========================================================================
 * Verify that a received packet's payload carries a matching MASTER_ID.
 * Caller must pass the correct offset for the frame type (beacon / COMMAND / DATA).
 * ========================================================================== */
static inline bool Proto_VerifyPayloadMasterID(const Proto_Packet *pkt,
                                               uint8_t masterid_offset,
                                               const uint8_t *expected_id)
{
    if ((pkt == NULL) || (expected_id == NULL)) {
        return false;
    }
    if (pkt->payload_len < (masterid_offset + PROTO_MASTER_ID_LEN)) {
        return false;
    }
    return Proto_MatchMasterID_Strict(&pkt->payload[masterid_offset], expected_id);
}

/* ==========================================================================
 * Sequence anti-replay helpers (modulo-256 monotonic window)
 * - true  : incoming_seq is newer than last_seq
 * - false : duplicate or old/replayed frame
 * ========================================================================== */
static inline bool Proto_IsSeqNewer(uint8_t incoming_seq, uint8_t last_seq)
{
    uint8_t delta = (uint8_t)(incoming_seq - last_seq);
    return (delta != 0U) && (delta <= 127U);
}

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H__ */