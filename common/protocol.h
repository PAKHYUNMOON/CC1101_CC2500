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
 *   Maximum LEN field = 63 (FCF+SEQ+SESS+PAYLOAD bytes after LEN).
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
 * Wake-up beacon PAYLOAD layout (inside the 58B PAYLOAD field)
 *
 *   Offset 0    : MICS_DTYPE_COMMAND (0xA0)
 *   Offset 1    : CMD_WAKEUP_REQ (0x01) or CMD_WAKEUP_ACK (0x02)
 *   Offset 2..5 : DEVICE_ID  (target / responder)
 *   Offset 6    : SESS       (session ID proposed / echoed)
 *   Offset 7    : CHANNEL    (MICS channel 0..9 pre-selected by Master via LBT)
 *   Offset 8    : flags / reserved
 *
 * FCF for wake-up = MICS_FCF_TYPE_COMMAND | MICS_FCF_ACK_REQ
 * The MAC-header SESS field is set to PROTO_SESS_BROADCAST during beacon;
 * assigned SESS is transported inside the payload.
 *
 * CHANNEL field is assigned by Master *before* the beacon is sent:
 *   1) Master runs CC1101_FindFreeChannel() over MICS ch 0..9
 *   2) Free channel number is embedded at offset 7 of the beacon payload
 *   3) Slave reads the channel and tunes CC1101 directly to it - no scanning
 *   4) Slave echoes the same channel in the wake-up ACK for confirmation
 * ========================================================================== */
#define PROTO_BEACON_DTYPE_OFFSET    0U
#define PROTO_BEACON_CMD_OFFSET      1U
#define PROTO_BEACON_DEVID_OFFSET    2U
#define PROTO_BEACON_SESS_OFFSET     6U
#define PROTO_BEACON_CHANNEL_OFFSET  7U
#define PROTO_BEACON_FLAGS_OFFSET    8U
#define PROTO_BEACON_PAYLOAD_LEN     9U

#define PROTO_CHANNEL_UNASSIGNED     0xFFU   /* no channel pre-selected */

/* ==========================================================================
 * 400 MHz COMMAND frame PAYLOAD layout (FCF type = MICS_FCF_TYPE_COMMAND)
 *
 *   Offset 0    : MICS_DTYPE_COMMAND (0xA0)
 *   Offset 1    : sub-command (CMD_POLL_REQ, CMD_DATA_WRITE, ...)
 *   Offset 2..5 : DEVICE_ID  (Implant identity, carried from 2.4 GHz wake-up)
 *   Offset 6..  : command-specific arguments
 *
 * The DEVICE_ID at offset 2..5 is the *Implant's* (Slave's) ID on every
 * 400 MHz COMMAND frame, regardless of direction.  Purpose:
 *   - Master → Slave: proves Master still knows the Slave it woke up
 *   - Slave  → Master: proves the responder is the same Slave that
 *                      accepted the wake-up on 2.4 GHz
 * Both ends verify DEVICE_ID in addition to SESS on every COMMAND packet.
 * ========================================================================== */
#define PROTO_CMD_DTYPE_OFFSET       0U
#define PROTO_CMD_SUBCMD_OFFSET      1U
#define PROTO_CMD_DEVID_OFFSET       2U
#define PROTO_CMD_ARGS_OFFSET        6U
#define PROTO_CMD_HEADER_LEN         6U      /* DTYPE + SUBCMD + DEVID[4] */

/* ==========================================================================
 * 400 MHz DATA frame PAYLOAD layout (FCF type = MICS_FCF_TYPE_DATA)
 *
 *   Offset 0    : data type (MICS_DTYPE_ECG_DELTA, _BATTERY, _TEMP, ...)
 *   Offset 1..4 : DEVICE_ID  (Implant identity, proves DATA is from the
 *                             same Slave that was woken up)
 *   Offset 5..  : actual telemetry / data body
 * ========================================================================== */
#define PROTO_DATA_DTYPE_OFFSET      0U
#define PROTO_DATA_DEVID_OFFSET      1U
#define PROTO_DATA_BODY_OFFSET       5U
#define PROTO_DATA_HEADER_LEN        5U      /* DTYPE + DEVID[4] */

/* ---------- timing constants ---------- */
#define PROTO_WAKEUP_BEACON_REPEAT   10U
#define PROTO_WAKEUP_ACK_TIMEOUT_MS  200U
#define PROTO_MICS_RESP_TIMEOUT_MS   500U
#define PROTO_SESSION_TIMEOUT_MS     10000U
#define PROTO_KEEPALIVE_INTERVAL_MS  5000U
#define PROTO_SLEEP_CONFIRM_TIMEOUT_MS 200U

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

/* ==========================================================================
 * Convenience: build / parse Wake-up beacon payload body
 *
 *   cmd        : CMD_WAKEUP_REQ (Master→Slave) or CMD_WAKEUP_ACK (Slave→Master)
 *   device_id  : target DEVICE_ID (4 B)
 *   sess       : proposed / echoed SESS (0x01..0xFE)
 *   channel    : MICS channel 0..9 pre-selected by Master (PROTO_CHANNEL_UNASSIGNED
 *                if not yet assigned, not recommended)
 *   flags      : reserved
 * ========================================================================== */
static inline uint8_t Proto_BuildBeaconPayload(uint8_t cmd,
                                               const uint8_t *device_id,
                                               uint8_t sess,
                                               uint8_t channel,
                                               uint8_t flags,
                                               uint8_t *out, uint8_t out_size)
{
    if ((device_id == NULL) || (out == NULL) ||
        (out_size < PROTO_BEACON_PAYLOAD_LEN)) {
        return 0U;
    }
    out[PROTO_BEACON_DTYPE_OFFSET]   = MICS_DTYPE_COMMAND;
    out[PROTO_BEACON_CMD_OFFSET]     = cmd;
    for (uint8_t i = 0U; i < PROTO_DEVICE_ID_LEN; i++) {
        out[PROTO_BEACON_DEVID_OFFSET + i] = device_id[i];
    }
    out[PROTO_BEACON_SESS_OFFSET]    = sess;
    out[PROTO_BEACON_CHANNEL_OFFSET] = channel;
    out[PROTO_BEACON_FLAGS_OFFSET]   = flags;
    return PROTO_BEACON_PAYLOAD_LEN;
}

/* ==========================================================================
 * Build a 400 MHz COMMAND-frame payload:
 *   [DTYPE_COMMAND][sub_cmd][DEVICE_ID[4]][args...]
 * Returns the number of bytes written to `out`, or 0 on error.
 * ========================================================================== */
static inline uint8_t Proto_BuildCommandPayload(uint8_t sub_cmd,
                                                const uint8_t *device_id,
                                                const uint8_t *args,
                                                uint8_t args_len,
                                                uint8_t *out, uint8_t out_size)
{
    uint8_t total = (uint8_t)(PROTO_CMD_HEADER_LEN + args_len);
    if ((device_id == NULL) || (out == NULL) || (out_size < total)) {
        return 0U;
    }
    out[PROTO_CMD_DTYPE_OFFSET]  = MICS_DTYPE_COMMAND;
    out[PROTO_CMD_SUBCMD_OFFSET] = sub_cmd;
    for (uint8_t i = 0U; i < PROTO_DEVICE_ID_LEN; i++) {
        out[PROTO_CMD_DEVID_OFFSET + i] = device_id[i];
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
 *   [dtype][DEVICE_ID[4]][body...]
 * Returns the number of bytes written to `out`, or 0 on error.
 * ========================================================================== */
static inline uint8_t Proto_BuildDataPayload(uint8_t dtype,
                                             const uint8_t *device_id,
                                             const uint8_t *body,
                                             uint8_t body_len,
                                             uint8_t *out, uint8_t out_size)
{
    uint8_t total = (uint8_t)(PROTO_DATA_HEADER_LEN + body_len);
    if ((device_id == NULL) || (out == NULL) || (out_size < total)) {
        return 0U;
    }
    out[PROTO_DATA_DTYPE_OFFSET] = dtype;
    for (uint8_t i = 0U; i < PROTO_DEVICE_ID_LEN; i++) {
        out[PROTO_DATA_DEVID_OFFSET + i] = device_id[i];
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
    return Proto_MatchDeviceID(&pkt->payload[devid_offset], expected_id);
}

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H__ */
