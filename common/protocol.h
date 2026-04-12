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
 *   CC1101 (400 MHz)  : MICS-band data communication with LBT
 *
 * Roles:
 *   Master (Programmer)   - Initiates all communication; performs LBT
 *   Slave  (Implant)      - Responds only; never self-initiates
 * ========================================================================== */

/* ---------- device identity ---------- */
#define PROTO_DEVICE_ID_LEN          4U     /* 4-byte unique device ID */
#define PROTO_BROADCAST_ID           {0xFF, 0xFF, 0xFF, 0xFF}

/* ---------- packet structure ----------
 * All packets on both radios share this envelope:
 *
 * Byte 0       : CMD  (command / message type)
 * Byte 1       : SEQ  (sequence number, wraps at 255)
 * Byte 2..5    : DEVICE_ID[4]  (target or source)
 * Byte 6..N    : PAYLOAD (command-specific, 0..55 bytes)
 *
 * CRC is handled by the radio hardware (PKTCTRL0.CRC_EN).
 * Length byte is prepended automatically by the driver.
 * ------------------------------------------------------ */
#define PROTO_HDR_SIZE               6U     /* CMD + SEQ + ID[4] */
#define PROTO_MAX_PAYLOAD            (61U - PROTO_HDR_SIZE)

/* ---------- command codes (CC2500 wake-up channel) ---------- */
#define CMD_WAKEUP_BEACON            0x01U  /* Master -> Slave: wake up */
#define CMD_WAKEUP_ACK               0x02U  /* Slave  -> Master: I'm awake */

/* ---------- command codes (CC1101 MICS data channel) ---------- */
#define CMD_POLL_REQ                 0x10U  /* Master -> Slave: request data */
#define CMD_POLL_RESP                0x11U  /* Slave  -> Master: data response */
#define CMD_DATA_WRITE               0x20U  /* Master -> Slave: write config/param */
#define CMD_DATA_ACK                 0x21U  /* Slave  -> Master: write acknowledged */
#define CMD_SLEEP_CMD                0x30U  /* Master -> Slave: return to deep sleep */
#define CMD_SLEEP_ACK                0x31U  /* Slave  -> Master: sleep acknowledged */
#define CMD_KEEPALIVE                0x40U  /* Master -> Slave: extend session */
#define CMD_KEEPALIVE_ACK            0x41U  /* Slave  -> Master: session extended */

/* ---------- error / status codes in payload ---------- */
#define PROTO_STATUS_OK              0x00U
#define PROTO_STATUS_ERR_UNKNOWN_CMD 0x01U
#define PROTO_STATUS_ERR_CRC         0x02U
#define PROTO_STATUS_ERR_ID_MISMATCH 0x03U
#define PROTO_STATUS_ERR_BUSY        0x04U
#define PROTO_STATUS_ERR_TIMEOUT     0x05U

/* ---------- timing constants ---------- */
#define PROTO_WAKEUP_BEACON_REPEAT   10U    /* beacons sent per wake-up attempt */
#define PROTO_WAKEUP_ACK_TIMEOUT_MS  200U   /* time to wait for wake-up ACK */
#define PROTO_MICS_RESP_TIMEOUT_MS   500U   /* time to wait for MICS data response */
#define PROTO_SESSION_TIMEOUT_MS     10000U /* max session duration before auto-sleep */
#define PROTO_KEEPALIVE_INTERVAL_MS  5000U  /* keepalive interval within session */
#define PROTO_SLEEP_CONFIRM_TIMEOUT_MS 200U

/* ---------- packet builder / parser ---------- */

typedef struct {
    uint8_t cmd;
    uint8_t seq;
    uint8_t device_id[PROTO_DEVICE_ID_LEN];
    uint8_t payload[PROTO_MAX_PAYLOAD];
    uint8_t payload_len;
} Proto_Packet;

static inline uint8_t Proto_BuildPacket(const Proto_Packet *pkt, uint8_t *buf, uint8_t buf_size)
{
    uint8_t total = (uint8_t)(PROTO_HDR_SIZE + pkt->payload_len);
    if ((buf == NULL) || (buf_size < total)) {
        return 0U;
    }

    buf[0] = pkt->cmd;
    buf[1] = pkt->seq;
    buf[2] = pkt->device_id[0];
    buf[3] = pkt->device_id[1];
    buf[4] = pkt->device_id[2];
    buf[5] = pkt->device_id[3];

    for (uint8_t i = 0U; i < pkt->payload_len; i++) {
        buf[PROTO_HDR_SIZE + i] = pkt->payload[i];
    }

    return total;
}

static inline int8_t Proto_ParsePacket(const uint8_t *buf, uint8_t len, Proto_Packet *pkt)
{
    if ((buf == NULL) || (pkt == NULL) || (len < PROTO_HDR_SIZE)) {
        return -1;
    }

    pkt->cmd = buf[0];
    pkt->seq = buf[1];
    pkt->device_id[0] = buf[2];
    pkt->device_id[1] = buf[3];
    pkt->device_id[2] = buf[4];
    pkt->device_id[3] = buf[5];
    pkt->payload_len = (uint8_t)(len - PROTO_HDR_SIZE);

    for (uint8_t i = 0U; i < pkt->payload_len; i++) {
        pkt->payload[i] = buf[PROTO_HDR_SIZE + i];
    }

    return 0;
}

static inline bool Proto_MatchDeviceID(const uint8_t *id_a, const uint8_t *id_b)
{
    /* Also matches broadcast ID */
    static const uint8_t broadcast[PROTO_DEVICE_ID_LEN] = PROTO_BROADCAST_ID;
    bool is_broadcast = true;

    for (uint8_t i = 0U; i < PROTO_DEVICE_ID_LEN; i++) {
        if (id_a[i] != broadcast[i]) {
            is_broadcast = false;
            break;
        }
    }
    if (is_broadcast) {
        return true;
    }

    for (uint8_t i = 0U; i < PROTO_DEVICE_ID_LEN; i++) {
        if (id_a[i] != id_b[i]) {
            return false;
        }
    }
    return true;
}

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H__ */
