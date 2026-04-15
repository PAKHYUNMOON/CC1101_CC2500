/*
 * Master (Programmer) - External Device Application
 *
 * Communication flow:
 *   1. CC2500 sends wake-up beacon (2.4 GHz) with DEVICE_ID+SESS in payload
 *   2. CC2500 waits for wake-up ACK (payload echoes DEVICE_ID+SESS)
 *   3. CC1101 performs LBT to find a free MICS channel (400 MHz)
 *   4. CC1101 sends POLL/DATA commands tagged with SESS; channel agility
 *   5. CC1101 receives responses filtered by SESS
 *   6. CC1101 sends SLEEP command to implant
 *   7. Both radios return to idle/sleep
 *
 * MCU: STM32U575 Q-series
 * Radios: CC2500 (2.4 GHz wake-up), CC1101 (400 MHz MICS data)
 */

#include "cc1101.h"
#include "cc2500.h"
#include "protocol.h"
#include "stm32u575_lowpower.h"
#include <string.h>

/* ---------- hardware handles ---------- */
static CC1101_HandleTypeDef g_cc1101;
static CC2500_HandleTypeDef g_cc2500;
static LP_HandleTypeDef     g_lp;

/* These would be defined by CubeMX-generated code */
extern SPI_HandleTypeDef hspi1;  /* CC1101 SPI */
extern SPI_HandleTypeDef hspi2;  /* CC2500 SPI */
extern RTC_HandleTypeDef hrtc;

/* ---------- session state ---------- */
static uint8_t g_seq = 0U;
static uint8_t g_target_id[PROTO_DEVICE_ID_LEN] = {0x00, 0x00, 0x00, 0x01};
static uint8_t g_active_channel = 0U;
static uint8_t g_session_id     = PROTO_SESS_UNASSIGNED;
static uint8_t g_next_sess      = 1U;   /* rolling session ID generator */

/* ---------- forward declarations ---------- */
static void Master_InitHardware(void);
static int  Master_WakeUpImplant(void);
static int  Master_PollImplantData(uint8_t *resp_buf, uint8_t *resp_len);
static int  Master_WriteToImplant(const uint8_t *data, uint8_t len);
static int  Master_SendSleepCommand(void);
static int  Master_CommunicationSession(void);

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
     *    PAYLOAD[6]    = proposed SESS
     *    PAYLOAD[7]    = assigned MICS channel (from LBT)
     *    PAYLOAD[8]    = flags
     */
    Proto_Packet beacon_pkt;
    beacon_pkt.fcf  = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, true, false);
    beacon_pkt.seq  = g_seq++;
    beacon_pkt.sess = PROTO_SESS_BROADCAST;
    beacon_pkt.payload_len = Proto_BuildBeaconPayload(CMD_WAKEUP_REQ,
                                                      g_target_id,
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
     *    - payload[1] = CMD_WAKEUP_ACK
     *    - payload[2..5] = our target DEVICE_ID
     *    - payload[6]   = echoed SESS (must match proposed_sess)
     *    - payload[7]   = echoed CHANNEL (must match g_active_channel)
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
    if (!Proto_MatchDeviceID(&ack_pkt.payload[PROTO_BEACON_DEVID_OFFSET],
                             g_target_id)) {
        return -9;
    }
    if (ack_pkt.payload[PROTO_BEACON_SESS_OFFSET] != proposed_sess) {
        return -10;
    }
    if (ack_pkt.payload[PROTO_BEACON_CHANNEL_OFFSET] != g_active_channel) {
        return -11;  /* Slave didn't acknowledge the assigned channel */
    }

    /* 7) Session established */
    g_session_id = proposed_sess;
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
     *   payload = [DTYPE_COMMAND, CMD_POLL_REQ, DEVICE_ID[4]]
     *   The DEVICE_ID proves to Slave that this Master knows the identity
     *   handed off during the 2.4 GHz wake-up handshake.
     */
    poll_pkt.fcf  = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, true, false);
    poll_pkt.seq  = g_seq++;
    poll_pkt.sess = g_session_id;
    poll_pkt.payload_len = Proto_BuildCommandPayload(CMD_POLL_REQ,
                                                     g_target_id,
                                                     NULL, 0U,
                                                     poll_pkt.payload,
                                                     sizeof(poll_pkt.payload));
    if (poll_pkt.payload_len == 0U) {
        return -1;
    }

    uint8_t tx_len = Proto_BuildPacket(&poll_pkt, tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return -1;
    }

    /* Send with LBT + channel agility */
    CC1101_Status st = CC1101_SendPacketLBT_Agile(&g_cc1101,
                                                   tx_buf, tx_len,
                                                   CC1101_MICS_LBT_THRESHOLD_DBM,
                                                   CC1101_MICS_LBT_LISTEN_MS,
                                                   200U,
                                                   CC1101_MICS_MAX_LBT_RETRIES,
                                                   &g_active_channel);
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

    /* Verify DEVICE_ID in payload - must match the Slave we woke up on 2.4 GHz */
    uint8_t ftype = Proto_FCF_Type(resp_pkt.fcf);
    uint8_t devid_off, body_off;

    if (ftype == MICS_FCF_TYPE_DATA) {
        devid_off = PROTO_DATA_DEVID_OFFSET;
        body_off  = PROTO_DATA_BODY_OFFSET;
    } else if (ftype == MICS_FCF_TYPE_COMMAND) {
        devid_off = PROTO_CMD_DEVID_OFFSET;
        body_off  = PROTO_CMD_ARGS_OFFSET;
    } else {
        return -6;
    }

    if (!Proto_VerifyPayloadDeviceID(&resp_pkt, devid_off, g_target_id)) {
        return -7;  /* Device ID mismatch - possible spoofer or wrong Slave */
    }

    /* Copy application body to caller */
    if (resp_pkt.payload_len < body_off) {
        return -8;
    }
    if ((resp_buf != NULL) && (resp_len != NULL)) {
        uint8_t n = (uint8_t)(resp_pkt.payload_len - body_off);
        memcpy(resp_buf, &resp_pkt.payload[body_off], n);
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

    /* Command payload overhead = DTYPE + CMD + DEVID[4] = 6 bytes */
    if ((data == NULL) || (len == 0U) ||
        (len > (PROTO_MAX_PAYLOAD - PROTO_CMD_HEADER_LEN))) {
        return -1;
    }

    /* Build DATA_WRITE packet with DEVICE_ID handoff */
    write_pkt.fcf  = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, true, false);
    write_pkt.seq  = g_seq++;
    write_pkt.sess = g_session_id;
    write_pkt.payload_len = Proto_BuildCommandPayload(CMD_DATA_WRITE,
                                                      g_target_id,
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

    /* Send with LBT on current active channel */
    CC1101_Status st = CC1101_SendPacketLBT(&g_cc1101,
                                            g_active_channel,
                                            tx_buf, tx_len,
                                            CC1101_MICS_LBT_THRESHOLD_DBM,
                                            CC1101_MICS_LBT_LISTEN_MS,
                                            200U);
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
    if (Proto_FCF_Type(ack_pkt.fcf) != MICS_FCF_TYPE_COMMAND) {
        return -7;
    }
    /* Verify DEVICE_ID handoff in ACK */
    if (!Proto_VerifyPayloadDeviceID(&ack_pkt, PROTO_CMD_DEVID_OFFSET,
                                     g_target_id)) {
        return -8;
    }
    if (ack_pkt.payload_len < (PROTO_CMD_ARGS_OFFSET + 1U)) {
        return -9;
    }
    if (ack_pkt.payload[PROTO_CMD_SUBCMD_OFFSET] != CMD_DATA_ACK) {
        return -10;
    }
    if (ack_pkt.payload[PROTO_CMD_ARGS_OFFSET] != PROTO_STATUS_OK) {
        return -11;
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

    CC1101_Status st = CC1101_SendPacketLBT(&g_cc1101,
                                            g_active_channel,
                                            tx_buf, tx_len,
                                            CC1101_MICS_LBT_THRESHOLD_DBM,
                                            CC1101_MICS_LBT_LISTEN_MS,
                                            200U);
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

    return 0;
}

/* ========================================================================== */
/* Full communication session                                                  */
/* ========================================================================== */

static int Master_CommunicationSession(void)
{
    int rc;

    /* Phase 1: Wake up implant via 2.4 GHz */
    rc = Master_WakeUpImplant();
    if (rc != 0) {
        return rc;
    }

    /* Phase 2: Poll data via 400 MHz MICS */
    uint8_t data_buf[PROTO_MAX_PAYLOAD];
    uint8_t data_len = 0U;

    rc = Master_PollImplantData(data_buf, &data_len);
    if (rc != 0) {
        /* Try to send sleep command even if poll failed */
        Master_SendSleepCommand();
        return rc;
    }

    /* Process received data (application-specific) */
    /* ... */

    /* Phase 3: End session - send implant back to sleep */
    Master_SendSleepCommand();

    /* Put radios to sleep */
    CC1101_EnterSleep(&g_cc1101);
    CC2500_EnterSleep(&g_cc2500);

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
