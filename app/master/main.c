/*
 * Master (Programmer) - External Device Application
 *
 * Communication flow:
 *   1. CC2500 sends wake-up beacon (2.4 GHz) to implant
 *   2. CC2500 waits for wake-up ACK from implant
 *   3. CC1101 performs LBT to find a free MICS channel (400 MHz)
 *   4. CC1101 sends POLL/DATA commands with channel agility
 *   5. CC1101 receives responses from implant
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
/* Phase 1: Wake-up via CC2500 (2.4 GHz)                                      */
/* ========================================================================== */

static int Master_WakeUpImplant(void)
{
    uint8_t rx_buf[CC2500_PKT_MAX_LEN];
    uint8_t rx_len = 0U;
    Proto_Packet ack_pkt;

    /* Send repeated wake-up beacons */
    CC2500_Status st = CC2500_SendWakeUpBeacon(&g_cc2500,
                                               g_target_id,
                                               PROTO_DEVICE_ID_LEN,
                                               PROTO_WAKEUP_BEACON_REPEAT);
    if (st != CC2500_OK) {
        return -1;
    }

    /* Wait for wake-up ACK on the same channel */
    st = CC2500_WaitAndReadPacket(&g_cc2500,
                                  CC2500_WAKEUP_CHANNEL,
                                  rx_buf, &rx_len,
                                  sizeof(rx_buf),
                                  PROTO_WAKEUP_ACK_TIMEOUT_MS);
    if (st != CC2500_OK) {
        return -2;
    }

    /* Parse and verify ACK */
    if (Proto_ParsePacket(rx_buf, rx_len, &ack_pkt) < 0) {
        return -3;
    }

    if (ack_pkt.cmd != CMD_WAKEUP_ACK) {
        return -4;
    }

    if (!Proto_MatchDeviceID(ack_pkt.device_id, g_target_id)) {
        return -5;
    }

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

    /* Build POLL request */
    poll_pkt.cmd = CMD_POLL_REQ;
    poll_pkt.seq = g_seq++;
    memcpy(poll_pkt.device_id, g_target_id, PROTO_DEVICE_ID_LEN);
    poll_pkt.payload_len = 0U;

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

    if (resp_pkt.cmd != CMD_POLL_RESP) {
        return -5;
    }

    /* Copy payload to caller */
    if ((resp_buf != NULL) && (resp_len != NULL)) {
        memcpy(resp_buf, resp_pkt.payload, resp_pkt.payload_len);
        *resp_len = resp_pkt.payload_len;
    }

    return 0;
}

static int Master_WriteToImplant(const uint8_t *data, uint8_t len)
{
    Proto_Packet write_pkt;
    uint8_t tx_buf[CC1101_PKT_MAX_LEN];
    uint8_t rx_buf[CC1101_PKT_MAX_LEN];
    uint8_t rx_len = 0U;

    if ((data == NULL) || (len == 0U) || (len > PROTO_MAX_PAYLOAD)) {
        return -1;
    }

    /* Build DATA_WRITE packet */
    write_pkt.cmd = CMD_DATA_WRITE;
    write_pkt.seq = g_seq++;
    memcpy(write_pkt.device_id, g_target_id, PROTO_DEVICE_ID_LEN);
    memcpy(write_pkt.payload, data, len);
    write_pkt.payload_len = len;

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

    if (ack_pkt.cmd != CMD_DATA_ACK) {
        return -6;
    }

    /* Check status in payload */
    if ((ack_pkt.payload_len > 0U) && (ack_pkt.payload[0] != PROTO_STATUS_OK)) {
        return -7;
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

    sleep_pkt.cmd = CMD_SLEEP_CMD;
    sleep_pkt.seq = g_seq++;
    memcpy(sleep_pkt.device_id, g_target_id, PROTO_DEVICE_ID_LEN);
    sleep_pkt.payload_len = 0U;

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
