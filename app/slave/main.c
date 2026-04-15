/*
 * Slave (Implant) - Implantable Device Application
 *
 * Ultra-low power operation:
 *   1. MCU in Stop 2 mode (~2 uA)
 *   2. CC2500 in WOR mode (periodic RX sniff at 2.4 GHz)
 *   3. CC2500 GDO0 -> EXTI wakes MCU on beacon detection
 *   4. CC1101 is powered down until communication session
 *
 * Communication flow (Slave never initiates):
 *   1. CC2500 WOR detects wake-up beacon -> GDO0 EXTI -> MCU wakes
 *   2. Parse beacon, remember SESS assigned by Master
 *   3. CC2500 sends wake-up ACK echoing DEVICE_ID+SESS
 *   4. CC1101 enters RX on all MICS channels (waits for Master poll)
 *   5. CC1101 responds to Master commands filtered by SESS
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

/* ---------- session state ---------- */
static volatile bool g_wakeup_flag = false;
static uint8_t g_seq = 0U;
static uint8_t g_session_id = PROTO_SESS_UNASSIGNED; /* assigned by Master wake-up */

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

/* ========================================================================== */
/* EXTI callback - CC2500 GDO0 wake-up interrupt                              */
/* ========================================================================== */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == g_cc2500.gdo0_pin) {
        g_wakeup_flag = true;
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
    g_lp.gdo0_port   = g_cc2500.gdo0_port;
    g_lp.gdo0_pin    = g_cc2500.gdo0_pin;
    g_lp.gdo0_irqn   = EXTI1_IRQn;
    g_lp.hrtc         = &hrtc;

    LP_Init(&g_lp);

    /* Use lowest voltage scaling for minimum power */
    LP_SetVoltageScaling_LowPower();

    /* Initialize CC2500 for WOR wake-up mode */
    CC2500_InitWakeUp26MHz(&g_cc2500);
}

/* ========================================================================== */
/* Deep sleep entry (main idle state)                                          */
/* ========================================================================== */

static void Slave_EnterDeepSleep(void)
{
    /* Ensure CC1101 is in sleep mode */
    CC1101_EnterSleep(&g_cc1101);

    /* Put CC2500 into WOR mode for wake-up monitoring */
    CC2500_EnterWOR(&g_cc2500);

    /* Configure RTC for periodic self-check (e.g., every 60 seconds) */
    LP_ConfigureRTCWakeUp(&g_lp, 60U);

    /* Enter Stop 2 - MCU draws ~2 uA
     * Wake-up sources:
     *   - CC2500 GDO0 EXTI (beacon received)
     *   - RTC wake-up timer (periodic self-check)
     */
    LP_WakeupSource src = LP_EnterStop2(&g_lp);

    /* --- MCU wakes up here --- */

    if (src & LP_WAKEUP_EXTI_GDO0) {
        g_wakeup_flag = true;
    }

    if (src & LP_WAKEUP_RTC_ALARM) {
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

    /* 4) Validate payload layout:
     *    [0] DTYPE_COMMAND  [1] CMD_WAKEUP_REQ
     *    [2..5] DEVICE_ID   [6] SESS  [7] flags
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

    /* 6) Remember the SESS assigned by Master */
    uint8_t assigned_sess = beacon.payload[PROTO_BEACON_SESS_OFFSET];
    if ((assigned_sess == PROTO_SESS_UNASSIGNED) ||
        (assigned_sess == PROTO_SESS_BROADCAST)) {
        return -8; /* invalid SESS allocation */
    }
    g_session_id = assigned_sess;

    /* 7) Build and send wake-up ACK: echo DEVICE_ID+SESS in payload */
    Proto_Packet ack;
    ack.fcf  = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, false, false);
    ack.seq  = g_seq++;
    ack.sess = PROTO_SESS_BROADCAST;  /* session not yet bilaterally confirmed */
    ack.payload_len = Proto_BuildBeaconPayload(CMD_WAKEUP_ACK,
                                               g_my_id,
                                               g_session_id,
                                               0U,
                                               ack.payload,
                                               sizeof(ack.payload));
    if (ack.payload_len == 0U) {
        return -9;
    }

    uint8_t tx_buf[CC2500_PKT_MAX_LEN];
    uint8_t tx_len = Proto_BuildPacket(&ack, tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return -10;
    }

    st = CC2500_SendPacketDirect(&g_cc2500, CC2500_WAKEUP_CHANNEL,
                                 tx_buf, tx_len, 100U);
    if (st != CC2500_OK) {
        return -11;
    }

    return 0;
}

/* ========================================================================== */
/* Phase 2: MICS data session                                                  */
/* ========================================================================== */

static int Slave_ProcessCommand(const Proto_Packet *req, Proto_Packet *resp)
{
    /* Session is already validated by caller; just echo SESS */
    resp->seq  = req->seq;
    resp->sess = g_session_id;

    /* All inbound packets use COMMAND frames with payload[0]=DTYPE_COMMAND,
     * payload[1]=sub-command. */
    if (req->payload_len < 2U) {
        resp->fcf = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, false, false);
        resp->payload[0] = MICS_DTYPE_COMMAND;
        resp->payload[1] = CMD_DATA_ACK;
        resp->payload[2] = PROTO_STATUS_ERR_UNKNOWN_CMD;
        resp->payload_len = 3U;
        return 0;
    }

    uint8_t sub_cmd = req->payload[1];

    switch (sub_cmd) {
    case CMD_POLL_REQ:
        /* Respond with DATA-type frame carrying sensor payload. */
        resp->fcf = Proto_FCF_Make(MICS_FCF_TYPE_DATA, false, false);
        resp->payload[0] = MICS_DTYPE_ECG_DELTA; /* example data type */
        memcpy(&resp->payload[1], g_sensor_data, sizeof(g_sensor_data));
        resp->payload_len = (uint8_t)(1U + sizeof(g_sensor_data));
        break;

    case CMD_DATA_WRITE:
        /* Accept config/parameters (req->payload[2..]) */
        resp->fcf = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, false, false);
        resp->payload[0] = MICS_DTYPE_COMMAND;
        resp->payload[1] = CMD_DATA_ACK;
        resp->payload[2] = (req->payload_len > 2U) ? PROTO_STATUS_OK
                                                    : PROTO_STATUS_ERR_UNKNOWN_CMD;
        resp->payload_len = 3U;
        break;

    case CMD_SLEEP_CMD:
        resp->fcf = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, false, false);
        resp->payload[0] = MICS_DTYPE_COMMAND;
        resp->payload[1] = CMD_SLEEP_ACK;
        resp->payload_len = 2U;
        break;

    case CMD_KEEPALIVE:
        resp->fcf = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, false, false);
        resp->payload[0] = MICS_DTYPE_COMMAND;
        resp->payload[1] = CMD_KEEPALIVE_ACK;
        resp->payload_len = 2U;
        break;

    default:
        resp->fcf = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, false, false);
        resp->payload[0] = MICS_DTYPE_COMMAND;
        resp->payload[1] = CMD_DATA_ACK;
        resp->payload[2] = PROTO_STATUS_ERR_UNKNOWN_CMD;
        resp->payload_len = 3U;
        break;
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

static int Slave_HandleMICSSession(void)
{
    uint8_t rx_buf[CC1101_PKT_MAX_LEN];
    uint8_t rx_len = 0U;
    Proto_Packet req_pkt, resp_pkt;
    bool session_active = true;
    uint32_t session_start = HAL_GetTick();

    /* Initialize CC1101 for MICS communication */
    CC1101_WakeFromSleep(&g_cc1101);
    CC1101_InitMICSLike26MHz(&g_cc1101);

    while (session_active) {
        /* Check session timeout */
        if ((HAL_GetTick() - session_start) > PROTO_SESSION_TIMEOUT_MS) {
            break;
        }

        /* Scan all MICS channels for Master's command */
        for (uint8_t ch = 0U; ch < CC1101_MICS_NUM_CHANNELS; ch++) {
            CC1101_Status st = CC1101_WaitAndReadPacket(&g_cc1101,
                                                        ch,
                                                        rx_buf, &rx_len,
                                                        sizeof(rx_buf),
                                                        50U); /* 50 ms per channel */
            if (st != CC1101_OK) {
                continue;
            }

            if (Proto_ParsePacket(rx_buf, rx_len, &req_pkt) < 0) {
                continue;
            }

            /* Filter by session ID */
            if (!Proto_MatchSession(req_pkt.sess, g_session_id)) {
                continue;
            }

            /* Process command and send response on the same channel */
            Slave_ProcessCommand(&req_pkt, &resp_pkt);
            Slave_SendResponse(&resp_pkt, ch);

            /* Check if session should end (SLEEP command) */
            if ((req_pkt.payload_len >= 2U) &&
                (req_pkt.payload[1] == CMD_SLEEP_CMD)) {
                session_active = false;
            }

            /* Reset session timer on valid packet */
            session_start = HAL_GetTick();
            break;  /* break channel scan, wait for next command */
        }
    }

    /* Invalidate session */
    g_session_id = PROTO_SESS_UNASSIGNED;
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
                /* Wake-up successful - enter MICS data session */
                Slave_HandleMICSSession();
            }

            /* Session done or wake-up failed - return to deep sleep */
            CC1101_EnterSleep(&g_cc1101);
        }

        /* If woken by RTC, the self-check was done in Slave_EnterDeepSleep.
         * Loop back to sleep. */
    }
}
