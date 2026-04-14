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
 *   2. CC2500 sends wake-up ACK to Master
 *   3. CC1101 enters RX on all MICS channels (waits for Master poll)
 *   4. CC1101 responds to Master commands (POLL/WRITE/SLEEP)
 *   5. CC1101 powers down, CC2500 re-enters WOR, MCU -> Stop 2
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
static void Slave_SendResponse(const Proto_Packet *resp);

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
    Proto_Packet beacon_pkt;

    /* Read the received beacon packet from CC2500 */
    CC2500_Status st = CC2500_ReadPacket(&g_cc2500, rx_buf, &rx_len, sizeof(rx_buf));
    if (st != CC2500_OK) {
        /* GDO0 fired but no valid packet - might be noise */
        return -1;
    }

    /* Parse beacon: expect CMD_WAKEUP with our device ID */
    if (rx_len < 2U) {
        return -2;
    }

    /* Wake-up beacon format: [CMD_WAKEUP(0x01)] [device_id...] */
    if (rx_buf[0] != 0x01U) {
        return -3;
    }

    /* Verify device ID matches ours or is broadcast */
    bool id_match = true;
    if (rx_len >= (1U + PROTO_DEVICE_ID_LEN)) {
        for (uint8_t i = 0U; i < PROTO_DEVICE_ID_LEN; i++) {
            if ((rx_buf[1U + i] != g_my_id[i]) && (rx_buf[1U + i] != 0xFFU)) {
                id_match = false;
                break;
            }
        }
    }

    if (!id_match) {
        return -4;
    }

    /* Send wake-up ACK */
    Proto_Packet ack_pkt;
    ack_pkt.cmd = CMD_WAKEUP_ACK;
    ack_pkt.seq = g_seq++;
    memcpy(ack_pkt.device_id, g_my_id, PROTO_DEVICE_ID_LEN);
    ack_pkt.payload_len = 0U;

    uint8_t tx_buf[CC2500_PKT_MAX_LEN];
    uint8_t tx_len = Proto_BuildPacket(&ack_pkt, tx_buf, sizeof(tx_buf));
    if (tx_len == 0U) {
        return -5;
    }

    st = CC2500_SendPacketDirect(&g_cc2500, CC2500_WAKEUP_CHANNEL,
                                 tx_buf, tx_len, 100U);
    if (st != CC2500_OK) {
        return -6;
    }

    return 0;
}

/* ========================================================================== */
/* Phase 2: MICS data session                                                  */
/* ========================================================================== */

static int Slave_ProcessCommand(const Proto_Packet *req, Proto_Packet *resp)
{
    resp->seq = req->seq;
    memcpy(resp->device_id, g_my_id, PROTO_DEVICE_ID_LEN);

    switch (req->cmd) {
    case CMD_POLL_REQ:
        resp->cmd = CMD_POLL_RESP;
        memcpy(resp->payload, g_sensor_data, sizeof(g_sensor_data));
        resp->payload_len = (uint8_t)sizeof(g_sensor_data);
        break;

    case CMD_DATA_WRITE:
        /* Write configuration/parameters from Master */
        if (req->payload_len > 0U) {
            /* Application-specific write handling */
            /* e.g., update sensor thresholds, calibration data */
            resp->cmd = CMD_DATA_ACK;
            resp->payload[0] = PROTO_STATUS_OK;
            resp->payload_len = 1U;
        } else {
            resp->cmd = CMD_DATA_ACK;
            resp->payload[0] = PROTO_STATUS_ERR_UNKNOWN_CMD;
            resp->payload_len = 1U;
        }
        break;

    case CMD_SLEEP_CMD:
        resp->cmd = CMD_SLEEP_ACK;
        resp->payload_len = 0U;
        break;

    case CMD_KEEPALIVE:
        resp->cmd = CMD_KEEPALIVE_ACK;
        resp->payload_len = 0U;
        break;

    default:
        resp->cmd = CMD_DATA_ACK;
        resp->payload[0] = PROTO_STATUS_ERR_UNKNOWN_CMD;
        resp->payload_len = 1U;
        break;
    }

    return 0;
}

static void Slave_SendResponse(const Proto_Packet *resp)
{
    uint8_t tx_buf[CC1101_PKT_MAX_LEN];
    uint8_t tx_len = Proto_BuildPacket(resp, tx_buf, sizeof(tx_buf));

    if (tx_len == 0U) {
        return;
    }

    /* Slave does NOT perform LBT - responds directly on the channel
     * Master selected (Master already performed LBT) */
    uint8_t fifo[1U + CC1101_PKT_MAX_LEN];
    fifo[0] = tx_len;
    memcpy(&fifo[1], tx_buf, tx_len);

    CC1101_FlushTx(&g_cc1101);

    /* Direct TX without LBT (Slave privilege - responds to Master's channel) */
    CC1101_Status st;
    uint8_t dummy = 0U;

    /* Write to FIFO and send */
    st = CC1101_SetChannel(&g_cc1101, 0U); /* channel already set from RX */
    (void)st;

    /* Use the low-level approach: write FIFO then strobe TX */
    /* The channel is already set from the RX phase */

    /* Re-use SendPacketLBT with very low listen time as direct send */
    /* This is safe because Master has already cleared the channel */
    CC1101_SendPacketLBT(&g_cc1101, 0U, tx_buf, tx_len,
                         -120, 1U, 200U);  /* -120 dBm threshold = always pass */
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
        bool pkt_received = false;

        for (uint8_t ch = 0U; ch < CC1101_MICS_NUM_CHANNELS; ch++) {
            CC1101_Status st = CC1101_WaitAndReadPacket(&g_cc1101,
                                                        ch,
                                                        rx_buf, &rx_len,
                                                        sizeof(rx_buf),
                                                        50U); /* 50 ms per channel */
            if (st == CC1101_OK) {
                pkt_received = true;

                if (Proto_ParsePacket(rx_buf, rx_len, &req_pkt) < 0) {
                    continue;
                }

                /* Verify packet is addressed to us */
                if (!Proto_MatchDeviceID(req_pkt.device_id, g_my_id)) {
                    continue;
                }

                /* Process command and build response */
                Slave_ProcessCommand(&req_pkt, &resp_pkt);

                /* Send response on the same channel */
                uint8_t tx_buf[CC1101_PKT_MAX_LEN];
                uint8_t tx_len = Proto_BuildPacket(&resp_pkt, tx_buf, sizeof(tx_buf));
                if (tx_len > 0U) {
                    /* Respond with minimal LBT (channel is reserved by Master) */
                    CC1101_SendPacketLBT(&g_cc1101, ch, tx_buf, tx_len,
                                        -120, 1U, 200U);
                }

                /* Check if session should end */
                if (req_pkt.cmd == CMD_SLEEP_CMD) {
                    session_active = false;
                }

                /* Reset session timer on valid packet */
                session_start = HAL_GetTick();
                break;  /* break channel scan, wait for next command */
            }
        }

        if (!pkt_received) {
            /* No packet on any channel - continue scanning */
        }
    }

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

            /* Handle the wake-up beacon */
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
