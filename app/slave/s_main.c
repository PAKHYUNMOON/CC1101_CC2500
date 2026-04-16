/*
 * Slave (Implant) - Implantable Device Application
 *
 * Ultra-low power design — two phases:
 *
 *   ┌─────────────────────────────────────────────────────────────────────┐
 *   │  IDLE phase (waiting for Master wake-up beacon)                    │
 *   │  MCU    : Stop 2  ~1.6 uA                                          │
 *   │  CC2500 : WOR     ~70 uA avg  (500 ms sniff @ 2.4 GHz OOK)        │
 *   │  CC1101 : SLEEP   ~200 nA     (SPWD strobe)                        │
 *   │  Wake   : CC2500 GDO0 EXTI (CRC OK on beacon)                      │
 *   │           RTC 60 s timer    (periodic self-check)                   │
 *   │  Total  : ~72 uA  — dominated by CC2500 WOR sniff                  │
 *   ├─────────────────────────────────────────────────────────────────────┤
 *   │  SESSION phase (MICS 400 MHz data exchange, typically < 5 s)       │
 *   │  MCU    : Stop 2 between frames  ~1.6 uA                           │
 *   │  CC1101 : RX      ~15 mA  — dominant, minimised by short dwell    │
 *   │           IDLE between frames when MCU is in Stop 2                │
 *   │  CC2500 : IDLE                                                      │
 *   │  Wake   : CC1101 GDO0 EXTI (CRC OK on command frame)               │
 *   │           RTC SESSION_TIMEOUT_S timer (session watchdog)           │
 *   └─────────────────────────────────────────────────────────────────────┘
 *
 * HAL_GetTick() stops advancing while MCU is in Stop 2 so session timeout
 * is implemented via the RTC wake-up timer, not HAL_GetTick().
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
static volatile bool g_wakeup_flag     = false;   /* CC2500 GDO0 EXTI flag */
static volatile bool g_cc1101_pkt_flag = false;   /* CC1101 GDO0 EXTI flag */
static uint8_t g_seq = 0U;
static uint8_t g_session_id     = PROTO_SESS_UNASSIGNED;
static uint8_t g_active_channel = PROTO_CHANNEL_UNASSIGNED;

/* ---------- simulated sensor data ---------- */
static uint8_t g_sensor_data[16] = {
    0x01, 0x02, 0x03, 0x04,  /* placeholder sensor values */
    0x05, 0x06, 0x07, 0x08,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};

/* Session timeout in seconds (derived from ms constant, rounded up) */
#define SESSION_TIMEOUT_S   ((PROTO_SESSION_TIMEOUT_MS + 999U) / 1000U)

/* ---------- forward declarations ---------- */
static void Slave_InitHardware(void);
static void Slave_EnterDeepSleep(void);
static int  Slave_HandleWakeUp(void);
static int  Slave_HandleMICSSession(void);
static int  Slave_ProcessCommand(const Proto_Packet *req, Proto_Packet *resp);
static void Slave_SendResponse(const Proto_Packet *resp, uint8_t channel);

/* ========================================================================== */
/* EXTI callbacks                                                              */
/* ========================================================================== */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == g_cc2500.gdo0_pin) {
        /* CC2500 GDO0: WOR beacon received (CRC OK) */
        g_wakeup_flag = true;
    } else if (GPIO_Pin == g_cc1101.gdo0_pin) {
        /* CC1101 GDO0: MICS command frame received (CRC OK) */
        g_cc1101_pkt_flag = true;
    }
}

/* ========================================================================== */
/* Hardware initialization                                                     */
/* ========================================================================== */

static void Slave_InitHardware(void)
{
    CC1101_DWT_DelayInit();

    /* CC1101 handle (400 MHz MICS) */
    g_cc1101.hspi      = &hspi1;
    g_cc1101.cs_port   = GPIOA;
    g_cc1101.cs_pin    = GPIO_PIN_4;
    g_cc1101.miso_port = GPIOA;
    g_cc1101.miso_pin  = GPIO_PIN_6;
    g_cc1101.gdo0_port = GPIOB;
    g_cc1101.gdo0_pin  = GPIO_PIN_0;  /* PB0 = EXTI0 */

    /* CC2500 handle (2.4 GHz WOR wake-up) */
    g_cc2500.hspi      = &hspi2;
    g_cc2500.cs_port   = GPIOB;
    g_cc2500.cs_pin    = GPIO_PIN_12;
    g_cc2500.miso_port = GPIOB;
    g_cc2500.miso_pin  = GPIO_PIN_14;
    g_cc2500.gdo0_port = GPIOB;
    g_cc2500.gdo0_pin  = GPIO_PIN_1;  /* PB1 = EXTI1 */

    /* Low-power handle:
     *   CC2500 GDO0 on EXTI1 (WOR beacon → IDLE phase wake)
     *   CC1101 GDO0 on EXTI0 (MICS frame → SESSION phase wake)
     *   RTC: 60 s self-check in IDLE; SESSION_TIMEOUT_S watchdog in SESSION */
    g_lp.hspi_cc1101        = &hspi1;
    g_lp.hspi_cc2500        = &hspi2;
    g_lp.gdo0_port          = g_cc2500.gdo0_port;
    g_lp.gdo0_pin           = g_cc2500.gdo0_pin;
    g_lp.gdo0_irqn          = EXTI1_IRQn;
    g_lp.cc1101_gdo0_port   = g_cc1101.gdo0_port;
    g_lp.cc1101_gdo0_pin    = g_cc1101.gdo0_pin;
    g_lp.cc1101_gdo0_irqn   = EXTI0_IRQn;
    g_lp.hrtc               = &hrtc;

    LP_Init(&g_lp);

    /* VOS4: lowest core voltage (max 24 MHz) — minimum active current */
    LP_SetVoltageScaling_LowPower();

    /* Configure CC2500 for WOR beacon reception */
    CC2500_InitWakeUp26MHz(&g_cc2500);
}

/* ========================================================================== */
/* IDLE phase deep sleep                                                       */
/* ========================================================================== */

static void Slave_EnterDeepSleep(void)
{
    /* CC1101 in SLEEP (SPWD, ~200 nA) */
    CC1101_EnterSleep(&g_cc1101);

    /* CC2500 in WOR (~70 uA avg at 500 ms sniff interval) */
    CC2500_EnterWOR(&g_cc2500);

    /* RTC 60 s periodic self-check */
    LP_ConfigureRTCWakeUp(&g_lp, 60U);

    /* MCU in Stop 2 (~1.6 uA)
     * Wake: CC2500 GDO0 EXTI  or  RTC 60 s */
    LP_WakeupSource src = LP_EnterStop2(&g_lp);

    if (src & LP_WAKEUP_EXTI_GDO0) {
        g_wakeup_flag = true;
    }

    if (src & LP_WAKEUP_RTC_ALARM) {
        /* Periodic self-check: update timestamp; add battery / sensor checks
         * as needed.  If nothing needs processing, the loop returns here
         * and re-enters Stop 2 on the next iteration. */
        g_sensor_data[8] = (uint8_t)(g_seq & 0xFFU);  /* lightweight activity */
    }
}

/* ========================================================================== */
/* Phase 1: Handle CC2500 wake-up beacon                                       */
/* ========================================================================== */

static int Slave_HandleWakeUp(void)
{
    uint8_t rx_buf[CC2500_PKT_MAX_LEN];
    uint8_t rx_len = 0U;

    /* 1) Read beacon from CC2500 RX FIFO */
    CC2500_Status st = CC2500_ReadPacket(&g_cc2500, rx_buf, &rx_len, sizeof(rx_buf));
    if (st != CC2500_OK) {
        return -1;
    }

    /* 2) Parse MAC PDU */
    Proto_Packet beacon;
    if (Proto_ParsePacket(rx_buf, rx_len, &beacon) < 0) {
        return -2;
    }

    /* 3) Validate FCF: must be COMMAND type */
    if (Proto_FCF_Type(beacon.fcf) != MICS_FCF_TYPE_COMMAND) {
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

    /* 5) Verify DEVICE_ID (us or broadcast) */
    const uint8_t *target_id = &beacon.payload[PROTO_BEACON_DEVID_OFFSET];
    if (!Proto_MatchDeviceID(target_id, g_my_id)) {
        return -7;
    }

    /* 6) Verify MASTER_ID — silent drop if not our paired Master */
    if (!Proto_VerifyPayloadMasterID(&beacon,
                                     PROTO_BEACON_MASTERID_OFFSET,
                                     g_authorized_master_id)) {
        return -8;
    }

    /* 7) Validate assigned SESS */
    uint8_t assigned_sess = beacon.payload[PROTO_BEACON_SESS_OFFSET];
    if ((assigned_sess == PROTO_SESS_UNASSIGNED) ||
        (assigned_sess == PROTO_SESS_BROADCAST)) {
        return -9;
    }

    /* 8) Accept the pre-selected MICS channel (no scanning needed) */
    uint8_t assigned_channel = beacon.payload[PROTO_BEACON_CHANNEL_OFFSET];
    if (assigned_channel >= CC1101_MICS_NUM_CHANNELS) {
        return -10;
    }

    g_session_id     = assigned_sess;
    g_active_channel = assigned_channel;

    /* 9) Build and send wake-up ACK echoing DEVICE_ID + MASTER_ID +
     *    SESS + CHANNEL so Master can verify bilateral binding. */
    Proto_Packet ack;
    ack.fcf  = Proto_FCF_Make(MICS_FCF_TYPE_COMMAND, false, false);
    ack.seq  = g_seq++;
    ack.sess = PROTO_SESS_BROADCAST;
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
/* Phase 2: MICS data session (low-power RX loop)                              */
/* ========================================================================== */

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
    resp->seq  = req->seq;
    resp->sess = g_session_id;

    if (req->payload_len < PROTO_CMD_HEADER_LEN) {
        uint8_t st = PROTO_STATUS_ERR_UNKNOWN_CMD;
        slave_build_cmd_response(resp, CMD_DATA_ACK, &st, 1U);
        return 0;
    }

    uint8_t sub_cmd = req->payload[PROTO_CMD_SUBCMD_OFFSET];

    switch (sub_cmd) {
    case CMD_POLL_REQ:
        resp->fcf = Proto_FCF_Make(MICS_FCF_TYPE_DATA, false, false);
        resp->payload_len = Proto_BuildDataPayload(MICS_DTYPE_ECG_DELTA,
                                                   g_my_id,
                                                   g_authorized_master_id,
                                                   g_sensor_data,
                                                   (uint8_t)sizeof(g_sensor_data),
                                                   resp->payload,
                                                   sizeof(resp->payload));
        break;

    case CMD_DATA_WRITE:
    {
        uint8_t status = (req->payload_len > PROTO_CMD_ARGS_OFFSET)
                         ? PROTO_STATUS_OK
                         : PROTO_STATUS_ERR_UNKNOWN_CMD;
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

    /* Slave responds on the channel Master pre-selected via LBT.
     * Use CCA threshold = -120 dBm (effectively bypassed) because
     * the Master already cleared the channel; a false CCA block here
     * would only cause unnecessary retries. */
    (void)CC1101_SendPacketLBT(&g_cc1101, channel,
                               tx_buf, tx_len,
                               -120, 1U, 200U);
    /* After TX, MCSM1.TXOFF_MODE=RX (0x03) returns CC1101 to RX
     * automatically, ready for the next incoming command. */
}

static int Slave_HandleMICSSession(void)
{
    uint8_t rx_buf[CC1101_PKT_MAX_LEN];
    uint8_t rx_len = 0U;
    Proto_Packet req_pkt, resp_pkt;
    bool session_active = true;

    if (g_active_channel >= CC1101_MICS_NUM_CHANNELS) {
        return -1;
    }

    /* Bring CC1101 up and configure for MICS session */
    CC1101_WakeFromSleep(&g_cc1101);
    CC1101_InitMICSLike26MHz(&g_cc1101);

    /* Enter RX on the channel the Master pre-selected via LBT.
     * CC1101 stays in RX; MCU enters Stop 2 between frames. */
    CC1101_EnterRx(&g_cc1101, g_active_channel);

    /* Session watchdog: RTC fires after SESSION_TIMEOUT_S.
     * Because HAL_GetTick() stops during Stop 2, we rely on the RTC
     * timer rather than HAL_GetTick()-based elapsed-time tracking. */
    LP_ConfigureRTCWakeUp(&g_lp, SESSION_TIMEOUT_S);

    while (session_active) {
        /* --- MCU enters Stop 2 (~1.6 uA) ---
         * Wake sources:
         *   LP_WAKEUP_EXTI_CC1101_GDO0 : CC1101 GDO0 asserted (CRC OK packet)
         *   LP_WAKEUP_RTC_ALARM        : session timeout
         * CC1101 remains in RX during Stop 2; its current (~15 mA) dominates
         * in the session phase, but the MCU's contribution drops from
         * ~1 mA (busy-poll at 4 MHz) to ~1.6 uA. */
        g_cc1101_pkt_flag = false;
        LP_WakeupSource src = LP_EnterStop2(&g_lp);

        if (src & LP_WAKEUP_RTC_ALARM) {
            /* Session timed out — return to IDLE phase */
            break;
        }

        if (!(src & LP_WAKEUP_EXTI_CC1101_GDO0)) {
            /* Spurious wake (e.g. CC2500 GDO0) — re-enter Stop 2 */
            continue;
        }

        /* --- CC1101 packet ready in FIFO --- */
        if (CC1101_ReadPacket(&g_cc1101, rx_buf, &rx_len, sizeof(rx_buf)) != CC1101_OK) {
            /* FIFO error; re-enter RX and try again */
            CC1101_EnterRx(&g_cc1101, g_active_channel);
            continue;
        }

        if (Proto_ParsePacket(rx_buf, rx_len, &req_pkt) < 0) {
            CC1101_EnterRx(&g_cc1101, g_active_channel);
            continue;
        }

        /* Filter by session ID */
        if (!Proto_MatchSession(req_pkt.sess, g_session_id)) {
            CC1101_EnterRx(&g_cc1101, g_active_channel);
            continue;
        }

        /* Only COMMAND frames from Master on 400 MHz */
        if (Proto_FCF_Type(req_pkt.fcf) != MICS_FCF_TYPE_COMMAND) {
            CC1101_EnterRx(&g_cc1101, g_active_channel);
            continue;
        }

        /* Triple verify: SESS (above) + DEVICE_ID + MASTER_ID */
        if (!Proto_VerifyPayloadDeviceID(&req_pkt,
                                         PROTO_CMD_DEVID_OFFSET,
                                         g_my_id)) {
            CC1101_EnterRx(&g_cc1101, g_active_channel);
            continue;
        }
        if (!Proto_VerifyPayloadMasterID(&req_pkt,
                                         PROTO_CMD_MASTERID_OFFSET,
                                         g_authorized_master_id)) {
            CC1101_EnterRx(&g_cc1101, g_active_channel);
            continue;
        }

        /* Process command and send response */
        Slave_ProcessCommand(&req_pkt, &resp_pkt);
        Slave_SendResponse(&resp_pkt, g_active_channel);
        /* MCSM1.TXOFF=RX: CC1101 auto-returns to RX after TX —
         * no explicit re-entry needed for the next round. */

        /* End session on SLEEP command */
        if ((req_pkt.payload_len >= PROTO_CMD_HEADER_LEN) &&
            (req_pkt.payload[PROTO_CMD_SUBCMD_OFFSET] == CMD_SLEEP_CMD)) {
            session_active = false;
        }
    }

    LP_DisableRTCWakeUp(&g_lp);

    /* Invalidate session */
    g_session_id     = PROTO_SESS_UNASSIGNED;
    g_active_channel = PROTO_CHANNEL_UNASSIGNED;
    return 0;
}

/* ========================================================================== */
/* Application entry (called from CubeMX-generated Core/Src/main.c)            */
/* ========================================================================== */

void Slave_AppMain(void)
{
    /* HAL_Init(), SystemClock_Config(), MX_GPIO_Init(),
     * MX_SPI1_Init(), MX_SPI2_Init(), MX_RTC_Init() are invoked
     * by CubeMX-generated Core/Src/main() before reaching this entry. */

    Slave_InitHardware();

    /* CC1101 starts in SLEEP — not needed until a session is established */
    CC1101_EnterSleep(&g_cc1101);

    while (1) {
        /* Primary state: IDLE deep sleep
         *   MCU     : Stop 2 (~1.6 uA)
         *   CC2500  : WOR    (~70 uA avg)
         *   CC1101  : SLEEP  (~200 nA)
         *   Total   : ~72 uA
         */
        Slave_EnterDeepSleep();

        if (g_wakeup_flag) {
            g_wakeup_flag = false;

            /* Parse beacon → store SESS + CHANNEL → send Wake-up ACK */
            int rc = Slave_HandleWakeUp();
            if (rc == 0) {
                /* SESSION phase: MCU Stop 2 + CC1101 RX + RTC watchdog */
                Slave_HandleMICSSession();
            }

            /* Session done or beacon invalid — back to IDLE */
            CC1101_EnterSleep(&g_cc1101);
        }

        /* If woken by RTC self-check: housekeeping was done in
         * Slave_EnterDeepSleep(); loop back to Stop 2. */
    }
}
