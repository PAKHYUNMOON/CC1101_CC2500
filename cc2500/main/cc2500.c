#include "cc2500.h"

/*
 * Profile summary (CC2500, 2.4 GHz ISM)
 * - XOSC: 26 MHz
 * - Base freq: ~2405.0 MHz (FREQ word 0x5C6E5C @ 26 MHz)
 * - Channel spacing: ~299.927 kHz (same CHANSPC as CC1101 profile)
 * - Channels: 0..9  => ~2405.0, 2405.3, ... 2407.7 MHz
 * - Data rate: ~49.99 kbps
 * - RX BW: ~101.56 kHz
 * - Modulation: OOK (ASK) — WOR 감도/전력 최적화용
 * - Deviation: N/A (DEVIATN은 OOK 모드에서 무시됨)
 * - Packet mode: variable length + CRC + append status
 * - GDO0: asserts when packet received with CRC OK
 * - PA: OOK 는 PATABLE[0]=OFF('0'), PATABLE[1]=ON('1'),
 *       FREND0.PA_POWER=1 로 2-엔트리 ramp 사용
 */

static int16_t g_rssi_offset_db = CC2500_DEFAULT_RSSI_OFFSET_DB;

/* -------------------------------------------------------------------------- */
/* small delay                                                                 */
/* -------------------------------------------------------------------------- */

void CC2500_DWT_DelayInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0U;
}

static void cc2500_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) {
        ;
    }
}

void CC2500_SetRSSIOffset(int16_t offset_db)
{
    g_rssi_offset_db = offset_db;
}

/* -------------------------------------------------------------------------- */
/* gpio helpers                                                                 */
/* -------------------------------------------------------------------------- */

static inline void cc2500_cs_low(CC2500_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void cc2500_cs_high(CC2500_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/* -------------------------------------------------------------------------- */
/* SPI helpers                                                                  */
/* -------------------------------------------------------------------------- */

static CC2500_Status cc2500_wait_miso_low(CC2500_HandleTypeDef *dev, uint32_t timeout_us)
{
    if (dev->miso_port == NULL) {
        cc2500_delay_us(10U);
        return CC2500_OK;
    }

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = timeout_us * (SystemCoreClock / 1000000U);

    while (HAL_GPIO_ReadPin(dev->miso_port, dev->miso_pin) == GPIO_PIN_SET) {
        if ((DWT->CYCCNT - start) > ticks) {
            return CC2500_ERR_TIMEOUT;
        }
    }
    return CC2500_OK;
}

static CC2500_Status cc2500_spi_xfer(CC2500_HandleTypeDef *dev, uint8_t tx, uint8_t *rx)
{
    if (HAL_SPI_TransmitReceive(dev->hspi, &tx, rx, 1U, CC2500_SPI_TIMEOUT_MS) != HAL_OK) {
        return CC2500_ERR_SPI;
    }
    return CC2500_OK;
}

static CC2500_Status cc2500_strobe(CC2500_HandleTypeDef *dev, uint8_t strobe, uint8_t *status)
{
    uint8_t rx = 0U;

    cc2500_cs_low(dev);
    if (cc2500_wait_miso_low(dev, 1000U) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_TIMEOUT;
    }

    if (cc2500_spi_xfer(dev, strobe, &rx) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_SPI;
    }

    cc2500_cs_high(dev);

    if (status != NULL) {
        *status = rx;
    }

    return CC2500_OK;
}

static CC2500_Status cc2500_write_reg(CC2500_HandleTypeDef *dev, uint8_t addr, uint8_t value)
{
    uint8_t rx = 0U;

    cc2500_cs_low(dev);
    if (cc2500_wait_miso_low(dev, 1000U) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_TIMEOUT;
    }

    if (cc2500_spi_xfer(dev, addr, &rx) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_SPI;
    }
    if (cc2500_spi_xfer(dev, value, &rx) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_SPI;
    }

    cc2500_cs_high(dev);
    return CC2500_OK;
}

static CC2500_Status cc2500_write_burst(CC2500_HandleTypeDef *dev, uint8_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t rx = 0U;

    cc2500_cs_low(dev);
    if (cc2500_wait_miso_low(dev, 1000U) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_TIMEOUT;
    }

    if (cc2500_spi_xfer(dev, (uint8_t)(addr | CC2500_WRITE_BURST), &rx) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_SPI;
    }

    for (uint8_t i = 0U; i < len; i++) {
        if (cc2500_spi_xfer(dev, data[i], &rx) != CC2500_OK) {
            cc2500_cs_high(dev);
            return CC2500_ERR_SPI;
        }
    }

    cc2500_cs_high(dev);
    return CC2500_OK;
}

static CC2500_Status cc2500_read_reg(CC2500_HandleTypeDef *dev, uint8_t addr, uint8_t *value)
{
    uint8_t rx = 0U;

    if (value == NULL) {
        return CC2500_ERR_PARAM;
    }

    cc2500_cs_low(dev);
    if (cc2500_wait_miso_low(dev, 1000U) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_TIMEOUT;
    }

    if (cc2500_spi_xfer(dev, (uint8_t)(addr | CC2500_READ_SINGLE), &rx) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_SPI;
    }
    if (cc2500_spi_xfer(dev, 0xFFU, &rx) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_SPI;
    }

    cc2500_cs_high(dev);
    *value = rx;
    return CC2500_OK;
}

static CC2500_Status cc2500_read_status(CC2500_HandleTypeDef *dev, uint8_t addr, uint8_t *value)
{
    uint8_t rx = 0U;

    if (value == NULL) {
        return CC2500_ERR_PARAM;
    }

    cc2500_cs_low(dev);
    if (cc2500_wait_miso_low(dev, 1000U) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_TIMEOUT;
    }

    if (cc2500_spi_xfer(dev, (uint8_t)(addr | CC2500_READ_BURST), &rx) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_SPI;
    }
    if (cc2500_spi_xfer(dev, 0xFFU, &rx) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_SPI;
    }

    cc2500_cs_high(dev);
    *value = rx;
    return CC2500_OK;
}

static CC2500_Status cc2500_read_burst(CC2500_HandleTypeDef *dev, uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t rx = 0U;

    if ((data == NULL) && (len != 0U)) {
        return CC2500_ERR_PARAM;
    }

    cc2500_cs_low(dev);
    if (cc2500_wait_miso_low(dev, 1000U) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_TIMEOUT;
    }

    if (cc2500_spi_xfer(dev, (uint8_t)(addr | CC2500_READ_BURST), &rx) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_SPI;
    }

    for (uint8_t i = 0U; i < len; i++) {
        if (cc2500_spi_xfer(dev, 0xFFU, &rx) != CC2500_OK) {
            cc2500_cs_high(dev);
            return CC2500_ERR_SPI;
        }
        data[i] = rx;
    }

    cc2500_cs_high(dev);
    return CC2500_OK;
}

/* -------------------------------------------------------------------------- */
/* state helpers                                                                */
/* -------------------------------------------------------------------------- */

static uint8_t cc2500_get_marcstate(CC2500_HandleTypeDef *dev)
{
    uint8_t v = 0U;
    (void)cc2500_read_status(dev, CC2500_MARCSTATE, &v);
    return (uint8_t)(v & 0x1FU);
}

static CC2500_Status cc2500_wait_state(CC2500_HandleTypeDef *dev, uint8_t expected, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (cc2500_get_marcstate(dev) == expected) {
            return CC2500_OK;
        }
    }
    return CC2500_ERR_TIMEOUT;
}

/* -------------------------------------------------------------------------- */
/* public                                                                       */
/* -------------------------------------------------------------------------- */

CC2500_Status CC2500_Reset(CC2500_HandleTypeDef *dev)
{
    uint8_t dummy = 0U;

    if ((dev == NULL) || (dev->hspi == NULL) || (dev->cs_port == NULL)) {
        return CC2500_ERR_PARAM;
    }

    cc2500_cs_high(dev);
    cc2500_delay_us(50U);

    cc2500_cs_low(dev);
    cc2500_delay_us(10U);
    cc2500_cs_high(dev);
    cc2500_delay_us(50U);

    cc2500_cs_low(dev);
    if (cc2500_wait_miso_low(dev, 2000U) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_TIMEOUT;
    }

    if (cc2500_spi_xfer(dev, CC2500_SRES, &dummy) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_SPI;
    }

    if (cc2500_wait_miso_low(dev, 3000U) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_TIMEOUT;
    }

    cc2500_cs_high(dev);
    cc2500_delay_us(1000U);

    return CC2500_OK;
}

CC2500_Status CC2500_Idle(CC2500_HandleTypeDef *dev)
{
    if (cc2500_strobe(dev, CC2500_SIDLE, NULL) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }
    return cc2500_wait_state(dev, CC2500_MARCSTATE_IDLE, 50U);
}

CC2500_Status CC2500_FlushRx(CC2500_HandleTypeDef *dev)
{
    if (CC2500_Idle(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }
    return cc2500_strobe(dev, CC2500_SFRX, NULL);
}

CC2500_Status CC2500_FlushTx(CC2500_HandleTypeDef *dev)
{
    if (CC2500_Idle(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }
    return cc2500_strobe(dev, CC2500_SFTX, NULL);
}

CC2500_Status CC2500_SetChannel(CC2500_HandleTypeDef *dev, uint8_t ch)
{
    if (ch > 9U) {
        return CC2500_ERR_PARAM;
    }

    /* Frequency-related registers should be updated only in IDLE */
    if (CC2500_Idle(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }

    return cc2500_write_reg(dev, CC2500_CHANNR, ch);
}

CC2500_Status CC2500_InitMICSLike26MHz(CC2500_HandleTypeDef *dev)
{
    static const struct {
        uint8_t reg;
        uint8_t val;
    } cfg[] = {
        /* GDO */
        { CC2500_IOCFG2,   0x2E }, /* high impedance */
        { CC2500_IOCFG1,   0x2E }, /* high impedance */
        { CC2500_IOCFG0,   0x07 }, /* packet received with CRC OK */

        /* FIFO / packet */
        { CC2500_FIFOTHR,  0x47 },
        { CC2500_SYNC1,    0xD3 },
        { CC2500_SYNC0,    0x91 },
        { CC2500_PKTLEN,   CC2500_PKT_MAX_LEN },
        { CC2500_PKTCTRL1, 0x04 }, /* APPEND_STATUS=1 */
        { CC2500_PKTCTRL0, 0x05 }, /* variable length + CRC */
        { CC2500_ADDR,     0x00 },
        { CC2500_CHANNR,   0x00 },

        /* Frequency (2.4 GHz ISM; verify with SmartRF Studio for your crystal) */
        { CC2500_FSCTRL1,  0x06 },
        { CC2500_FSCTRL0,  0x00 },
        { CC2500_FREQ2,    0x5C },
        { CC2500_FREQ1,    0x6E },
        { CC2500_FREQ0,    0x5C },

        /* Modem */
        { CC2500_MDMCFG4,  0xCA }, /* RX BW ~101.56kHz, DRATE_E=10 */
        { CC2500_MDMCFG3,  0xF8 }, /* DRATE_M=248 -> ~49.99kbps */
        { CC2500_MDMCFG2,  0x33 }, /* OOK/ASK, 30/32 sync */
        { CC2500_MDMCFG1,  0x23 }, /* 4-byte preamble, CHANSPC_E=3 */
        { CC2500_MDMCFG0,  0x7A }, /* CHANSPC_M=122 -> ~299.93kHz */
        { CC2500_DEVIATN,  0x00 }, /* OOK: DEVIATN 무시됨 (상위 nibble 0) */

        /* Main radio control */
        { CC2500_MCSM2,    0x07 },
        { CC2500_MCSM1,    0x00 }, /* RX/TX end -> IDLE */
        { CC2500_MCSM0,    0x18 }, /* auto-calibrate on IDLE->RX/TX */

        /* FS / AGC */
        { CC2500_FOCCFG,   0x16 },
        { CC2500_BSCFG,    0x6C },
        { CC2500_AGCCTRL2, 0x43 },
        { CC2500_AGCCTRL1, 0x40 },
        { CC2500_AGCCTRL0, 0x91 },

        /* WOR defaults */
        { CC2500_WOREVT1,  0x87 },
        { CC2500_WOREVT0,  0x6B },
        { CC2500_WORCTRL,  0xFB },

        /* Front-end / calibration / test */
        { CC2500_FREND1,   0x56 },
        { CC2500_FREND0,   0x11 }, /* OOK: PA_POWER=1 -> PATABLE[0]=OFF, PATABLE[1]=ON */
        { CC2500_FSCAL3,   0xE9 },
        { CC2500_FSCAL2,   0x2A },
        { CC2500_FSCAL1,   0x00 },
        { CC2500_FSCAL0,   0x1F },
        { CC2500_RCCTRL1,  0x41 },
        { CC2500_RCCTRL0,  0x00 },
        { CC2500_FSTEST,   0x59 },
        { CC2500_PTEST,    0x7F },
        { CC2500_AGCTEST,  0x3F },
        { CC2500_TEST2,    0x81 },
        { CC2500_TEST1,    0x35 },
        { CC2500_TEST0,    0x09 }
    };

    /* OOK PA ramp: PATABLE[0]='0' state (OFF), PATABLE[1]='1' state (low-power setpoint) */
    static const uint8_t pa_table[2] = { 0x00, 0x12 };

    if (CC2500_Reset(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }

    for (uint32_t i = 0U; i < (sizeof(cfg) / sizeof(cfg[0])); i++) {
        if (cc2500_write_reg(dev, cfg[i].reg, cfg[i].val) != CC2500_OK) {
            return CC2500_ERR_SPI;
        }
    }

    if (cc2500_write_burst(dev, CC2500_PATABLE, pa_table, (uint8_t)sizeof(pa_table)) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }

    if (CC2500_FlushRx(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }
    if (CC2500_FlushTx(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }

    if (cc2500_strobe(dev, CC2500_SCAL, NULL) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }
    HAL_Delay(2U);

    return CC2500_OK;
}

CC2500_Status CC2500_InitWakeUp26MHz(CC2500_HandleTypeDef *dev)
{
    /*
     * Wake-up optimized profile for CC2500 (2.4 GHz)
     * - Lower data rate (~2.4 kbps) for better sensitivity in WOR mode
     * - Wider RX BW for frequency tolerance
     * - WOR enabled with ~500 ms sniff interval
     * - GDO0 asserts on packet received with CRC OK -> wakes MCU via EXTI
     */
    static const struct {
        uint8_t reg;
        uint8_t val;
    } cfg[] = {
        /* GDO: GDO0 = CRC OK (used as EXTI wake-up source) */
        { CC2500_IOCFG2,   0x2E }, /* high impedance */
        { CC2500_IOCFG1,   0x2E }, /* high impedance */
        { CC2500_IOCFG0,   0x07 }, /* asserts on CRC OK */

        /* FIFO / packet */
        { CC2500_FIFOTHR,  0x47 },
        { CC2500_SYNC1,    0xBE }, /* distinct sync word for wake-up */
        { CC2500_SYNC0,    0xEF },
        { CC2500_PKTLEN,   0x0A }, /* max 10 bytes for wake-up packets */
        { CC2500_PKTCTRL1, 0x04 }, /* APPEND_STATUS=1 */
        { CC2500_PKTCTRL0, 0x05 }, /* variable length + CRC */
        { CC2500_ADDR,     0x00 },
        { CC2500_CHANNR,   CC2500_WAKEUP_CHANNEL },

        /* Frequency: 2.4 GHz ISM */
        { CC2500_FSCTRL1,  0x06 },
        { CC2500_FSCTRL0,  0x00 },
        { CC2500_FREQ2,    0x5C },
        { CC2500_FREQ1,    0x6E },
        { CC2500_FREQ0,    0x5C },

        /* Modem: low data rate ~2.4 kbps OOK for WOR sensitivity/power */
        { CC2500_MDMCFG4,  0xF6 }, /* RX BW ~58.04 kHz, DRATE_E=6 */
        { CC2500_MDMCFG3,  0x83 }, /* DRATE_M=131 -> ~2.4 kbps */
        { CC2500_MDMCFG2,  0x33 }, /* OOK/ASK, 30/32 sync */
        { CC2500_MDMCFG1,  0x22 }, /* 4-byte preamble, CHANSPC_E=2 */
        { CC2500_MDMCFG0,  0xF8 }, /* channel spacing */
        { CC2500_DEVIATN,  0x00 }, /* OOK: DEVIATN 무시됨 (상위 nibble 0) */

        /* Main radio control: RX->IDLE after pkt, auto-cal on IDLE->RX/TX */
        { CC2500_MCSM2,    0x07 },
        { CC2500_MCSM1,    0x00 }, /* CCA=always, RX/TX->IDLE */
        { CC2500_MCSM0,    0x18 }, /* auto-calibrate on IDLE->RX/TX */

        /* FS / AGC */
        { CC2500_FOCCFG,   0x16 },
        { CC2500_BSCFG,    0x6C },
        { CC2500_AGCCTRL2, 0x03 },
        { CC2500_AGCCTRL1, 0x40 },
        { CC2500_AGCCTRL0, 0x91 },

        /* WOR: ~500 ms sniff interval
         * EVENT0 = 0x876B, WOR_RES=0 => t_event0 ~ 1.89s @ 26MHz/750
         * Using WOR_RES=1 (2^5 = 32 periods): EVENT0=0x04E2 => ~487 ms */
        { CC2500_WOREVT1,  0x04 },
        { CC2500_WOREVT0,  0xE2 },
        { CC2500_WORCTRL,  0x79 }, /* WOR_RES=1, EVENT1=7 (48 clk), RC_PD=1 */

        /* Front-end / calibration / test */
        { CC2500_FREND1,   0x56 },
        { CC2500_FREND0,   0x11 }, /* OOK: PA_POWER=1 -> PATABLE[0]=OFF, PATABLE[1]=ON */
        { CC2500_FSCAL3,   0xE9 },
        { CC2500_FSCAL2,   0x2A },
        { CC2500_FSCAL1,   0x00 },
        { CC2500_FSCAL0,   0x1F },
        { CC2500_RCCTRL1,  0x41 },
        { CC2500_RCCTRL0,  0x00 },
        { CC2500_FSTEST,   0x59 },
        { CC2500_PTEST,    0x7F },
        { CC2500_AGCTEST,  0x3F },
        { CC2500_TEST2,    0x81 },
        { CC2500_TEST1,    0x35 },
        { CC2500_TEST0,    0x09 }
    };

    /* OOK PA ramp: PATABLE[0]='0' state (carrier OFF),
     * PATABLE[1]='1' state (high power for wake-up reliability) */
    static const uint8_t pa_table[2] = { 0x00, 0xFE };

    if (CC2500_Reset(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }

    for (uint32_t i = 0U; i < (sizeof(cfg) / sizeof(cfg[0])); i++) {
        if (cc2500_write_reg(dev, cfg[i].reg, cfg[i].val) != CC2500_OK) {
            return CC2500_ERR_SPI;
        }
    }

    if (cc2500_write_burst(dev, CC2500_PATABLE, pa_table, (uint8_t)sizeof(pa_table)) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }

    if (CC2500_FlushRx(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }
    if (CC2500_FlushTx(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }

    if (cc2500_strobe(dev, CC2500_SCAL, NULL) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }
    HAL_Delay(2U);

    return CC2500_OK;
}

int16_t CC2500_ReadRSSI_dBm(CC2500_HandleTypeDef *dev)
{
    uint8_t raw = 0U;
    int16_t rssi_dec;

    if (cc2500_read_status(dev, CC2500_RSSI, &raw) != CC2500_OK) {
        return -127;
    }

    if (raw >= 128U) {
        rssi_dec = (int16_t)raw - 256;
    } else {
        rssi_dec = (int16_t)raw;
    }

    return (int16_t)(rssi_dec / 2) - g_rssi_offset_db;
}

bool CC2500_ChannelClear(CC2500_HandleTypeDef *dev,
                         uint8_t ch,
                         int16_t threshold_dBm,
                         uint32_t listen_ms)
{
    if (CC2500_SetChannel(dev, ch) != CC2500_OK) {
        return false;
    }

    if (CC2500_FlushRx(dev) != CC2500_OK) {
        return false;
    }

    if (cc2500_strobe(dev, CC2500_SRX, NULL) != CC2500_OK) {
        return false;
    }

    HAL_Delay(listen_ms);

    int16_t rssi = CC2500_ReadRSSI_dBm(dev);

    (void)CC2500_Idle(dev);

    return (rssi < threshold_dBm);
}

int8_t CC2500_FindFreeChannel(CC2500_HandleTypeDef *dev,
                              int16_t threshold_dBm,
                              uint32_t listen_ms)
{
    for (uint8_t ch = 0U; ch < 10U; ch++) {
        if (CC2500_ChannelClear(dev, ch, threshold_dBm, listen_ms)) {
            return (int8_t)ch;
        }
    }
    return -1;
}

CC2500_Status CC2500_EnterRx(CC2500_HandleTypeDef *dev, uint8_t ch)
{
    if (CC2500_SetChannel(dev, ch) != CC2500_OK) {
        return CC2500_ERR_PARAM;
    }

    if (CC2500_FlushRx(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }

    if (cc2500_strobe(dev, CC2500_SRX, NULL) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }

    return cc2500_wait_state(dev, CC2500_MARCSTATE_RX, 20U);
}

CC2500_Status CC2500_SendPacketLBT(CC2500_HandleTypeDef *dev,
                                   uint8_t ch,
                                   const uint8_t *data,
                                   uint8_t len,
                                   int16_t cca_threshold_dBm,
                                   uint32_t listen_ms,
                                   uint32_t tx_timeout_ms)
{
    uint8_t fifo[1U + CC2500_PKT_MAX_LEN];

    if ((data == NULL) || (len == 0U) || (len > CC2500_PKT_MAX_LEN)) {
        return CC2500_ERR_PARAM;
    }

    if (!CC2500_ChannelClear(dev, ch, cca_threshold_dBm, listen_ms)) {
        return CC2500_ERR_CCA;
    }

    if (CC2500_FlushTx(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }

    fifo[0] = len;
    for (uint8_t i = 0U; i < len; i++) {
        fifo[1U + i] = data[i];
    }

    if (cc2500_write_burst(dev, CC2500_TXFIFO, fifo, (uint8_t)(len + 1U)) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }

    if (cc2500_strobe(dev, CC2500_STX, NULL) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }

    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < tx_timeout_ms) {
        uint8_t st = cc2500_get_marcstate(dev);

        if (st == CC2500_MARCSTATE_IDLE) {
            return CC2500_OK;
        }

        if (st == CC2500_MARCSTATE_TXFIFO_UNDERFLOW) {
            (void)CC2500_FlushTx(dev);
            return CC2500_ERR_STATE;
        }
    }

    (void)CC2500_Idle(dev);
    return CC2500_ERR_TIMEOUT;
}

CC2500_Status CC2500_ReadPacket(CC2500_HandleTypeDef *dev,
                                uint8_t *buf,
                                uint8_t *len,
                                uint8_t max_len)
{
    uint8_t pkt_len = 0U;
    uint8_t status_bytes[2] = {0U, 0U};
    uint8_t rx = 0U;

    if ((buf == NULL) || (len == NULL) || (max_len == 0U)) {
        return CC2500_ERR_PARAM;
    }

    /*
     * Call this after GDO0 IRQ (IOCFG0 = 0x07) or after confirming packet complete.
     * Reads length + payload + appended status in one burst.
     */

    cc2500_cs_low(dev);
    if (cc2500_wait_miso_low(dev, 1000U) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_TIMEOUT;
    }

    if (cc2500_spi_xfer(dev, (uint8_t)(CC2500_RXFIFO | CC2500_READ_BURST), &rx) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_SPI;
    }

    if (cc2500_spi_xfer(dev, 0xFFU, &pkt_len) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_SPI;
    }

    if ((pkt_len == 0U) || (pkt_len > CC2500_PKT_MAX_LEN) || (pkt_len > max_len)) {
        cc2500_cs_high(dev);
        (void)CC2500_FlushRx(dev);
        return CC2500_ERR_RX;
    }

    for (uint8_t i = 0U; i < pkt_len; i++) {
        if (cc2500_spi_xfer(dev, 0xFFU, &buf[i]) != CC2500_OK) {
            cc2500_cs_high(dev);
            return CC2500_ERR_SPI;
        }
    }

    if (cc2500_spi_xfer(dev, 0xFFU, &status_bytes[0]) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_SPI;
    }
    if (cc2500_spi_xfer(dev, 0xFFU, &status_bytes[1]) != CC2500_OK) {
        cc2500_cs_high(dev);
        return CC2500_ERR_SPI;
    }

    cc2500_cs_high(dev);

    if ((status_bytes[1] & 0x80U) == 0U) {
        (void)CC2500_FlushRx(dev);
        return CC2500_ERR_RX;
    }

    *len = pkt_len;
    return CC2500_OK;
}

/* -------------------------------------------------------------------------- */
/* power management                                                            */
/* -------------------------------------------------------------------------- */

CC2500_Status CC2500_EnterSleep(CC2500_HandleTypeDef *dev)
{
    if (CC2500_Idle(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }
    return cc2500_strobe(dev, CC2500_SPWD, NULL);
}

CC2500_Status CC2500_WakeFromSleep(CC2500_HandleTypeDef *dev)
{
    cc2500_cs_low(dev);
    cc2500_delay_us(10U);
    cc2500_cs_high(dev);

    cc2500_delay_us(200U);

    if (cc2500_wait_miso_low(dev, 5000U) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }

    return cc2500_wait_state(dev, CC2500_MARCSTATE_IDLE, 50U);
}

/* -------------------------------------------------------------------------- */
/* WOR (Wake-on-Radio) for Slave implant                                       */
/* -------------------------------------------------------------------------- */

CC2500_Status CC2500_ConfigureWOR(CC2500_HandleTypeDef *dev,
                                  uint8_t wor_res,
                                  uint16_t event0_timeout)
{
    if (CC2500_Idle(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }

    /* Set EVENT0 timeout */
    if (cc2500_write_reg(dev, CC2500_WOREVT1, (uint8_t)(event0_timeout >> 8U)) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }
    if (cc2500_write_reg(dev, CC2500_WOREVT0, (uint8_t)(event0_timeout & 0xFFU)) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }

    /* WORCTRL: WOR_RES | EVENT1=7 (48 clk) | RC_PD=1 (RC osc off in sleep) */
    uint8_t worctrl = (uint8_t)((wor_res << 4U) | 0x79U);
    if (cc2500_write_reg(dev, CC2500_WORCTRL, worctrl) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }

    return CC2500_OK;
}

CC2500_Status CC2500_EnterWOR(CC2500_HandleTypeDef *dev)
{
    /* Ensure RX->IDLE on pkt, so GDO0 pulse can trigger EXTI before WOR re-sleeps */
    if (cc2500_write_reg(dev, CC2500_MCSM1, 0x00) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }

    if (CC2500_FlushRx(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }

    /* Reset WOR timer and enter WOR mode */
    if (cc2500_strobe(dev, CC2500_SWORRST, NULL) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }
    if (cc2500_strobe(dev, CC2500_SWOR, NULL) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }

    return CC2500_OK;
}

/* -------------------------------------------------------------------------- */
/* Wake-up beacon for Master programmer                                        */
/* -------------------------------------------------------------------------- */

CC2500_Status CC2500_SendWakeUpBeacon(CC2500_HandleTypeDef *dev,
                                      const uint8_t *pkt,
                                      uint8_t pkt_len,
                                      uint8_t repeat_count)
{
    /*
     * Repeatedly transmit the caller-supplied MAC PDU on the wake-up
     * channel. The PDU itself is built by the application layer using
     * Proto_BuildPacket()+Proto_BuildBeaconPayload() from protocol.h.
     * This keeps the radio driver free of protocol knowledge.
     */
    if ((pkt == NULL) || (pkt_len == 0U) || (pkt_len > CC2500_PKT_MAX_LEN)) {
        return CC2500_ERR_PARAM;
    }
    if (repeat_count == 0U) {
        repeat_count = CC2500_WAKEUP_BEACON_REPEAT;
    }

    if (CC2500_SetChannel(dev, CC2500_WAKEUP_CHANNEL) != CC2500_OK) {
        return CC2500_ERR_STATE;
    }

    for (uint8_t r = 0U; r < repeat_count; r++) {
        if (CC2500_FlushTx(dev) != CC2500_OK) {
            return CC2500_ERR_TIMEOUT;
        }

        uint8_t fifo[1U + CC2500_PKT_MAX_LEN];
        fifo[0] = pkt_len;
        for (uint8_t i = 0U; i < pkt_len; i++) {
            fifo[1U + i] = pkt[i];
        }

        if (cc2500_write_burst(dev, CC2500_TXFIFO, fifo,
                               (uint8_t)(pkt_len + 1U)) != CC2500_OK) {
            return CC2500_ERR_SPI;
        }

        if (cc2500_strobe(dev, CC2500_STX, NULL) != CC2500_OK) {
            return CC2500_ERR_SPI;
        }

        /* Wait for TX to complete */
        if (cc2500_wait_state(dev, CC2500_MARCSTATE_IDLE, 100U) != CC2500_OK) {
            (void)CC2500_Idle(dev);
        }

        if (r < (repeat_count - 1U)) {
            HAL_Delay(CC2500_WAKEUP_BEACON_INTERVAL_MS);
        }
    }

    return CC2500_OK;
}

/* -------------------------------------------------------------------------- */
/* direct send (no LBT, for wake-up channel)                                   */
/* -------------------------------------------------------------------------- */

CC2500_Status CC2500_SendPacketDirect(CC2500_HandleTypeDef *dev,
                                      uint8_t ch,
                                      const uint8_t *data,
                                      uint8_t len,
                                      uint32_t tx_timeout_ms)
{
    uint8_t fifo[1U + CC2500_PKT_MAX_LEN];

    if ((data == NULL) || (len == 0U) || (len > CC2500_PKT_MAX_LEN)) {
        return CC2500_ERR_PARAM;
    }

    if (CC2500_SetChannel(dev, ch) != CC2500_OK) {
        return CC2500_ERR_STATE;
    }

    if (CC2500_FlushTx(dev) != CC2500_OK) {
        return CC2500_ERR_TIMEOUT;
    }

    fifo[0] = len;
    for (uint8_t i = 0U; i < len; i++) {
        fifo[1U + i] = data[i];
    }

    if (cc2500_write_burst(dev, CC2500_TXFIFO, fifo, (uint8_t)(len + 1U)) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }

    if (cc2500_strobe(dev, CC2500_STX, NULL) != CC2500_OK) {
        return CC2500_ERR_SPI;
    }

    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < tx_timeout_ms) {
        uint8_t st = cc2500_get_marcstate(dev);
        if (st == CC2500_MARCSTATE_IDLE) {
            return CC2500_OK;
        }
        if (st == CC2500_MARCSTATE_TXFIFO_UNDERFLOW) {
            (void)CC2500_FlushTx(dev);
            return CC2500_ERR_STATE;
        }
    }

    (void)CC2500_Idle(dev);
    return CC2500_ERR_TIMEOUT;
}

/* -------------------------------------------------------------------------- */
/* RX with timeout                                                             */
/* -------------------------------------------------------------------------- */

CC2500_Status CC2500_WaitAndReadPacket(CC2500_HandleTypeDef *dev,
                                       uint8_t ch,
                                       uint8_t *buf,
                                       uint8_t *len,
                                       uint8_t max_len,
                                       uint32_t timeout_ms)
{
    if (CC2500_EnterRx(dev, ch) != CC2500_OK) {
        return CC2500_ERR_STATE;
    }

    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        if ((dev->gdo0_port != NULL) &&
            (HAL_GPIO_ReadPin(dev->gdo0_port, dev->gdo0_pin) == GPIO_PIN_SET)) {
            return CC2500_ReadPacket(dev, buf, len, max_len);
        }

        if (dev->gdo0_port == NULL) {
            uint8_t rxbytes = 0U;
            (void)cc2500_read_status(dev, CC2500_RXBYTES, &rxbytes);
            if ((rxbytes & 0x7FU) > 0U) {
                return CC2500_ReadPacket(dev, buf, len, max_len);
            }
        }
    }

    (void)CC2500_Idle(dev);
    return CC2500_ERR_TIMEOUT;
}