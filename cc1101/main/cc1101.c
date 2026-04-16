#include "cc1101.h"

/*
 * Profile summary
 * - XOSC: 26 MHz
 * - Base freq: 402.150 MHz
 * - Channel spacing: ~299.927 kHz
 * - Channels: 0..9  => 402.15, 402.45, ... 404.85 MHz
 * - Data rate: ~49.99 kbps
 * - RX BW: ~101.56 kHz
 * - Modulation: 2-FSK (MDMCFG2.MOD_FORMAT = 000)
 * - Deviation: ~20.63 kHz
 * - Packet mode: variable length + CRC + append status
 * - GDO0: asserts when packet received with CRC OK
 */

static int16_t g_rssi_offset_db = CC1101_DEFAULT_RSSI_OFFSET_DB;

/* -------------------------------------------------------------------------- */
/* small delay                                                                 */
/* -------------------------------------------------------------------------- */

void CC1101_DWT_DelayInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0U;
}

static void cc1101_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) {
        ;
    }
}

void CC1101_SetRSSIOffset(int16_t offset_db)
{
    g_rssi_offset_db = offset_db;
}

/* -------------------------------------------------------------------------- */
/* gpio helpers                                                                 */
/* -------------------------------------------------------------------------- */

static inline void cc1101_cs_low(CC1101_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void cc1101_cs_high(CC1101_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/* -------------------------------------------------------------------------- */
/* SPI helpers                                                                  */
/* -------------------------------------------------------------------------- */

static CC1101_Status cc1101_wait_miso_low(CC1101_HandleTypeDef *dev, uint32_t timeout_us)
{
    if (dev->miso_port == NULL) {
        cc1101_delay_us(10U);
        return CC1101_OK;
    }

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = timeout_us * (SystemCoreClock / 1000000U);

    while (HAL_GPIO_ReadPin(dev->miso_port, dev->miso_pin) == GPIO_PIN_SET) {
        if ((DWT->CYCCNT - start) > ticks) {
            return CC1101_ERR_TIMEOUT;
        }
    }
    return CC1101_OK;
}

static CC1101_Status cc1101_spi_xfer(CC1101_HandleTypeDef *dev, uint8_t tx, uint8_t *rx)
{
    if (HAL_SPI_TransmitReceive(dev->hspi, &tx, rx, 1U, CC1101_SPI_TIMEOUT_MS) != HAL_OK) {
        return CC1101_ERR_SPI;
    }
    return CC1101_OK;
}

static CC1101_Status cc1101_strobe(CC1101_HandleTypeDef *dev, uint8_t strobe, uint8_t *status)
{
    uint8_t rx = 0U;

    cc1101_cs_low(dev);
    if (cc1101_wait_miso_low(dev, 1000U) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_TIMEOUT;
    }

    if (cc1101_spi_xfer(dev, strobe, &rx) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_SPI;
    }

    cc1101_cs_high(dev);

    if (status != NULL) {
        *status = rx;
    }

    return CC1101_OK;
}

static CC1101_Status cc1101_write_reg(CC1101_HandleTypeDef *dev, uint8_t addr, uint8_t value)
{
    uint8_t rx = 0U;

    cc1101_cs_low(dev);
    if (cc1101_wait_miso_low(dev, 1000U) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_TIMEOUT;
    }

    if (cc1101_spi_xfer(dev, addr, &rx) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_SPI;
    }
    if (cc1101_spi_xfer(dev, value, &rx) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_SPI;
    }

    cc1101_cs_high(dev);
    return CC1101_OK;
}

static CC1101_Status cc1101_write_burst(CC1101_HandleTypeDef *dev, uint8_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t rx = 0U;

    cc1101_cs_low(dev);
    if (cc1101_wait_miso_low(dev, 1000U) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_TIMEOUT;
    }

    if (cc1101_spi_xfer(dev, (uint8_t)(addr | CC1101_WRITE_BURST), &rx) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_SPI;
    }

    for (uint8_t i = 0U; i < len; i++) {
        if (cc1101_spi_xfer(dev, data[i], &rx) != CC1101_OK) {
            cc1101_cs_high(dev);
            return CC1101_ERR_SPI;
        }
    }

    cc1101_cs_high(dev);
    return CC1101_OK;
}

static CC1101_Status cc1101_read_reg(CC1101_HandleTypeDef *dev, uint8_t addr, uint8_t *value)
{
    uint8_t rx = 0U;

    if (value == NULL) {
        return CC1101_ERR_PARAM;
    }

    cc1101_cs_low(dev);
    if (cc1101_wait_miso_low(dev, 1000U) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_TIMEOUT;
    }

    if (cc1101_spi_xfer(dev, (uint8_t)(addr | CC1101_READ_SINGLE), &rx) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_SPI;
    }
    if (cc1101_spi_xfer(dev, 0xFFU, &rx) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_SPI;
    }

    cc1101_cs_high(dev);
    *value = rx;
    return CC1101_OK;
}

static CC1101_Status cc1101_read_status(CC1101_HandleTypeDef *dev, uint8_t addr, uint8_t *value)
{
    uint8_t rx = 0U;

    if (value == NULL) {
        return CC1101_ERR_PARAM;
    }

    cc1101_cs_low(dev);
    if (cc1101_wait_miso_low(dev, 1000U) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_TIMEOUT;
    }

    if (cc1101_spi_xfer(dev, (uint8_t)(addr | CC1101_READ_BURST), &rx) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_SPI;
    }
    if (cc1101_spi_xfer(dev, 0xFFU, &rx) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_SPI;
    }

    cc1101_cs_high(dev);
    *value = rx;
    return CC1101_OK;
}

static CC1101_Status cc1101_read_burst(CC1101_HandleTypeDef *dev, uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t rx = 0U;

    if ((data == NULL) && (len != 0U)) {
        return CC1101_ERR_PARAM;
    }

    cc1101_cs_low(dev);
    if (cc1101_wait_miso_low(dev, 1000U) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_TIMEOUT;
    }

    if (cc1101_spi_xfer(dev, (uint8_t)(addr | CC1101_READ_BURST), &rx) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_SPI;
    }

    for (uint8_t i = 0U; i < len; i++) {
        if (cc1101_spi_xfer(dev, 0xFFU, &rx) != CC1101_OK) {
            cc1101_cs_high(dev);
            return CC1101_ERR_SPI;
        }
        data[i] = rx;
    }

    cc1101_cs_high(dev);
    return CC1101_OK;
}

/* -------------------------------------------------------------------------- */
/* state helpers                                                                */
/* -------------------------------------------------------------------------- */

static uint8_t cc1101_get_marcstate(CC1101_HandleTypeDef *dev)
{
    uint8_t v = 0U;
    (void)cc1101_read_status(dev, CC1101_MARCSTATE, &v);
    return (uint8_t)(v & 0x1FU);
}

static CC1101_Status cc1101_wait_state(CC1101_HandleTypeDef *dev, uint8_t expected, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (cc1101_get_marcstate(dev) == expected) {
            return CC1101_OK;
        }
    }
    return CC1101_ERR_TIMEOUT;
}

/* -------------------------------------------------------------------------- */
/* public                                                                       */
/* -------------------------------------------------------------------------- */

CC1101_Status CC1101_Reset(CC1101_HandleTypeDef *dev)
{
    uint8_t dummy = 0U;

    if ((dev == NULL) || (dev->hspi == NULL) || (dev->cs_port == NULL)) {
        return CC1101_ERR_PARAM;
    }

    cc1101_cs_high(dev);
    cc1101_delay_us(50U);

    cc1101_cs_low(dev);
    cc1101_delay_us(10U);
    cc1101_cs_high(dev);
    cc1101_delay_us(50U);

    cc1101_cs_low(dev);
    if (cc1101_wait_miso_low(dev, 2000U) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_TIMEOUT;
    }

    if (cc1101_spi_xfer(dev, CC1101_SRES, &dummy) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_SPI;
    }

    if (cc1101_wait_miso_low(dev, 3000U) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_TIMEOUT;
    }

    cc1101_cs_high(dev);
    cc1101_delay_us(1000U);

    return CC1101_OK;
}

CC1101_Status CC1101_Idle(CC1101_HandleTypeDef *dev)
{
    if (cc1101_strobe(dev, CC1101_SIDLE, NULL) != CC1101_OK) {
        return CC1101_ERR_SPI;
    }
    return cc1101_wait_state(dev, CC1101_MARCSTATE_IDLE, 50U);
}

CC1101_Status CC1101_FlushRx(CC1101_HandleTypeDef *dev)
{
    if (CC1101_Idle(dev) != CC1101_OK) {
        return CC1101_ERR_TIMEOUT;
    }
    return cc1101_strobe(dev, CC1101_SFRX, NULL);
}

CC1101_Status CC1101_FlushTx(CC1101_HandleTypeDef *dev)
{
    if (CC1101_Idle(dev) != CC1101_OK) {
        return CC1101_ERR_TIMEOUT;
    }
    return cc1101_strobe(dev, CC1101_SFTX, NULL);
}

CC1101_Status CC1101_SetChannel(CC1101_HandleTypeDef *dev, uint8_t ch)
{
    if (ch > 9U) {
        return CC1101_ERR_PARAM;
    }

    /* Frequency-related registers should be updated only in IDLE */
    if (CC1101_Idle(dev) != CC1101_OK) {
        return CC1101_ERR_TIMEOUT;
    }

    return cc1101_write_reg(dev, CC1101_CHANNR, ch);
}

CC1101_Status CC1101_InitMICSLike26MHz(CC1101_HandleTypeDef *dev)
{
    static const struct {
        uint8_t reg;
        uint8_t val;
    } cfg[] = {
        /* GDO */
        { CC1101_IOCFG2,   0x2E }, /* high impedance */
        { CC1101_IOCFG1,   0x2E }, /* high impedance */
        { CC1101_IOCFG0,   0x07 }, /* packet received with CRC OK */

        /* FIFO / packet */
        { CC1101_FIFOTHR,  0x47 },
        { CC1101_SYNC1,    0xD3 },
        { CC1101_SYNC0,    0x91 },
        { CC1101_PKTLEN,   CC1101_PKT_MAX_LEN },
        { CC1101_PKTCTRL1, 0x04 }, /* APPEND_STATUS=1 */
        { CC1101_PKTCTRL0, 0x05 }, /* variable length + CRC */
        { CC1101_ADDR,     0x00 },
        { CC1101_CHANNR,   0x00 },

        /* Frequency */
        { CC1101_FSCTRL1,  0x06 },
        { CC1101_FSCTRL0,  0x00 },
        { CC1101_FREQ2,    0x0F },
        { CC1101_FREQ1,    0x77 },
        { CC1101_FREQ0,    0xA1 },

        /* Modem */
        { CC1101_MDMCFG4,  0xCA }, /* RX BW ~101.56kHz, DRATE_E=10 */
        { CC1101_MDMCFG3,  0xF8 }, /* DRATE_M=248 -> ~49.99kbps */
        { CC1101_MDMCFG2,  0x03 }, /* 2-FSK, 30/32 sync */
        { CC1101_MDMCFG1,  0x23 }, /* 4-byte preamble, CHANSPC_E=3 */
        { CC1101_MDMCFG0,  0x7A }, /* CHANSPC_M=122 -> ~299.93kHz */
        { CC1101_DEVIATN,  0x35 }, /* ~20.63kHz */

        /* Main radio control */
        { CC1101_MCSM2,    0x07 },
        { CC1101_MCSM1,    0x00 }, /* RX/TX end -> IDLE */
        { CC1101_MCSM0,    0x18 }, /* auto-calibrate on IDLE->RX/TX */

        /* FS / AGC */
        { CC1101_FOCCFG,   0x16 },
        { CC1101_BSCFG,    0x6C },
        { CC1101_AGCCTRL2, 0x43 },
        { CC1101_AGCCTRL1, 0x40 },
        { CC1101_AGCCTRL0, 0x91 },

        /* WOR defaults */
        { CC1101_WOREVT1,  0x87 },
        { CC1101_WOREVT0,  0x6B },
        { CC1101_WORCTRL,  0xFB },

        /* Front-end / calibration / test */
        { CC1101_FREND1,   0x56 },
        { CC1101_FREND0,   0x10 },
        { CC1101_FSCAL3,   0xE9 },
        { CC1101_FSCAL2,   0x2A },
        { CC1101_FSCAL1,   0x00 },
        { CC1101_FSCAL0,   0x1F },
        { CC1101_RCCTRL1,  0x41 },
        { CC1101_RCCTRL0,  0x00 },
        { CC1101_FSTEST,   0x59 },
        { CC1101_PTEST,    0x7F },
        { CC1101_AGCTEST,  0x3F },
        { CC1101_TEST2,    0x81 },
        { CC1101_TEST1,    0x35 },
        { CC1101_TEST0,    0x09 }
    };

    uint8_t pa = 0x12; /* low-power startup point */

    if (CC1101_Reset(dev) != CC1101_OK) {
        return CC1101_ERR_TIMEOUT;
    }

    for (uint32_t i = 0U; i < (sizeof(cfg) / sizeof(cfg[0])); i++) {
        if (cc1101_write_reg(dev, cfg[i].reg, cfg[i].val) != CC1101_OK) {
            return CC1101_ERR_SPI;
        }
    }

    if (cc1101_write_burst(dev, CC1101_PATABLE, &pa, 1U) != CC1101_OK) {
        return CC1101_ERR_SPI;
    }

    if (CC1101_FlushRx(dev) != CC1101_OK) {
        return CC1101_ERR_TIMEOUT;
    }
    if (CC1101_FlushTx(dev) != CC1101_OK) {
        return CC1101_ERR_TIMEOUT;
    }

    if (cc1101_strobe(dev, CC1101_SCAL, NULL) != CC1101_OK) {
        return CC1101_ERR_SPI;
    }
    HAL_Delay(2U);

    return CC1101_OK;
}

int16_t CC1101_ReadRSSI_dBm(CC1101_HandleTypeDef *dev)
{
    uint8_t raw = 0U;
    int16_t rssi_dec;

    if (cc1101_read_status(dev, CC1101_RSSI, &raw) != CC1101_OK) {
        return -127;
    }

    if (raw >= 128U) {
        rssi_dec = (int16_t)raw - 256;
    } else {
        rssi_dec = (int16_t)raw;
    }

    return (int16_t)(rssi_dec / 2) - g_rssi_offset_db;
}

bool CC1101_ChannelClear(CC1101_HandleTypeDef *dev,
                         uint8_t ch,
                         int16_t threshold_dBm,
                         uint32_t listen_ms)
{
    if (CC1101_SetChannel(dev, ch) != CC1101_OK) {
        return false;
    }

    if (CC1101_FlushRx(dev) != CC1101_OK) {
        return false;
    }

    if (cc1101_strobe(dev, CC1101_SRX, NULL) != CC1101_OK) {
        return false;
    }

    HAL_Delay(listen_ms);

    int16_t rssi = CC1101_ReadRSSI_dBm(dev);

    (void)CC1101_Idle(dev);

    return (rssi < threshold_dBm);
}

int8_t CC1101_FindFreeChannel(CC1101_HandleTypeDef *dev,
                              int16_t threshold_dBm,
                              uint32_t listen_ms)
{
    for (uint8_t ch = 0U; ch < 10U; ch++) {
        if (CC1101_ChannelClear(dev, ch, threshold_dBm, listen_ms)) {
            return (int8_t)ch;
        }
    }
    return -1;
}

CC1101_Status CC1101_EnterRx(CC1101_HandleTypeDef *dev, uint8_t ch)
{
    if (CC1101_SetChannel(dev, ch) != CC1101_OK) {
        return CC1101_ERR_PARAM;
    }

    if (CC1101_FlushRx(dev) != CC1101_OK) {
        return CC1101_ERR_TIMEOUT;
    }

    if (cc1101_strobe(dev, CC1101_SRX, NULL) != CC1101_OK) {
        return CC1101_ERR_SPI;
    }

    return cc1101_wait_state(dev, CC1101_MARCSTATE_RX, 20U);
}

CC1101_Status CC1101_SendPacketLBT(CC1101_HandleTypeDef *dev,
                                   uint8_t ch,
                                   const uint8_t *data,
                                   uint8_t len,
                                   int16_t cca_threshold_dBm,
                                   uint32_t listen_ms,
                                   uint32_t tx_timeout_ms)
{
    uint8_t fifo[1U + CC1101_PKT_MAX_LEN];

    if ((data == NULL) || (len == 0U) || (len > CC1101_PKT_MAX_LEN)) {
        return CC1101_ERR_PARAM;
    }

    if (!CC1101_ChannelClear(dev, ch, cca_threshold_dBm, listen_ms)) {
        return CC1101_ERR_CCA;
    }

    if (CC1101_FlushTx(dev) != CC1101_OK) {
        return CC1101_ERR_TIMEOUT;
    }

    fifo[0] = len;
    for (uint8_t i = 0U; i < len; i++) {
        fifo[1U + i] = data[i];
    }

    if (cc1101_write_burst(dev, CC1101_TXFIFO, fifo, (uint8_t)(len + 1U)) != CC1101_OK) {
        return CC1101_ERR_SPI;
    }

    if (cc1101_strobe(dev, CC1101_STX, NULL) != CC1101_OK) {
        return CC1101_ERR_SPI;
    }

    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < tx_timeout_ms) {
        uint8_t st = cc1101_get_marcstate(dev);

        if (st == CC1101_MARCSTATE_IDLE) {
            return CC1101_OK;
        }

        if (st == CC1101_MARCSTATE_TXFIFO_UNDERFLOW) {
            (void)CC1101_FlushTx(dev);
            return CC1101_ERR_STATE;
        }
    }

    (void)CC1101_Idle(dev);
    return CC1101_ERR_TIMEOUT;
}

CC1101_Status CC1101_ReadPacket(CC1101_HandleTypeDef *dev,
                                uint8_t *buf,
                                uint8_t *len,
                                uint8_t max_len)
{
    uint8_t pkt_len = 0U;
    uint8_t status_bytes[2] = {0U, 0U};
    uint8_t rx = 0U;

    if ((buf == NULL) || (len == NULL) || (max_len == 0U)) {
        return CC1101_ERR_PARAM;
    }

    /*
     * Call this after GDO0 IRQ (IOCFG0 = 0x07) or after confirming packet complete.
     * Reads length + payload + appended status in one burst.
     */

    cc1101_cs_low(dev);
    if (cc1101_wait_miso_low(dev, 1000U) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_TIMEOUT;
    }

    if (cc1101_spi_xfer(dev, (uint8_t)(CC1101_RXFIFO | CC1101_READ_BURST), &rx) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_SPI;
    }

    if (cc1101_spi_xfer(dev, 0xFFU, &pkt_len) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_SPI;
    }

    if ((pkt_len == 0U) || (pkt_len > CC1101_PKT_MAX_LEN) || (pkt_len > max_len)) {
        cc1101_cs_high(dev);
        (void)CC1101_FlushRx(dev);
        return CC1101_ERR_RX;
    }

    for (uint8_t i = 0U; i < pkt_len; i++) {
        if (cc1101_spi_xfer(dev, 0xFFU, &buf[i]) != CC1101_OK) {
            cc1101_cs_high(dev);
            return CC1101_ERR_SPI;
        }
    }

    if (cc1101_spi_xfer(dev, 0xFFU, &status_bytes[0]) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_SPI;
    }
    if (cc1101_spi_xfer(dev, 0xFFU, &status_bytes[1]) != CC1101_OK) {
        cc1101_cs_high(dev);
        return CC1101_ERR_SPI;
    }

    cc1101_cs_high(dev);

    if ((status_bytes[1] & 0x80U) == 0U) {
        (void)CC1101_FlushRx(dev);
        return CC1101_ERR_RX;
    }

    *len = pkt_len;
    return CC1101_OK;
}

/* -------------------------------------------------------------------------- */
/* power management                                                            */
/* -------------------------------------------------------------------------- */

CC1101_Status CC1101_EnterSleep(CC1101_HandleTypeDef *dev)
{
    if (CC1101_Idle(dev) != CC1101_OK) {
        return CC1101_ERR_TIMEOUT;
    }
    return cc1101_strobe(dev, CC1101_SPWD, NULL);
}

CC1101_Status CC1101_WakeFromSleep(CC1101_HandleTypeDef *dev)
{
    cc1101_cs_low(dev);
    cc1101_delay_us(10U);
    cc1101_cs_high(dev);

    /* Wait for crystal oscillator to stabilize (~150 us typical) */
    cc1101_delay_us(200U);

    if (cc1101_wait_miso_low(dev, 5000U) != CC1101_OK) {
        return CC1101_ERR_TIMEOUT;
    }

    return cc1101_wait_state(dev, CC1101_MARCSTATE_IDLE, 50U);
}

/* -------------------------------------------------------------------------- */
/* PA table                                                                    */
/* -------------------------------------------------------------------------- */

CC1101_Status CC1101_SetPATable(CC1101_HandleTypeDef *dev,
                                const uint8_t *pa_table,
                                uint8_t len)
{
    if ((pa_table == NULL) || (len == 0U) || (len > 8U)) {
        return CC1101_ERR_PARAM;
    }

    if (CC1101_Idle(dev) != CC1101_OK) {
        return CC1101_ERR_TIMEOUT;
    }

    return cc1101_write_burst(dev, CC1101_PATABLE, pa_table, len);
}

/* -------------------------------------------------------------------------- */
/* LBT with channel agility & random back-off (Master only)                    */
/* -------------------------------------------------------------------------- */

static uint32_t cc1101_pseudo_random(uint32_t seed)
{
    /* Simple LFSR-based PRNG for back-off jitter */
    seed ^= seed << 13U;
    seed ^= seed >> 17U;
    seed ^= seed << 5U;
    return seed;
}

CC1101_Status CC1101_SendPacketLBT_Agile(CC1101_HandleTypeDef *dev,
                                         const uint8_t *data,
                                         uint8_t len,
                                         int16_t cca_threshold_dBm,
                                         uint32_t listen_ms,
                                         uint32_t tx_timeout_ms,
                                         uint8_t max_retries,
                                         uint8_t *used_channel)
{
    uint32_t rng_state = HAL_GetTick();

    if ((data == NULL) || (len == 0U) || (len > CC1101_PKT_MAX_LEN)) {
        return CC1101_ERR_PARAM;
    }

    for (uint8_t attempt = 0U; attempt < max_retries; attempt++) {
        /* Try to find a free channel across all MICS channels */
        int8_t ch = CC1101_FindFreeChannel(dev, cca_threshold_dBm, listen_ms);

        if (ch >= 0) {
            CC1101_Status st = CC1101_SendPacketLBT(dev,
                                                    (uint8_t)ch,
                                                    data, len,
                                                    cca_threshold_dBm,
                                                    listen_ms,
                                                    tx_timeout_ms);
            if (st == CC1101_OK) {
                if (used_channel != NULL) {
                    *used_channel = (uint8_t)ch;
                }
                return CC1101_OK;
            }
        }

        /* Random back-off before retry */
        rng_state = cc1101_pseudo_random(rng_state);
        uint32_t backoff_ms = (rng_state % CC1101_MICS_MAX_BACKOFF_MS) + 1U;
        HAL_Delay(backoff_ms);
    }

    return CC1101_ERR_CCA;
}

/* -------------------------------------------------------------------------- */
/* RX with timeout                                                             */
/* -------------------------------------------------------------------------- */

CC1101_Status CC1101_WaitAndReadPacket(CC1101_HandleTypeDef *dev,
                                       uint8_t ch,
                                       uint8_t *buf,
                                       uint8_t *len,
                                       uint8_t max_len,
                                       uint32_t timeout_ms)
{
    if (CC1101_EnterRx(dev, ch) != CC1101_OK) {
        return CC1101_ERR_STATE;
    }

    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        /* Check GDO0 for packet received (IOCFG0 = 0x07: CRC OK) */
        if ((dev->gdo0_port != NULL) &&
            (HAL_GPIO_ReadPin(dev->gdo0_port, dev->gdo0_pin) == GPIO_PIN_SET)) {
            return CC1101_ReadPacket(dev, buf, len, max_len);
        }

        /* Fallback: check RXBYTES if GDO0 not wired */
        if (dev->gdo0_port == NULL) {
            uint8_t rxbytes = 0U;
            (void)cc1101_read_status(dev, CC1101_RXBYTES, &rxbytes);
            if ((rxbytes & 0x7FU) > 0U) {
                return CC1101_ReadPacket(dev, buf, len, max_len);
            }
        }
    }

    (void)CC1101_Idle(dev);
    return CC1101_ERR_TIMEOUT;
}