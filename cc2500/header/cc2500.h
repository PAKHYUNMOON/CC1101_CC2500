#ifndef __CC2500_H__
#define __CC2500_H__

#include "stm32u5xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- user-tunable ---------- */
#define CC2500_PKT_MAX_LEN                 61U
#define CC2500_SPI_TIMEOUT_MS              100U
#define CC2500_DEFAULT_RSSI_OFFSET_DB      74

/* ---------- return codes ---------- */
typedef enum
{
    CC2500_OK = 0,
    CC2500_ERR_PARAM   = -1,
    CC2500_ERR_TIMEOUT = -2,
    CC2500_ERR_SPI     = -3,
    CC2500_ERR_STATE   = -4,
    CC2500_ERR_CCA     = -5,
    CC2500_ERR_RX      = -6
} CC2500_Status;

/* ---------- device handle ---------- */
typedef struct
{
    SPI_HandleTypeDef *hspi;

    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;

    GPIO_TypeDef *miso_port;   /* optional but strongly recommended for CHIP_RDYn wait */
    uint16_t      miso_pin;

    GPIO_TypeDef *gdo0_port;   /* optional, for RX IRQ */
    uint16_t      gdo0_pin;
} CC2500_HandleTypeDef;

/* ---------- CC2500 registers ---------- */
#define CC2500_IOCFG2        0x00
#define CC2500_IOCFG1        0x01
#define CC2500_IOCFG0        0x02
#define CC2500_FIFOTHR       0x03
#define CC2500_SYNC1         0x04
#define CC2500_SYNC0         0x05
#define CC2500_PKTLEN        0x06
#define CC2500_PKTCTRL1      0x07
#define CC2500_PKTCTRL0      0x08
#define CC2500_ADDR          0x09
#define CC2500_CHANNR        0x0A
#define CC2500_FSCTRL1       0x0B
#define CC2500_FSCTRL0       0x0C
#define CC2500_FREQ2         0x0D
#define CC2500_FREQ1         0x0E
#define CC2500_FREQ0         0x0F
#define CC2500_MDMCFG4       0x10
#define CC2500_MDMCFG3       0x11
#define CC2500_MDMCFG2       0x12
#define CC2500_MDMCFG1       0x13
#define CC2500_MDMCFG0       0x14
#define CC2500_DEVIATN       0x15
#define CC2500_MCSM2         0x16
#define CC2500_MCSM1         0x17
#define CC2500_MCSM0         0x18
#define CC2500_FOCCFG        0x19
#define CC2500_BSCFG         0x1A
#define CC2500_AGCCTRL2      0x1B
#define CC2500_AGCCTRL1      0x1C
#define CC2500_AGCCTRL0      0x1D
#define CC2500_WOREVT1       0x1E
#define CC2500_WOREVT0       0x1F
#define CC2500_WORCTRL       0x20
#define CC2500_FREND1        0x21
#define CC2500_FREND0        0x22
#define CC2500_FSCAL3        0x23
#define CC2500_FSCAL2        0x24
#define CC2500_FSCAL1        0x25
#define CC2500_FSCAL0        0x26
#define CC2500_RCCTRL1       0x27
#define CC2500_RCCTRL0       0x28
#define CC2500_FSTEST        0x29
#define CC2500_PTEST         0x2A
#define CC2500_AGCTEST       0x2B
#define CC2500_TEST2         0x2C
#define CC2500_TEST1         0x2D
#define CC2500_TEST0         0x2E
#define CC2500_PATABLE       0x3E
#define CC2500_TXFIFO        0x3F
#define CC2500_RXFIFO        0x3F

/* ---------- status registers ---------- */
#define CC2500_RSSI          0x34
#define CC2500_MARCSTATE     0x35
#define CC2500_PKTSTATUS     0x38
#define CC2500_TXBYTES       0x3A
#define CC2500_RXBYTES       0x3B

/* ---------- command strobes ---------- */
#define CC2500_SRES          0x30
#define CC2500_SFSTXON       0x31
#define CC2500_SXOFF         0x32
#define CC2500_SCAL          0x33
#define CC2500_SRX           0x34
#define CC2500_STX           0x35
#define CC2500_SIDLE         0x36
#define CC2500_SAFC          0x37
#define CC2500_SWOR          0x38
#define CC2500_SPWD          0x39
#define CC2500_SFRX          0x3A
#define CC2500_SFTX          0x3B
#define CC2500_SWORRST       0x3C
#define CC2500_SNOP          0x3D

/* ---------- access flags ---------- */
#define CC2500_WRITE_BURST   0x40
#define CC2500_READ_SINGLE   0x80
#define CC2500_READ_BURST    0xC0

/* ---------- MARCSTATE values ---------- */
#define CC2500_MARCSTATE_IDLE              0x01
#define CC2500_MARCSTATE_RX                0x0D
#define CC2500_MARCSTATE_TX                0x13
#define CC2500_MARCSTATE_RXFIFO_OVERFLOW   0x11
#define CC2500_MARCSTATE_TXFIFO_UNDERFLOW  0x16

/* ---------- WOR configuration ---------- */
#define CC2500_WOR_RES_1_PERIOD            0x00   /* t_Event0 resolution: 1 period */
#define CC2500_WOR_RES_32_PERIODS          0x01   /* 2^5 periods */
#define CC2500_WOR_RES_1024_PERIODS        0x02   /* 2^10 periods */
#define CC2500_WOR_RES_32768_PERIODS       0x03   /* 2^15 periods */

#define CC2500_WAKEUP_CHANNEL              0U     /* fixed channel for wake-up */
#define CC2500_WAKEUP_BEACON_REPEAT        10U    /* number of beacon repetitions */
#define CC2500_WAKEUP_BEACON_INTERVAL_MS   15U    /* interval between beacons */

/* ---------- API ---------- */
void CC2500_DWT_DelayInit(void);
void CC2500_SetRSSIOffset(int16_t offset_db);

CC2500_Status CC2500_Reset(CC2500_HandleTypeDef *dev);
CC2500_Status CC2500_InitMICSLike26MHz(CC2500_HandleTypeDef *dev);
CC2500_Status CC2500_InitWakeUp26MHz(CC2500_HandleTypeDef *dev);

CC2500_Status CC2500_Idle(CC2500_HandleTypeDef *dev);
CC2500_Status CC2500_FlushRx(CC2500_HandleTypeDef *dev);
CC2500_Status CC2500_FlushTx(CC2500_HandleTypeDef *dev);

CC2500_Status CC2500_SetChannel(CC2500_HandleTypeDef *dev, uint8_t ch); /* ch: 0~9 */
CC2500_Status CC2500_EnterRx(CC2500_HandleTypeDef *dev, uint8_t ch);

int16_t CC2500_ReadRSSI_dBm(CC2500_HandleTypeDef *dev);
bool CC2500_ChannelClear(CC2500_HandleTypeDef *dev,
                         uint8_t ch,
                         int16_t threshold_dBm,
                         uint32_t listen_ms);
int8_t CC2500_FindFreeChannel(CC2500_HandleTypeDef *dev,
                              int16_t threshold_dBm,
                              uint32_t listen_ms);

CC2500_Status CC2500_SendPacketLBT(CC2500_HandleTypeDef *dev,
                                   uint8_t ch,
                                   const uint8_t *data,
                                   uint8_t len,
                                   int16_t cca_threshold_dBm,
                                   uint32_t listen_ms,
                                   uint32_t tx_timeout_ms);

CC2500_Status CC2500_ReadPacket(CC2500_HandleTypeDef *dev,
                                uint8_t *buf,
                                uint8_t *len,
                                uint8_t max_len);

/* ---------- power management ---------- */
CC2500_Status CC2500_EnterSleep(CC2500_HandleTypeDef *dev);
CC2500_Status CC2500_WakeFromSleep(CC2500_HandleTypeDef *dev);

/* ---------- WOR (Wake-on-Radio) for Slave ---------- */
CC2500_Status CC2500_EnterWOR(CC2500_HandleTypeDef *dev);
CC2500_Status CC2500_ConfigureWOR(CC2500_HandleTypeDef *dev,
                                  uint8_t wor_res,
                                  uint16_t event0_timeout);

/* ---------- Wake-up beacon for Master ----------
 * Sends `repeat_count` copies of the pre-built MAC PDU on the wake-up
 * channel with CC2500_WAKEUP_BEACON_INTERVAL_MS spacing.
 * The PDU content (FCF|SEQ|SESS|PAYLOAD) is built by the caller using
 * Proto_BuildPacket() + Proto_BuildBeaconPayload() from protocol.h.
 */
CC2500_Status CC2500_SendWakeUpBeacon(CC2500_HandleTypeDef *dev,
                                      const uint8_t *pkt,
                                      uint8_t pkt_len,
                                      uint8_t repeat_count);

/* ---------- send / receive (no LBT, wake-up channel) ---------- */
CC2500_Status CC2500_SendPacketDirect(CC2500_HandleTypeDef *dev,
                                      uint8_t ch,
                                      const uint8_t *data,
                                      uint8_t len,
                                      uint32_t tx_timeout_ms);

CC2500_Status CC2500_WaitAndReadPacket(CC2500_HandleTypeDef *dev,
                                       uint8_t ch,
                                       uint8_t *buf,
                                       uint8_t *len,
                                       uint8_t max_len,
                                       uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __CC2500_H__ */