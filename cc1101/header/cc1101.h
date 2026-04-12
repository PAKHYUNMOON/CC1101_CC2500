#ifndef __CC1101_H__
#define __CC1101_H__

#include "stm32u5xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- user-tunable ---------- */
#define CC1101_PKT_MAX_LEN                 61U
#define CC1101_SPI_TIMEOUT_MS              100U
#define CC1101_DEFAULT_RSSI_OFFSET_DB      74

/* ---------- return codes ---------- */
typedef enum
{
    CC1101_OK = 0,
    CC1101_ERR_PARAM   = -1,
    CC1101_ERR_TIMEOUT = -2,
    CC1101_ERR_SPI     = -3,
    CC1101_ERR_STATE   = -4,
    CC1101_ERR_CCA     = -5,
    CC1101_ERR_RX      = -6
} CC1101_Status;

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
} CC1101_HandleTypeDef;

/* ---------- CC1101 registers ---------- */
#define CC1101_IOCFG2        0x00
#define CC1101_IOCFG1        0x01
#define CC1101_IOCFG0        0x02
#define CC1101_FIFOTHR       0x03
#define CC1101_SYNC1         0x04
#define CC1101_SYNC0         0x05
#define CC1101_PKTLEN        0x06
#define CC1101_PKTCTRL1      0x07
#define CC1101_PKTCTRL0      0x08
#define CC1101_ADDR          0x09
#define CC1101_CHANNR        0x0A
#define CC1101_FSCTRL1       0x0B
#define CC1101_FSCTRL0       0x0C
#define CC1101_FREQ2         0x0D
#define CC1101_FREQ1         0x0E
#define CC1101_FREQ0         0x0F
#define CC1101_MDMCFG4       0x10
#define CC1101_MDMCFG3       0x11
#define CC1101_MDMCFG2       0x12
#define CC1101_MDMCFG1       0x13
#define CC1101_MDMCFG0       0x14
#define CC1101_DEVIATN       0x15
#define CC1101_MCSM2         0x16
#define CC1101_MCSM1         0x17
#define CC1101_MCSM0         0x18
#define CC1101_FOCCFG        0x19
#define CC1101_BSCFG         0x1A
#define CC1101_AGCCTRL2      0x1B
#define CC1101_AGCCTRL1      0x1C
#define CC1101_AGCCTRL0      0x1D
#define CC1101_WOREVT1       0x1E
#define CC1101_WOREVT0       0x1F
#define CC1101_WORCTRL       0x20
#define CC1101_FREND1        0x21
#define CC1101_FREND0        0x22
#define CC1101_FSCAL3        0x23
#define CC1101_FSCAL2        0x24
#define CC1101_FSCAL1        0x25
#define CC1101_FSCAL0        0x26
#define CC1101_RCCTRL1       0x27
#define CC1101_RCCTRL0       0x28
#define CC1101_FSTEST        0x29
#define CC1101_PTEST         0x2A
#define CC1101_AGCTEST       0x2B
#define CC1101_TEST2         0x2C
#define CC1101_TEST1         0x2D
#define CC1101_TEST0         0x2E
#define CC1101_PATABLE       0x3E
#define CC1101_TXFIFO        0x3F
#define CC1101_RXFIFO        0x3F

/* ---------- status registers ---------- */
#define CC1101_RSSI          0x34
#define CC1101_MARCSTATE     0x35
#define CC1101_PKTSTATUS     0x38
#define CC1101_TXBYTES       0x3A
#define CC1101_RXBYTES       0x3B

/* ---------- command strobes ---------- */
#define CC1101_SRES          0x30
#define CC1101_SFSTXON       0x31
#define CC1101_SXOFF         0x32
#define CC1101_SCAL          0x33
#define CC1101_SRX           0x34
#define CC1101_STX           0x35
#define CC1101_SIDLE         0x36
#define CC1101_SAFC          0x37
#define CC1101_SWOR          0x38
#define CC1101_SPWD          0x39
#define CC1101_SFRX          0x3A
#define CC1101_SFTX          0x3B
#define CC1101_SWORRST       0x3C
#define CC1101_SNOP          0x3D

/* ---------- access flags ---------- */
#define CC1101_WRITE_BURST   0x40
#define CC1101_READ_SINGLE   0x80
#define CC1101_READ_BURST    0xC0

/* ---------- MARCSTATE values ---------- */
#define CC1101_MARCSTATE_IDLE              0x01
#define CC1101_MARCSTATE_RX                0x0D
#define CC1101_MARCSTATE_TX                0x13
#define CC1101_MARCSTATE_RXFIFO_OVERFLOW   0x11
#define CC1101_MARCSTATE_TXFIFO_UNDERFLOW  0x16

/* ---------- API ---------- */
void CC1101_DWT_DelayInit(void);
void CC1101_SetRSSIOffset(int16_t offset_db);

CC1101_Status CC1101_Reset(CC1101_HandleTypeDef *dev);
CC1101_Status CC1101_InitMICSLike26MHz(CC1101_HandleTypeDef *dev);

CC1101_Status CC1101_Idle(CC1101_HandleTypeDef *dev);
CC1101_Status CC1101_FlushRx(CC1101_HandleTypeDef *dev);
CC1101_Status CC1101_FlushTx(CC1101_HandleTypeDef *dev);

CC1101_Status CC1101_SetChannel(CC1101_HandleTypeDef *dev, uint8_t ch); /* ch: 0~9 */
CC1101_Status CC1101_EnterRx(CC1101_HandleTypeDef *dev, uint8_t ch);

int16_t CC1101_ReadRSSI_dBm(CC1101_HandleTypeDef *dev);
bool CC1101_ChannelClear(CC1101_HandleTypeDef *dev,
                         uint8_t ch,
                         int16_t threshold_dBm,
                         uint32_t listen_ms);
int8_t CC1101_FindFreeChannel(CC1101_HandleTypeDef *dev,
                              int16_t threshold_dBm,
                              uint32_t listen_ms);

CC1101_Status CC1101_SendPacketLBT(CC1101_HandleTypeDef *dev,
                                   uint8_t ch,
                                   const uint8_t *data,
                                   uint8_t len,
                                   int16_t cca_threshold_dBm,
                                   uint32_t listen_ms,
                                   uint32_t tx_timeout_ms);

CC1101_Status CC1101_ReadPacket(CC1101_HandleTypeDef *dev,
                                uint8_t *buf,
                                uint8_t *len,
                                uint8_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* __CC1101_H__ */