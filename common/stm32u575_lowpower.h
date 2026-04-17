#ifndef __STM32U575_LOWPOWER_H__
#define __STM32U575_LOWPOWER_H__

#include "stm32u5xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * STM32U575 Ultra-Low Power Management
 *
 * Power modes used:
 *   Run         : Normal operation (4 MHz MSI for low-power run)
 *   LP Run      : Low-power run with MSI at 100 kHz~4 MHz
 *   Stop 2      : ~2 uA, SRAM retained, peripherals off, wake by EXTI/RTC
 *   Standby     : ~0.3 uA, SRAM lost, wake by WKUP pin / RTC
 *   Shutdown     : ~0.1 uA, everything off, wake by WKUP pin only
 *
 * For implant (Slave): Stop 2 is the primary idle mode.
 *   - CC2500 GDO0 -> EXTI line wakes MCU on beacon reception
 *   - RTC can provide a periodic self-check wake-up
 *
 * For programmer (Master): LP Run or normal Run during communication.
 * ========================================================================== */

/* ---------- wake-up sources ---------- */
typedef enum {
    LP_WAKEUP_EXTI_GDO0          = 0x01U,  /* CC2500 GDO0 (beacon/packet received) */
    LP_WAKEUP_RTC_ALARM           = 0x02U,  /* RTC alarm for periodic check / session timeout */
    LP_WAKEUP_WKUP_PIN            = 0x04U,  /* STM32 PWR WKUP (WUF1..WUF8) */
    LP_WAKEUP_EXTI_CC1101_GDO0   = 0x08U,  /* CC1101 GDO0 (MICS packet received, CRC OK) */
} LP_WakeupSource;

/* ---------- SPI power state ---------- */
typedef struct {
    SPI_HandleTypeDef *hspi_cc1101;
    SPI_HandleTypeDef *hspi_cc2500;

    /* CC2500 GDO0 used as EXTI wake-up (deep-sleep beacon detection) */
    GPIO_TypeDef *gdo0_port;
    uint16_t      gdo0_pin;
    IRQn_Type     gdo0_irqn;

    /* CC1101 GDO0 used as EXTI wake-up during MICS session RX (Stop 2) */
    GPIO_TypeDef *cc1101_gdo0_port;
    uint16_t      cc1101_gdo0_pin;
    IRQn_Type     cc1101_gdo0_irqn;

    /* RTC handle for periodic wake-up / session timeout */
    RTC_HandleTypeDef *hrtc;
} LP_HandleTypeDef;

/* ---------- API ---------- */

/* Initialize low-power infrastructure (configure EXTI, voltage scaling) */
void LP_Init(LP_HandleTypeDef *hlp);

/* Enter Stop 2 mode - wakes on configured EXTI or RTC
 * Returns the wake-up source that triggered exit */
LP_WakeupSource LP_EnterStop2(LP_HandleTypeDef *hlp);

/* Enter Stop 3 mode (~1 uA, deeper than Stop 2, SRAM retained)
 * Wakes on EXTI or RTC; uses same restore path as Stop 2 */
LP_WakeupSource LP_EnterStop3(LP_HandleTypeDef *hlp);

/* Enter Standby mode - only WKUP pin or RTC can wake
 * WARNING: SRAM content is lost */
void LP_EnterStandby(LP_HandleTypeDef *hlp);

/* Restore system clocks and peripherals after Stop 2 wake-up */
void LP_RestoreFromStop2(LP_HandleTypeDef *hlp);

/* Configure RTC wake-up timer (seconds, 1..65535) */
void LP_ConfigureRTCWakeUp(LP_HandleTypeDef *hlp, uint32_t seconds);

/* Configure RTC wake-up timer with millisecond granularity (1..32000 ms).
 * Uses RTCCLK/16 = 2048 Hz from LSE 32768 Hz → ~488 µs resolution.
 * Allows Stop 2 sleep on short intervals such as stream frame periods (≥20 ms)
 * without busy-polling at 4 MHz between transmissions. */
void LP_ConfigureRTCWakeUp_ms(LP_HandleTypeDef *hlp, uint32_t ms);

/* Disable RTC wake-up */
void LP_DisableRTCWakeUp(LP_HandleTypeDef *hlp);

/* Enable / disable STM32 hardware WKUP pin(s) for Stop 3 / Standby.
 * `wake_up_pin` is a HAL constant such as PWR_WAKEUP_PIN1_HIGH_0 (PA0).
 * Call Enable before LP_EnterStop3 when ship mode needs a physical exit;
 * call Disable when returning to normal Stop 2 + radio WOR. */
void LP_EnableWakeUpPin(uint32_t wake_up_pin);
void LP_DisableWakeUpPin(uint32_t wake_up_pin);

/* Disable SPI peripherals before sleep to reduce leakage */
void LP_SPI_Suspend(LP_HandleTypeDef *hlp);

/* Re-enable SPI peripherals after wake-up */
void LP_SPI_Resume(LP_HandleTypeDef *hlp);

/* Configure voltage scaling for minimum power in active mode */
void LP_SetVoltageScaling_LowPower(void);
void LP_SetVoltageScaling_Normal(void);

/* Switch system clock to MSI at given frequency for low-power run */
void LP_SwitchToMSI(uint32_t msi_range);

#ifdef __cplusplus
}
#endif

#endif /* __STM32U575_LOWPOWER_H__ */
