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
 * Power modes (STM32U575 Q-series):
 *   Run         : ~4 mA  @ 4 MHz MSI, VOS4  (active processing)
 *   LP Run      : ~200 uA @ 100 kHz MSI, VOS4 (light housekeeping)
 *   Stop 2      : ~1.6 uA, all SRAMs retained, wake by EXTI / RTC
 *   Stop 3      : ~1.2 uA, SRAM2 retained only, wake by EXTI / RTC
 *   Standby     : ~0.3 uA, SRAM lost, wake by WKUP pin / RTC alarm
 *   Shutdown    : ~0.1 uA, everything off, wake by WKUP pin only
 *
 * For Slave implant — two operating phases:
 *
 *   IDLE phase (waiting for wake-up beacon):
 *     MCU   : Stop 2 (~1.6 uA)
 *     CC2500: WOR mode (~70 uA avg at 500 ms sniff interval)
 *     CC1101: SLEEP (SPWD, ~200 nA)
 *     Wake  : CC2500 GDO0 EXTI (CRC OK on beacon) or RTC 60 s self-check
 *
 *   SESSION phase (active MICS data exchange):
 *     MCU   : Stop 2 between frames (~1.6 uA)
 *     CC1101: RX mode (~15 mA) — dominant current; minimised by short session
 *     CC2500: IDLE (not in WOR)
 *     Wake  : CC1101 GDO0 EXTI (CRC OK on command frame) or RTC session timeout
 *
 * Session timeout replaces HAL_GetTick() (which stops during Stop 2) with
 * the RTC wake-up timer so elapsed time is tracked correctly.
 *
 * For Master (Programmer): LP Run or normal Run during communication.
 *   ICACHE enabled for faster execution → quicker return to sleep.
 * ========================================================================== */

/* ---------- wake-up sources ---------- */
typedef enum {
    LP_WAKEUP_EXTI_GDO0         = 0x01U,  /* CC2500 GDO0 — beacon/WOR packet */
    LP_WAKEUP_RTC_ALARM         = 0x02U,  /* RTC wake-up timer               */
    LP_WAKEUP_WKUP_PIN          = 0x04U,  /* physical WKUP pin               */
    LP_WAKEUP_EXTI_CC1101_GDO0  = 0x08U,  /* CC1101 GDO0 — MICS session frame */
} LP_WakeupSource;

/* ---------- low-power handle ---------- */
typedef struct {
    SPI_HandleTypeDef *hspi_cc1101;
    SPI_HandleTypeDef *hspi_cc2500;

    /* CC2500 GDO0: EXTI wake-up for WOR beacon reception */
    GPIO_TypeDef *gdo0_port;
    uint16_t      gdo0_pin;
    IRQn_Type     gdo0_irqn;

    /* CC1101 GDO0: EXTI wake-up for MICS session frame reception
     * Set to NULL if CC1101 GDO0 is not wired to an EXTI-capable pin. */
    GPIO_TypeDef *cc1101_gdo0_port;
    uint16_t      cc1101_gdo0_pin;
    IRQn_Type     cc1101_gdo0_irqn;

    /* RTC: periodic self-check (IDLE phase) and session timeout (SESSION phase) */
    RTC_HandleTypeDef *hrtc;
} LP_HandleTypeDef;

/* ---------- API ---------- */

/* Initialize low-power infrastructure.
 * Configures EXTI for CC2500 GDO0 (always) and CC1101 GDO0 (if port != NULL),
 * enables ULP mode, disables fast wake-up, enables ICACHE. */
void LP_Init(LP_HandleTypeDef *hlp);

/* Enter Stop 2 — all SRAMs retained, wake by EXTI or RTC.
 * SPI peripherals are suspended before entry and restored on exit.
 * Returns a bitmask of LP_WakeupSource flags. */
LP_WakeupSource LP_EnterStop2(LP_HandleTypeDef *hlp);

/* Enter Stop 3 — only SRAM2 retained; deepest stop with RAM.
 * Use only when the application state fits in SRAM2 (64 KB).
 * Returns a bitmask of LP_WakeupSource flags. */
LP_WakeupSource LP_EnterStop3(LP_HandleTypeDef *hlp);

/* Enter Standby — SRAM lost, wake by WKUP pin or RTC alarm only.
 * WARNING: execution resumes from reset vector on wake-up. */
void LP_EnterStandby(LP_HandleTypeDef *hlp);

/* Restore system clocks and SPI peripherals after Stop 2 / Stop 3 wake-up */
void LP_RestoreFromStop2(LP_HandleTypeDef *hlp);

/* Configure RTC wake-up timer (1..65535 seconds).
 * Used for periodic self-check in IDLE phase and session timeout in
 * SESSION phase.  Call LP_DisableRTCWakeUp() when no longer needed. */
void LP_ConfigureRTCWakeUp(LP_HandleTypeDef *hlp, uint32_t seconds);

/* Disable RTC wake-up */
void LP_DisableRTCWakeUp(LP_HandleTypeDef *hlp);

/* Suspend / resume SPI peripherals around stop modes */
void LP_SPI_Suspend(LP_HandleTypeDef *hlp);
void LP_SPI_Resume(LP_HandleTypeDef *hlp);

/* Voltage scaling — call LowPower before sleep, Normal before high-speed ops */
void LP_SetVoltageScaling_LowPower(void);  /* VOS4, max 24 MHz  */
void LP_SetVoltageScaling_Normal(void);    /* VOS2, max 80 MHz  */

/* Switch SYSCLK to MSI at msi_range (RCC_MSIRANGE_x constant) */
void LP_SwitchToMSI(uint32_t msi_range);

#ifdef __cplusplus
}
#endif

#endif /* __STM32U575_LOWPOWER_H__ */
