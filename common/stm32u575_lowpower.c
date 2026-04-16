#include "stm32u575_lowpower.h"

/* ==========================================================================
 * STM32U575 Ultra-Low Power Implementation
 *
 * Target: STM32U575 Q-series (UFBGA132)
 * Key hardware features exploited:
 *   - Stop 2 (~1.6 uA) and Stop 3 (~1.2 uA) with SRAM retention
 *   - PWR voltage scaling VOS4 (Range 4 = lowest, max 24 MHz)
 *   - MSI RC oscillator (no external crystal during stop/run)
 *   - EXTI on two GDO0 lines: CC2500 (WOR beacon) + CC1101 (MICS frame)
 *   - RTC with LSI: periodic self-check AND session timeout
 *   - ICACHE: reduces Flash accesses → shorter active windows → faster sleep
 *   - ULP (Ultra-Low Power): VREFINT off in Stop; saves ~2 uA vs. default
 * ========================================================================== */

/* -------------------------------------------------------------------------- */
/* initialization                                                              */
/* -------------------------------------------------------------------------- */

void LP_Init(LP_HandleTypeDef *hlp)
{
    if (hlp == NULL) {
        return;
    }

    /* Enable PWR clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Enable backup domain access for RTC */
    HAL_PWR_EnableBkUpAccess();

    /* -----------------------------------------------------------------------
     * CC2500 GDO0: rising-edge EXTI for WOR beacon reception
     * Asserts when a packet with CRC OK is received (IOCFG0 = 0x07).
     * --------------------------------------------------------------------- */
    if (hlp->gdo0_port != NULL) {
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin   = hlp->gdo0_pin;
        gpio.Mode  = GPIO_MODE_IT_RISING;
        gpio.Pull  = GPIO_PULLDOWN;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(hlp->gdo0_port, &gpio);

        HAL_NVIC_SetPriority(hlp->gdo0_irqn, 0, 0);
        HAL_NVIC_EnableIRQ(hlp->gdo0_irqn);
    }

    /* -----------------------------------------------------------------------
     * CC1101 GDO0: rising-edge EXTI for MICS session frame reception
     * Same IOCFG0 = 0x07 configuration as CC2500.
     * Only configured when the pin is physically wired to an EXTI line.
     * --------------------------------------------------------------------- */
    if (hlp->cc1101_gdo0_port != NULL) {
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin   = hlp->cc1101_gdo0_pin;
        gpio.Mode  = GPIO_MODE_IT_RISING;
        gpio.Pull  = GPIO_PULLDOWN;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(hlp->cc1101_gdo0_port, &gpio);

        HAL_NVIC_SetPriority(hlp->cc1101_gdo0_irqn, 1, 0); /* lower prio than CC2500 */
        HAL_NVIC_EnableIRQ(hlp->cc1101_gdo0_irqn);
    }

    /* -----------------------------------------------------------------------
     * Ultra-Low Power mode: disables VREFINT in Stop → saves ~2 uA
     * Disable fast wake-up: adds ~3.5 us to exit time, saves power in Stop
     * --------------------------------------------------------------------- */
    HAL_PWREx_EnableUltraLowPowerMode();
    HAL_PWREx_DisableFastWakeUpFromStop();

    /* -----------------------------------------------------------------------
     * ICACHE: enable instruction cache for STM32U575.
     * Reduces Flash wait-state stalls during active processing, shortening
     * the time the MCU runs at full current before returning to Stop.
     * --------------------------------------------------------------------- */
    __HAL_RCC_ICACHE_CLK_ENABLE();
    HAL_ICACHE_Enable();

    /* -----------------------------------------------------------------------
     * SRAM2 retention in Stop modes.
     * On STM32U575, SRAM2 (64 KB) is retained across Stop 2 and Stop 3
     * by default (PWR_CR2_SRAM2_16KBPD = 0).  SRAM1 is also retained in
     * Stop 2 by default.  This explicit call ensures retention is not
     * accidentally disabled by other init code.
     * --------------------------------------------------------------------- */
    HAL_PWREx_EnableRAMsContentStopRetention(PWR_SRAM2_FULL_STOP_RETENTION);
}

/* -------------------------------------------------------------------------- */
/* Stop 2 entry / exit  (all SRAMs retained, ~1.6 uA)                         */
/* -------------------------------------------------------------------------- */

LP_WakeupSource LP_EnterStop2(LP_HandleTypeDef *hlp)
{
    if (hlp == NULL) {
        return 0U;
    }

    LP_SPI_Suspend(hlp);

    /* Clear pending wake-up and EXTI flags before entry */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    if (hlp->gdo0_port != NULL) {
        __HAL_GPIO_EXTI_CLEAR_IT(hlp->gdo0_pin);
    }
    if (hlp->cc1101_gdo0_port != NULL) {
        __HAL_GPIO_EXTI_CLEAR_IT(hlp->cc1101_gdo0_pin);
    }

    /* Suspend SysTick: prevents spurious wake every 1 ms.
     * NOTE: HAL_GetTick() will not advance during Stop; use RTC for elapsed
     * time measurement across Stop entries. */
    HAL_SuspendTick();

    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

    /* --- MCU wakes here --- */

    HAL_ResumeTick();
    LP_RestoreFromStop2(hlp);

    /* Identify wake source */
    LP_WakeupSource src = 0U;

    if ((hlp->gdo0_port != NULL) &&
        (__HAL_GPIO_EXTI_GET_IT(hlp->gdo0_pin) != 0U)) {
        src |= LP_WAKEUP_EXTI_GDO0;
        __HAL_GPIO_EXTI_CLEAR_IT(hlp->gdo0_pin);
    }

    if ((hlp->cc1101_gdo0_port != NULL) &&
        (__HAL_GPIO_EXTI_GET_IT(hlp->cc1101_gdo0_pin) != 0U)) {
        src |= LP_WAKEUP_EXTI_CC1101_GDO0;
        __HAL_GPIO_EXTI_CLEAR_IT(hlp->cc1101_gdo0_pin);
    }

    if ((hlp->hrtc != NULL) &&
        (__HAL_RTC_WAKEUPTIMER_GET_FLAG(hlp->hrtc, RTC_FLAG_WUTF) != 0U)) {
        src |= LP_WAKEUP_RTC_ALARM;
        __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hlp->hrtc, RTC_FLAG_WUTF);
    }

    return src;
}

/* -------------------------------------------------------------------------- */
/* Stop 3 entry / exit  (SRAM2 only retained, ~1.2 uA)                        */
/*                                                                             */
/* Use when the full application state fits in SRAM2 (64 KB on STM32U575).   */
/* SRAM1 (192 KB) and other RAM banks are NOT retained — re-initialise them   */
/* from Flash on wake-up if needed.                                            */
/* -------------------------------------------------------------------------- */

LP_WakeupSource LP_EnterStop3(LP_HandleTypeDef *hlp)
{
    if (hlp == NULL) {
        return 0U;
    }

    LP_SPI_Suspend(hlp);

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    if (hlp->gdo0_port != NULL) {
        __HAL_GPIO_EXTI_CLEAR_IT(hlp->gdo0_pin);
    }
    if (hlp->cc1101_gdo0_port != NULL) {
        __HAL_GPIO_EXTI_CLEAR_IT(hlp->cc1101_gdo0_pin);
    }

    HAL_SuspendTick();

    /* Stop 3: only SRAM2 retained.  On U575 this is the deepest stop mode
     * that preserves any RAM.  LPMS[2:0] = 011 in PWR_CR1. */
    HAL_PWREx_EnterSTOP3Mode(PWR_STOPENTRY_WFI);

    /* --- MCU wakes here --- */

    HAL_ResumeTick();
    LP_RestoreFromStop2(hlp);   /* clock restoration is identical */

    LP_WakeupSource src = 0U;

    if ((hlp->gdo0_port != NULL) &&
        (__HAL_GPIO_EXTI_GET_IT(hlp->gdo0_pin) != 0U)) {
        src |= LP_WAKEUP_EXTI_GDO0;
        __HAL_GPIO_EXTI_CLEAR_IT(hlp->gdo0_pin);
    }

    if ((hlp->cc1101_gdo0_port != NULL) &&
        (__HAL_GPIO_EXTI_GET_IT(hlp->cc1101_gdo0_pin) != 0U)) {
        src |= LP_WAKEUP_EXTI_CC1101_GDO0;
        __HAL_GPIO_EXTI_CLEAR_IT(hlp->cc1101_gdo0_pin);
    }

    if ((hlp->hrtc != NULL) &&
        (__HAL_RTC_WAKEUPTIMER_GET_FLAG(hlp->hrtc, RTC_FLAG_WUTF) != 0U)) {
        src |= LP_WAKEUP_RTC_ALARM;
        __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hlp->hrtc, RTC_FLAG_WUTF);
    }

    return src;
}

/* -------------------------------------------------------------------------- */
/* Standby (SRAM lost, ~0.3 uA)                                                */
/* -------------------------------------------------------------------------- */

void LP_EnterStandby(LP_HandleTypeDef *hlp)
{
    LP_SPI_Suspend(hlp);

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

    HAL_SuspendTick();

    HAL_PWR_EnterSTANDBYMode();
    /* Execution never returns here; MCU resets on wake-up */
}

/* -------------------------------------------------------------------------- */
/* Clock restoration after Stop 2 / Stop 3                                     */
/* -------------------------------------------------------------------------- */

void LP_RestoreFromStop2(LP_HandleTypeDef *hlp)
{
    /*
     * After any Stop mode exit, SYSCLK is MSI @ 4 MHz.
     * Re-lock MSI explicitly and restore the bus clock tree.
     * VOS4 is retained across Stop, so no voltage scaling change needed.
     */
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
    osc.MSIState            = RCC_MSI_ON;
    osc.MSIClockRange       = RCC_MSIRANGE_4;   /* 4 MHz — sufficient for SPI */
    osc.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    (void)HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2 |
                         RCC_CLOCKTYPE_PCLK3;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    clk.APB3CLKDivider = RCC_HCLK_DIV1;
    (void)HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);

    LP_SPI_Resume(hlp);
}

/* -------------------------------------------------------------------------- */
/* RTC wake-up timer                                                           */
/* -------------------------------------------------------------------------- */

void LP_ConfigureRTCWakeUp(LP_HandleTypeDef *hlp, uint32_t seconds)
{
    if ((hlp == NULL) || (hlp->hrtc == NULL) || (seconds == 0U)) {
        return;
    }

    HAL_RTCEx_DeactivateWakeUpTimer(hlp->hrtc);

    if (seconds <= 18U) {
        /* RTCCLK/16 = 32768/16 = 2048 Hz (LSE) or ~32 kHz/16 (LSI)
         * Counter = seconds * 2048 - 1; max ~18 s at 16-bit */
        uint32_t count = (seconds * 2048U) - 1U;
        if (count > 0xFFFFU) count = 0xFFFFU;
        HAL_RTCEx_SetWakeUpTimer_IT(hlp->hrtc, count,
                                    RTC_WAKEUPCLOCK_RTCCLK_DIV16);
    } else {
        /* 1 Hz CK_SPRE for longer intervals (up to ~18 h) */
        uint32_t count = seconds - 1U;
        if (count > 0xFFFFU) count = 0xFFFFU;
        HAL_RTCEx_SetWakeUpTimer_IT(hlp->hrtc, count,
                                    RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
    }
}

void LP_DisableRTCWakeUp(LP_HandleTypeDef *hlp)
{
    if ((hlp != NULL) && (hlp->hrtc != NULL)) {
        HAL_RTCEx_DeactivateWakeUpTimer(hlp->hrtc);
    }
}

/* -------------------------------------------------------------------------- */
/* SPI suspend / resume                                                        */
/* -------------------------------------------------------------------------- */

void LP_SPI_Suspend(LP_HandleTypeDef *hlp)
{
    if (hlp == NULL) {
        return;
    }
    if (hlp->hspi_cc1101 != NULL) {
        HAL_SPI_DeInit(hlp->hspi_cc1101);
    }
    if (hlp->hspi_cc2500 != NULL) {
        HAL_SPI_DeInit(hlp->hspi_cc2500);
    }
}

void LP_SPI_Resume(LP_HandleTypeDef *hlp)
{
    if (hlp == NULL) {
        return;
    }
    if (hlp->hspi_cc1101 != NULL) {
        HAL_SPI_Init(hlp->hspi_cc1101);
    }
    if (hlp->hspi_cc2500 != NULL) {
        HAL_SPI_Init(hlp->hspi_cc2500);
    }
}

/* -------------------------------------------------------------------------- */
/* Voltage scaling                                                             */
/* -------------------------------------------------------------------------- */

void LP_SetVoltageScaling_LowPower(void)
{
    /* VOS4 (Range 4): lowest core voltage, max 24 MHz, minimum run current.
     * Mandatory before entering Stop 2 / Stop 3 for minimum stop current. */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4);
}

void LP_SetVoltageScaling_Normal(void)
{
    /* VOS2 (Range 2): balanced — max 80 MHz, moderate current */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
}

/* -------------------------------------------------------------------------- */
/* MSI clock switching                                                         */
/* -------------------------------------------------------------------------- */

void LP_SwitchToMSI(uint32_t msi_range)
{
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
    osc.MSIState            = RCC_MSI_ON;
    osc.MSIClockRange       = msi_range;
    osc.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    (void)HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
    (void)HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}
