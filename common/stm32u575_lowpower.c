#include "stm32u575_lowpower.h"

/* ==========================================================================
 * STM32U575 Ultra-Low Power Implementation
 *
 * Target: STM32U575 Q-series (UFBGA132)
 * Key features used:
 *   - LPMS (Low Power Mode Selection) for Stop 2
 *   - PWR voltage scaling Range 4 (lowest) in LP run
 *   - MSI RC oscillator (no external crystal needed in sleep)
 *   - EXTI line for CC2500 GDO0 wake-up
 *   - RTC with LSI for periodic housekeeping wake-up
 * ========================================================================== */

/* -------------------------------------------------------------------------- */
/* initialization                                                              */
/* -------------------------------------------------------------------------- */

void LP_Init(LP_HandleTypeDef *hlp)
{
    if (hlp == NULL) {
        return;
    }

    /* Enable instruction cache: reduces flash-read stalls at 4 MHz MSI,
     * cutting active current during radio driver execution. */
    __HAL_RCC_ICACHE_CLK_ENABLE();
    ICACHE->CR |= ICACHE_CR_EN_Msk;

    /* Enable PWR clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Enable backup domain access for RTC */
    HAL_PWR_EnableBkUpAccess();

    /* Configure CC2500 GDO0 as EXTI wake-up source (rising edge) */
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

    /* Configure CC1101 GDO0 as EXTI wake-up source (rising edge).
     * Used to wake MCU from Stop 2 during MICS session RX without
     * busy-polling at 4 MHz — GDO0 asserts only on CRC-OK packet. */
    if (hlp->cc1101_gdo0_port != NULL) {
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin   = hlp->cc1101_gdo0_pin;
        gpio.Mode  = GPIO_MODE_IT_RISING;
        gpio.Pull  = GPIO_PULLDOWN;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(hlp->cc1101_gdo0_port, &gpio);

        HAL_NVIC_SetPriority(hlp->cc1101_gdo0_irqn, 0, 0);
        HAL_NVIC_EnableIRQ(hlp->cc1101_gdo0_irqn);
    }

    /* Enable ultra-low power mode: VREFINT off in Stop 2 */
    HAL_PWREx_EnableUltraLowPowerMode();

    /* Disable fast wake-up from Stop for lower consumption
     * (adds ~3.5 us to wake-up time, but saves power in stop) */
    HAL_PWREx_DisableFastWakeUpFromStop();
}

/* -------------------------------------------------------------------------- */
/* Stop 2 entry/exit                                                           */
/* -------------------------------------------------------------------------- */

/* Internal helper: common Stop entry/exit logic shared by Stop 2 and Stop 3.
 * Caller passes the HAL enter function to use. */
static LP_WakeupSource lp_enter_stop_common(LP_HandleTypeDef *hlp,
                                             void (*enter_fn)(uint8_t))
{
    LP_SPI_Suspend(hlp);

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    /* Clear all configured EXTI pending bits before sleeping so a stale
     * assertion from before the WFI doesn't immediately re-wake the core. */
    __HAL_GPIO_EXTI_CLEAR_IT(hlp->gdo0_pin);
    if (hlp->cc1101_gdo0_port != NULL) {
        __HAL_GPIO_EXTI_CLEAR_IT(hlp->cc1101_gdo0_pin);
    }

    HAL_SuspendTick();

    enter_fn(PWR_STOPENTRY_WFI);

    /* --- MCU wakes up here; ISRs fire before next C statement --- */

    HAL_ResumeTick();
    LP_RestoreFromStop2(hlp);

    /* Wake-source detection: ISRs (CC2500/CC1101 EXTI callbacks, RTC handler)
     * run between WFI and here and will have cleared the EXTI/RTC pending bits.
     * We therefore rely on the volatile flags set inside those ISR callbacks
     * (g_wakeup_flag for CC2500, g_cc1101_gdo0_flag for CC1101) at the
     * application layer.  The LP source bits are set here only for the RTC
     * whose WUTF flag is cleared by HAL_RTCEx_WakeUpTimerIRQHandler; if that
     * handler is not registered the flag is still set and we pick it up here.
     * Either way the application can also check its own g_*_flag variables. */
    LP_WakeupSource src = 0U;

    if (__HAL_GPIO_EXTI_GET_IT(hlp->gdo0_pin) != 0U) {
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

    /* PWR WKUP lines (WUF1..WUF8) — Stop 3 / Standby physical wake */
    {
        static const uint32_t s_wkup_pwr_flags[8] = {
            PWR_WAKEUP_FLAG1, PWR_WAKEUP_FLAG2, PWR_WAKEUP_FLAG3, PWR_WAKEUP_FLAG4,
            PWR_WAKEUP_FLAG5, PWR_WAKEUP_FLAG6, PWR_WAKEUP_FLAG7, PWR_WAKEUP_FLAG8
        };
        bool wkup = false;
        for (uint32_t i = 0U; i < 8U; i++) {
            if (__HAL_PWR_GET_FLAG(s_wkup_pwr_flags[i])) {
                wkup = true;
                __HAL_PWR_CLEAR_FLAG(s_wkup_pwr_flags[i]);
            }
        }
        if (wkup) {
            src |= LP_WAKEUP_WKUP_PIN;
        }
    }

    return src;
}

LP_WakeupSource LP_EnterStop2(LP_HandleTypeDef *hlp)
{
    if (hlp == NULL) {
        return 0U;
    }
    return lp_enter_stop_common(hlp, HAL_PWREx_EnterSTOP2Mode);
}

LP_WakeupSource LP_EnterStop3(LP_HandleTypeDef *hlp)
{
    if (hlp == NULL) {
        return 0U;
    }
    return lp_enter_stop_common(hlp, HAL_PWREx_EnterSTOP3Mode);
}

void LP_RestoreFromStop2(LP_HandleTypeDef *hlp)
{
    /*
     * After Stop 2 wake-up, the system clock is MSI at 4 MHz.
     * Reconfigure clocks as needed for the application.
     */

    /* Re-enable HSI if it was used */
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    osc.MSIState       = RCC_MSI_ON;
    osc.MSIClockRange  = RCC_MSIRANGE_4;  /* 4 MHz - sufficient for radio comms */
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

    /* Resume SPI peripherals */
    LP_SPI_Resume(hlp);
}

/* -------------------------------------------------------------------------- */
/* Standby (ultra-deep sleep, SRAM lost)                                       */
/* -------------------------------------------------------------------------- */

void LP_EnterStandby(LP_HandleTypeDef *hlp)
{
    LP_SPI_Suspend(hlp);

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

    HAL_SuspendTick();

    HAL_PWR_EnterSTANDBYMode();

    /* Never reaches here - system resets on Standby wake-up */
}

/* -------------------------------------------------------------------------- */
/* RTC wake-up timer                                                           */
/* -------------------------------------------------------------------------- */

void LP_ConfigureRTCWakeUp(LP_HandleTypeDef *hlp, uint32_t seconds)
{
    if ((hlp == NULL) || (hlp->hrtc == NULL) || (seconds == 0U)) {
        return;
    }

    /* Disable first, then reconfigure */
    HAL_RTCEx_DeactivateWakeUpTimer(hlp->hrtc);

    /*
     * RTC wake-up clock = RTCCLK/16 = 32768/16 = 2048 Hz (for LSE)
     * or LSI ~32 kHz / 16 = ~2000 Hz
     * For seconds: counter = seconds * 2048 - 1
     * Max ~32 seconds with 16-bit counter at /16
     * For longer intervals, use 1 Hz clock source
     */
    if (seconds <= 18U) {
        /* Use RTCCLK/16 for precision up to ~18 seconds */
        uint32_t count = (seconds * 2048U) - 1U;
        if (count > 0xFFFFU) count = 0xFFFFU;
        HAL_RTCEx_SetWakeUpTimer_IT(hlp->hrtc, count, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
    } else {
        /* Use 1 Hz CK_SPRE clock for longer intervals (up to 18 hours) */
        uint32_t count = seconds - 1U;
        if (count > 0xFFFFU) count = 0xFFFFU;
        HAL_RTCEx_SetWakeUpTimer_IT(hlp->hrtc, count, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
    }
}

void LP_ConfigureRTCWakeUp_ms(LP_HandleTypeDef *hlp, uint32_t ms)
{
    if ((hlp == NULL) || (hlp->hrtc == NULL) || (ms == 0U)) {
        return;
    }

    HAL_RTCEx_DeactivateWakeUpTimer(hlp->hrtc);

    if (ms <= 32000U) {
        /*
         * RTCCLK/16 clock source: 32768 Hz / 16 = 2048 Hz → ~488 µs/tick.
         * count = ms × 2048 / 1000.  WakeUpTimer fires after (count) ticks,
         * so actual delay = count / 2048 s ≈ ms × 1 ms.
         * Example: ms=20 → count=41 → 41/2048 ≈ 20.0 ms.
         */
        uint32_t count = (ms * 2048U) / 1000U;
        if (count == 0U) { count = 1U; }
        if (count > 0xFFFFU) { count = 0xFFFFU; }
        HAL_RTCEx_SetWakeUpTimer_IT(hlp->hrtc, count - 1U,
                                    RTC_WAKEUPCLOCK_RTCCLK_DIV16);
    } else {
        /* Beyond 32 s fall back to 1 Hz CK_SPRE (same as LP_ConfigureRTCWakeUp) */
        uint32_t count = (ms / 1000U);
        if (count == 0U) { count = 1U; }
        if (count > 0xFFFFU) { count = 0xFFFFU; }
        HAL_RTCEx_SetWakeUpTimer_IT(hlp->hrtc, count - 1U,
                                    RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
    }
}

void LP_DisableRTCWakeUp(LP_HandleTypeDef *hlp)
{
    if ((hlp != NULL) && (hlp->hrtc != NULL)) {
        HAL_RTCEx_DeactivateWakeUpTimer(hlp->hrtc);
    }
}

void LP_EnableWakeUpPin(uint32_t wake_up_pin)
{
    HAL_PWR_EnableWakeUpPin(wake_up_pin);
}

void LP_DisableWakeUpPin(uint32_t wake_up_pin)
{
    HAL_PWR_DisableWakeUpPin(wake_up_pin);
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
        /* Disable SPI clock to reduce leakage */
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
/* voltage scaling                                                             */
/* -------------------------------------------------------------------------- */

void LP_SetVoltageScaling_LowPower(void)
{
    /* Range 4: lowest voltage, max 24 MHz, minimum consumption */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4);
}

void LP_SetVoltageScaling_Normal(void)
{
    /* Range 2: balanced, max 80 MHz */
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