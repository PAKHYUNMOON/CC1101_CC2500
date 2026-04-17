/*
 * STM32U575ZI-Q USB-UART helper
 *
 * Usage:
 *   - Call with the UART handle already initialized by CubeMX/main.c
 *   - Typical USB-UART on NUCLEO board is USART1 or USART3 depending on setup
 */

#include "stm32u575zi_UART.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#ifndef STM32U575ZI_UART_TX_TIMEOUT_MS
#define STM32U575ZI_UART_TX_TIMEOUT_MS  100U
#endif

HAL_StatusTypeDef STM32U575ZI_UART_SendBytes(UART_HandleTypeDef *huart,
                                             const uint8_t *data,
                                             uint16_t len)
{
    if ((huart == NULL) || (data == NULL) || (len == 0U)) {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(huart, (uint8_t *)data, len, STM32U575ZI_UART_TX_TIMEOUT_MS);
}

HAL_StatusTypeDef STM32U575ZI_UART_SendString(UART_HandleTypeDef *huart,
                                              const char *str)
{
    size_t len = 0U;

    if ((huart == NULL) || (str == NULL)) {
        return HAL_ERROR;
    }

    len = strlen(str);
    if (len == 0U) {
        return HAL_OK;
    }

    if (len > 0xFFFFU) {
        len = 0xFFFFU;
    }

    return HAL_UART_Transmit(huart,
                             (uint8_t *)str,
                             (uint16_t)len,
                             STM32U575ZI_UART_TX_TIMEOUT_MS);
}

HAL_StatusTypeDef STM32U575ZI_UART_Printf(UART_HandleTypeDef *huart,
                                          const char *fmt,
                                          ...)
{
    char tx_buf[256];
    int n = 0;
    va_list ap;

    if ((huart == NULL) || (fmt == NULL)) {
        return HAL_ERROR;
    }

    va_start(ap, fmt);
    n = vsnprintf(tx_buf, sizeof(tx_buf), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return HAL_ERROR;
    }

    if ((size_t)n >= sizeof(tx_buf)) {
        n = (int)(sizeof(tx_buf) - 1U);
    }

    return HAL_UART_Transmit(huart,
                             (uint8_t *)tx_buf,
                             (uint16_t)n,
                             STM32U575ZI_UART_TX_TIMEOUT_MS);
}

HAL_StatusTypeDef STM32U575ZI_UART_SendLine(UART_HandleTypeDef *huart,
                                            const char *str)
{
    HAL_StatusTypeDef st;

    st = STM32U575ZI_UART_SendString(huart, str);
    if (st != HAL_OK) {
        return st;
    }

    return STM32U575ZI_UART_SendString(huart, "\r\n");
}
