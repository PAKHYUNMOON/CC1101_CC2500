#ifndef STM32U575ZI_UART_H
#define STM32U575ZI_UART_H

#include "stm32u5xx_hal.h"

HAL_StatusTypeDef STM32U575ZI_UART_SendBytes(UART_HandleTypeDef *huart,
                                             const uint8_t *data,
                                             uint16_t len);
HAL_StatusTypeDef STM32U575ZI_UART_SendString(UART_HandleTypeDef *huart,
                                              const char *str);
HAL_StatusTypeDef STM32U575ZI_UART_Printf(UART_HandleTypeDef *huart,
                                          const char *fmt,
                                          ...);
HAL_StatusTypeDef STM32U575ZI_UART_SendLine(UART_HandleTypeDef *huart,
                                            const char *str);

#endif /* STM32U575ZI_UART_H */
