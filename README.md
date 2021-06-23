### FreeRTOS, ADC, PWM, DMA Example ###

This project can be extended to test SPI and I2C communications with FreeRTOS on the STM32F411RE Nucleo Board

Currently, this project only tests using ADC and PWM with DMA, and evaluates whether updating TIMx->CCRy through 
DMA is more advantageous in contrast to modifying the TIMx->CCRy registers manually