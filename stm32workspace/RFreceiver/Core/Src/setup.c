/*
 * setup.c
 *
 *  Created on: Aug 3, 2026
 *      Author: Nymph
 */

#include "main.h"

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi){
	GPIO_InitTypeDef GPIO_InitStruct1  = {0};
	GPIO_InitTypeDef GPIO_InitStruct2  = {0};

	if(hspi->Instance==SPI1){
		__HAL_RCC_SPI1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();

		GPIO_InitStruct1.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7; //sck, miso, mosi
		GPIO_InitStruct1.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct1.Pull = GPIO_NOPULL;
		GPIO_InitStruct1.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct1.Alternate = GPIO_AF5_SPI1;

		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct1);

		__HAL_RCC_GPIOB_CLK_ENABLE();

		GPIO_InitStruct2.Pin = GPIO_PIN_6 | GPIO_PIN_8; // CSn and GDO2
		GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct2.Pull = GPIO_NOPULL;
		GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct2);
	}
}
