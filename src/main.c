/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Damjan Marion <damjan.marion@gmail.com>
 * Copyright (C) 2011 Mark Panajotovic <marko@electrontube.org>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "ws2812.h"


int main(void)
{
    uint16_t num_of_leds = 16;
    uint16_t main_data_block[(num_of_leds*24)+40];
    ws2812_setup(GPIOD, GPIO_AF2, GPIO12, 
                    TIM4, TIM_OC1, TIM4_CCR1, RCC_TIM4, NVIC_TIM4_IRQ, DMA1,
                    DMA_STREAM6, NVIC_DMA1_STREAM6_IRQ, RCC_DMA1);

    setPixelColor(5, main_data_block, 255, 15, 15);
    show();

    while (1) {
        __WFI();
    }
    return 0;
}