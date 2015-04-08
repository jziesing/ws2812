/*
* This file is part of the libopencm3 project.
*
* Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
* Copyright (C) 2015 Jack Ziesing <jziesing@gmail.com>
*
* This library is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this library. If not, see <http://www.gnu.org/licenses/>.
*/
#include "ws2812.h"

uint16_t data_block_size;
uint32_t curr_dma_controller; 
uint8_t curr_dma_stream;
uint8_t curr_nvic_dma_irq;
enum rcc_periph_clken curr_tim_clken;
uint32_t curr_tim_cc_reg;

enum rcc_periph_clken curr_dma_clken;

void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
}

void gpio_setup(uint32_t gpio_port, uint8_t gpio_pin, uint8_t af_num)
{
    rcc_periph_clock_enable(gpio_port);
    gpio_mode_setup(gpio_port, GPIO_MODE_AF, GPIO_PUPD_NONE, gpio_pin);
    gpio_set_af(gpio_port, af_num, gpio_pin);
}

void tim_setup(uint32_t timer, enum tim_oc_id oc_id, 
        enum rcc_periph_clken tim_clken, uint8_t nvic_timer_irq)
{
    rcc_periph_clock_enable(tim_clken);
    nvic_enable_irq(nvic_timer_irq);
    timer_reset(timer);
    timer_set_mode(timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(timer, 0);
    timer_continuous_mode(timer);
    timer_set_period(timer, 104);
    timer_disable_oc_output(timer, oc_id);
    timer_disable_oc_clear(timer, oc_id);
    timer_enable_oc_preload(timer, oc_id);
    timer_set_oc_slow_mode(timer, oc_id);
    timer_set_oc_mode(timer, oc_id, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(timer, oc_id);
    timer_set_oc_value(timer, oc_id, 0);
    timer_enable_oc_output(timer, oc_id);
    timer_enable_preload(timer);
    timer_enable_counter(timer);
    timer_enable_irq(timer, TIM_DIER_UDE);
}

void dma_setup(enum rcc_periph_clken dma_clken, uint32_t dma_controller, 
                uint8_t dma_stream, uint8_t nvic_dma_irq, uint32_t tim_cc_reg, uint16_t *data_block)
{
    rcc_periph_clock_enable(dma_clken);
    nvic_enable_irq(nvic_dma_irq);
    dma_stream_reset(dma_controller, dma_stream);
    dma_set_priority(dma_controller, dma_stream, DMA_SxCR_PL_VERY_HIGH);
    dma_set_memory_size(dma_controller, dma_stream, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(dma_controller, dma_stream, DMA_SxCR_PSIZE_16BIT);
    dma_enable_circular_mode(dma_controller, dma_stream);
    dma_enable_memory_increment_mode(dma_controller, dma_stream);
    dma_set_transfer_mode(dma_controller, dma_stream, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    dma_set_peripheral_address(dma_controller, dma_stream, (uint32_t)&tim_cc_reg);
    dma_set_memory_address(dma_controller, dma_stream, (uint32_t)&data_block);
    // 28 LED's = (24 * 28) + 40 
    // 40 = 50us/1.25us
    dma_set_number_of_data(dma_controller, dma_stream, data_block_size);
    dma_enable_half_transfer_interrupt(dma_controller, dma_stream);
    dma_enable_transfer_complete_interrupt(dma_controller, dma_stream);
    dma_channel_select(dma_controller, dma_stream, DMA_SxCR_CHSEL_2);
    nvic_clear_pending_irq(nvic_dma_irq);
    nvic_enable_irq(nvic_dma_irq);
    nvic_set_priority(nvic_dma_irq, 0);
}

void show(void)
{
    dma_enable_stream(curr_dma_controller, curr_dma_stream);
}

void dma1_stream6_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_HTIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_HTIF);
        
    }
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_TCIF);
        
    }
}


// index from 0, color vals 0-255
void setPixelColor(uint8_t led_index, uint16_t *data_block, 
                    uint8_t red, uint8_t green, uint8_t blue)
{
    uint16_t color_block[24];
    if (green/128 >= 1) {
        color_block[0] = 60;
        green -= 128;
    } else {
        color_block[0] = 29;
    }
    if (green/64 >= 1) {
        color_block[1] = 60;
        green -= 64;
    } else {
        color_block[1] = 29;
    }
    if (green/32 >= 1) {
        color_block[2] = 60;
        green -= 32;
    } else {
        color_block[2] = 29;
    }
    if (green/16 >= 1) {
        color_block[3] = 60;
        green -= 16;
    } else {
        color_block[3] = 29;
    }
    if (green/8 >= 1) {
        color_block[4] = 60;
        green -= 8;
    } else {
        color_block[4] = 29;
    }
    if (green/4 >= 1) {
        color_block[5] = 60;
        green -= 4;
    } else {
        color_block[5] = 29;
    }
    if (green/2 >= 1) {
        color_block[6] = 60;
        green -= 2;
    } else {
        color_block[6] = 29;
    }
    if (green/1 >= 1) {
        color_block[7] = 60;
        green -= 1;
    } else {
        color_block[7] = 29;
    }
    if (red/128 >= 1) {
        color_block[8] = 60;
        red -= 128;
    } else {
        color_block[8] = 29;
    }
    if (red/64 >= 1) {
        color_block[9] = 60;
        red -= 64;
    } else {
        color_block[9] = 29;
    }
    if (red/32 >= 1) {
        color_block[10] = 60;
        red -= 32;
    } else {
        color_block[10] = 29;
    }
    if (red/16 >= 1) {
        color_block[11] = 60;
        red -= 16;
    } else {
        color_block[11] = 29;
    }
    if (red/8 >= 1) {
        color_block[12] = 60;
        red -= 8;
    } else {
        color_block[12] = 29;
    }
    if (red/4 >= 1) {
        color_block[13] = 60;
        red -= 4;
    } else {
        color_block[13] = 29;
    }
    if (red/2 >= 1) {
        color_block[14] = 60;
        red -= 2;
    } else {
        color_block[14] = 29;
    }
    if (red/1 >= 1) {
        color_block[15] = 60;
        red -= 1;
    } else {
        color_block[15] = 29;
    }
    if (blue/128 >= 1) {
        color_block[16] = 60;
        blue -= 128;
    } else {
        color_block[16] = 29;
    }
    if (blue/64 >= 1) {
        color_block[17] = 60;
        blue -= 64;
    } else {
        color_block[17] = 29;
    }
    if (blue/32 >= 1) {
        color_block[18] = 60;
        blue -= 32;
    } else {
        color_block[18] = 29;
    }
    if (blue/16 >= 1) {
        color_block[19] = 60;
        blue -= 16;
    } else {
        color_block[19] = 29;
    }
    if (blue/8 >= 1) {
        color_block[20] = 60;
        blue -= 8;
    } else {
        color_block[20] = 29;
    }
    if (blue/4 >= 1) {
        color_block[21] = 60;
        blue -= 4;
    } else {
        color_block[21] = 29;
    }
    if (blue/2 >= 1) {
        color_block[22] = 60;
        blue -= 2;
    } else {
        color_block[22] = 29;
    }
    if (blue/1 >= 1) {
        color_block[23] = 60;
        blue -= 1;
    } else {
        color_block[23] = 29;
    }
    led_index = led_index*24;
    int end_index = led_index + 24;
    int i;
    int j = 0;
    for (i = 0; i < 88; ++i) {
        if (i >= led_index && i < end_index) {
            data_block[i] = color_block[j];
            ++j;
        }  
    }
    dma_setup(curr_dma_clken, curr_dma_controller, curr_dma_stream, curr_nvic_dma_irq, curr_tim_cc_reg, data_block);
}
/*
 * Initial method to call to setup ws2812 mod
 * See board specs for configuration.
 * All params can be satisfied with libopencm3 conviences vars
 * Params:
 *   - uint16_t num_leds = number of led you are controlling
 *   - uint32_t gpio_port = the gpio port (f4 = a-f)
 *   - uint8_t af_num = the altnernate function you choose
 *   - uint8_t gpio_pin = the specific pin on the board you use
 *   - uint32_t timer = the timer you use, need general pupose timer on f4 tim2-tim5 are general purpose
 *   - enum tim_oc_id oc_id = the timer output compare register you choose
 *   - uint32_t tim_cc_reg = the timer capture compare register you choose
 *   - enum rcc_periph_clken tim_clken = APB1 peripheral setup for timer
 *   - uint8_t nvic_timer_irq = global interrup for timer
 *   - uint32_t dma_controller = recomend use of dma1
 *   - uint8_t dma_stream = the stream on the channel and controller you use
 *   - uint8_t nvic_dma_irq = global interrupt for dma controller
 *   - enum rcc_periph_clken dma_clken = APB1 peripheral setup for dma
 */
void ws2812_setup(uint32_t gpio_port, uint8_t af_num, uint8_t gpio_pin, 
                    uint32_t timer, enum tim_oc_id oc_id, uint32_t tim_cc_reg, 
                    enum rcc_periph_clken tim_clken, uint8_t nvic_timer_irq, 
                    uint32_t dma_controller, uint8_t dma_stream, uint8_t nvic_dma_irq, 
                    enum rcc_periph_clken dma_clken) 
{
    clock_setup();
    gpio_setup(gpio_port, gpio_pin, af_num);
    tim_setup(timer, oc_id, tim_clken, nvic_timer_irq);
    curr_dma_controller = dma_controller;
    curr_dma_stream = dma_stream;
    curr_nvic_dma_irq = nvic_dma_irq; 
    curr_dma_clken = dma_clken;
    curr_tim_clken = tim_clken;
    curr_tim_cc_reg = tim_cc_reg;
    
    

}

