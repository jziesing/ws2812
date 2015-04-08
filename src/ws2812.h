#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/dma.h>
#include <libopencmsis/core_cm3.h>

void ws2812_setup(uint32_t gpio_port, uint8_t af_num, uint8_t gpio_pin, 
                    uint32_t timer, enum tim_oc_id oc_id, uint32_t tim_cc_reg, enum rcc_periph_clken tim_clken, 
                    uint8_t nvic_timer_irq, uint32_t dma_controller, uint8_t dma_stream, uint8_t nvic_dma_irq, 
                    enum rcc_periph_clken dma_clken);
void tim_setup(uint32_t timer, enum tim_oc_id oc_id, enum rcc_periph_clken clken, uint8_t nvic_timer_irq);
void gpio_setup(uint32_t gpio_port, uint8_t gpio_pin, uint8_t af_num);
void dma_setup(enum rcc_periph_clken dma_clken, uint32_t dma_controller, 
                uint8_t dma_stream, uint8_t nvic_dma_irq, uint32_t tim_cc_reg, uint16_t *data_block);
void show(void);
void clock_setup(void);
void setPixelColor(uint8_t led_index, uint16_t* data_block, uint8_t red, uint8_t green, uint8_t blue);
