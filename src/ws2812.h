#include "../libopencm3/libopencm3-examples-git/libopencm3/include/libopencm3/stm32/rcc.h"
#include <../libopencm3/libopencm3-examples-git/libopencm3/include/libopencm3/stm32/gpio.h>
#include <../libopencm3/libopencm3-examples-git/libopencm3/include/libopencm3/stm32/timer.h>
#include <../libopencm3/libopencm3-examples-git/libopencm3/include/libopencm3/cm3/nvic.h>
#include <../libopencm3/libopencm3-examples-git/libopencm3/include/libopencm3/stm32/exti.h>
#include <../libopencm3/libopencm3-examples-git/libopencm3/include/libopencm3/stm32/dma.h>
#include <libopencmsis/core_cm3.h>

void clock_setup(void);
void gpio_setup(void);
void tim_setup(void);
void dma_init(void);
void dma_setup(void);
void dma_start(void);
void dma1_stream6_isr(void);
