/* Linux host test: actual FOC/HAL stop functions, RAM-backed TIM1 registers.
 * gcc -std=c99 -O2 -ffunction-sections -fdata-sections -Wall -Werror \
 *   -Wno-pointer-to-int-cast -Wno-int-to-pointer-cast -Wno-overflow \
 *   -DUSE_HAL_DRIVER -DSTM32G474xx -IUSER -ICore/Inc \
 *   -IDrivers/STM32G4xx_HAL_Driver/Inc \
 *   -IDrivers/CMSIS/Device/ST/STM32G4xx/Include -IDrivers/CMSIS/Include \
 *   tests/test_foc_pwm_stop.c USER/foc.c \
 *   Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c \
 *   Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c \
 *   -Wl,--gc-sections -lm -o /tmp/test_foc_pwm_stop
 * /tmp/test_foc_pwm_stop
 */
#define _GNU_SOURCE
#include <assert.h>
#include <sys/mman.h>
#include <unistd.h>
#include "tim.h"
#include "foc.h"

TIM_HandleTypeDef htim1;

int main(void)
{
    size_t page_size = (size_t)sysconf(_SC_PAGESIZE);
    uintptr_t base = (uintptr_t)TIM1 & ~(page_size - 1);
    void *mapped = mmap((void *)base, page_size, PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    uint32_t bridge_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                           TIM_CCER_CC2E | TIM_CCER_CC2NE |
                           TIM_CCER_CC3E | TIM_CCER_CC3NE;
    unsigned int i;

    assert(mapped != MAP_FAILED && mapped == (void *)base);
    assert((uintptr_t)TIM1 - base + sizeof(*TIM1) <= page_size);
    htim1.Instance = TIM1;
    TIM1->ARR = 4249;
    TIM1->CCR4 = 4100;
    TIM_CHANNEL_STATE_SET(&htim1, TIM_CHANNEL_4, HAL_TIM_CHANNEL_STATE_READY);
    assert(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) == HAL_OK);

    for (i = 0; i < 3; ++i)
    {
        /* Cover initial IDLE, active bridge shutdown, and repeated shutdown. */
        if (i == 1)
            TIM1->CCER |= bridge_mask;
        foc_pwm_stop();
        assert((TIM1->CCER & bridge_mask) == 0);
        assert((TIM1->CCER & TIM_CCER_CC4E) != 0);
        assert((TIM1->CR1 & TIM_CR1_CEN) != 0);
        assert((TIM1->BDTR & TIM_BDTR_MOE) != 0);
        assert(TIM1->CCR4 == 4100 && TIM1->ARR == 4249);
        assert(TIM_CHANNEL_STATE_GET(&htim1, TIM_CHANNEL_4) == HAL_TIM_CHANNEL_STATE_BUSY);
    }

    /* Stopping must never re-enable MOE after a hardware break. */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER |= bridge_mask;
    foc_pwm_stop();
    assert((TIM1->BDTR & TIM_BDTR_MOE) == 0);
    assert((TIM1->CCER & bridge_mask) == 0);
    assert(munmap(mapped, page_size) == 0);
    return 0;
}
