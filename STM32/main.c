#include "stm32f10x.h"

unsigned long TIM3_interrupts = 0;

void SystemCoreClockConfigure(void) {
    RCC->CR |= ((uint32_t)RCC_CR_HSION);                     // Включаем HSI
    while ((RCC->CR & RCC_CR_HSIRDY) == 0);                  // Ждем, пока HSI не будет готово

    RCC->CFGR = RCC_CFGR_SW_HSI;                             // HSI как системный тактировщик
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // Ждем, пока HSI не станет источником тактов

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // HCLK = SYSCLK
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;                        // APB1 = HCLK/4
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;                        // APB2 = HCLK

    RCC->CR &= ~RCC_CR_PLLON;                                // Отключаем PLL

    // Настройка PLL: HSI/2 * 12 = 48 MHz
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    RCC->CFGR |=  (RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLMULL12);

    RCC->CR |= RCC_CR_PLLON;                                 // Включаем PLL
    while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP();           // Ждем, пока PLL не станет готово

    RCC->CFGR &= ~RCC_CFGR_SW;                               // Выбираем PLL как системный источник тактов
    RCC->CFGR |=  RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Ждем, пока PLL не станет источником тактов
}

void TIM3_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                      // Включаем тактирование TIM3
    TIM3->CR1 = TIM_CR1_CEN;                                 // Разрешаем таймер TIM3

    TIM3->PSC = (SystemCoreClock / 2) / 1000 - 1;            // 47999
    TIM3->ARR = 100 - 1;

    TIM3->DIER |= TIM_DIER_UIE;                              // Разрешаем прерывание от TIM3
    NVIC_EnableIRQ(TIM3_IRQn);                                // Разрешаем прерывание в контроллере прерываний
}

void GPIO_Init(void) {
    RCC->APB2ENR |= (1UL << 2);                               // Включаем тактирование порта A (GPIOA)

    // Настроим PA5 как выход (аналогично встроенному светодиоду)
    GPIOA->CRL &= ~((15ul << 4*5));  // Очищаем настройки для PA5
    GPIOA->CRL |=  ((1ul << 4*5));   // Настроим PA5 как цифровой выход с максимальной скоростью
}

void LED_BLINKING(void) {
    GPIOA->BSRR = (GPIOA->ODR & (1ul << 5)) ? (1ul << 21) : (1ul << 5); // Тогглинг для PA5
}

void LED_ON(void) {
    GPIOA->BSRR = 1ul << 5;  // Включить светодиод на PA5
}

void LED_OFF(void) {
    GPIOA->BSRR = 1ul << 21; // Выключить светодиод на PA5
}

void TIM3_IRQHandler(void) {  // Обработчик прерывания от TIM3
    TIM3->SR &= ~TIM_SR_UIF;     // Очищаем флаг прерывания
    TIM3_interrupts++;           // Увеличиваем счетчик прерываний
    LED_BLINKING();              // Мигаем светодиодом
}

int main(void) {
    SystemCoreClockConfigure();                             
    SystemCoreClockUpdate();
    
    TIM3_Init();    
    GPIO_Init();
    
    while (1) {
        // Основной цикл можно оставить пустым, поскольку мигание осуществляется в прерывании
    }
}
