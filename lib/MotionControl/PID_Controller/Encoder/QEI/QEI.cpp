#include "QEI.h"
#include <utils.h>

QEI::QEI(int pulsesPerRev, int tsample, TIM_TypeDef* timer) {
  pulsesPerRev_ = pulsesPerRev;
  tsample_ = tsample;
  timer_ = timer;
}

void QEI::init(void) {
  // Configure Timers and Frequency callback
  init_enc_timer(this->timer_);
  this->resetPulses();
}

void QEI::frequency(void) {
  // Motor spin frequency in Hz
  frequency_ = double((this->getPulses() * (1000.0 / tsample_)) / (4.0 * pulsesPerRev_));
  this->resetPulses();
}

double QEI::getFrequency(void) {
  return frequency_;
}

double QEI::getSpeed() {
  return 2.0 * M_PI * (MOTOR_GEAR / WHEEL_GEAR) * this->getFrequency();
}

void QEI::resetPulses(void) {
  // Reset Timer and Pulses counter
  this->timer_->CNT = PULSES_RESET;
  pulses_ = 0;
}

int QEI::getPulses(void) {
  pulses_ = PULSES_RESET - this->timer_->CNT;
  return pulses_;
}

// Choosing Timer to Init.
void QEI::init_enc_timer(TIM_TypeDef* timer) {
  if (timer == TIM1)
    init_enc_timer1();
  else if (timer == TIM2)
    init_enc_timer2();
  else if (timer == TIM8)
    init_enc_timer8();
  else if (timer == TIM3)
    init_enc_timer3();
}

void QEI::init_enc_timer1() // PE_9 PE_11 - Encoder of M1
{
  // Enable registers manipulation for Timer1, GPIOE
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

  // // GPIO Init for E ports 9
  GPIOE->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER9_1;        // Alternate function mode MODERy: 10
  GPIOE->OTYPER |= GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_11;          // Output open-drain OTy: 1
  GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_0 | GPIO_OSPEEDER_OSPEEDR11_0; // Medium Speed OSPEEDRy: 01
  // PUPDR - No pull-up, pull-down PUPDRy: 00
  GPIOE->AFR[1] |= GPIO_AFRH_AFRH1_0 | GPIO_AFRH_AFRH3_0; // Alternate function 1 AFRy: 0001

  TIM1->CR1 = 0;
  TIM1->CR2 = 0;
  TIM1->PSC = 0;
  TIM1->ARR = 0xFFFF;
  TIM1->RCR = 0;
  // IC1 mapped to TI1, IC2 mapped to TI2, filter = 0x0F on both, no prescaler
  TIM1->CCMR1 = TIM_CCMR1_CC1S_0 | (0x0F << 4) | TIM_CCMR1_CC2S_0 | (0x0F << 12);
// set TIM_CCER_CC1P for inverting polarity -> reads positive velocity for motor in clockwise
// direction
#if defined(DIRECT_TRANSMISSION)
  TIM1->CCER =
      TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC1P; // enable input capture, rising polarity
#else
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E; // enable input capture, rising polarity
#endif
  TIM1->DIER = 0;
  TIM1->SMCR = 3; // encoder mode 3, count all flanks
  TIM1->CR1 = TIM_CR1_CEN;
}

void QEI::init_enc_timer2() // PA_5 PB_3 - Encoder of M2
{
  // Enable registers manipulation for Timer2, GPIOA GPIOB
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

  // // GPIO Init for A ports 5
  GPIOA->MODER |= GPIO_MODER_MODER5_1;        // Alternate function mode MODERy: 10
  GPIOA->OTYPER |= GPIO_OTYPER_OT_5;          // Output open-drain OTy: 1
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5_0; // Medium Speed OSPEEDRy: 01
  // PUPDR - No pull-up, pull-down PUPDRy: 00
  GPIOA->AFR[0] |= GPIO_AFRH_AFRH5_0; // Alternate function 1 AFRy: 0001

  // printf("TIMER 2.A:\n - MODER: %X\n - OTYPER: %X\n - OSPEEDR: %X\n - AFR: %X\n", GPIOA->MODER,
  // GPIOA->OTYPER, GPIOA->OSPEEDR, GPIOA->AFR[0]);
  // // GPIO Init for B ports 3
  GPIOB->MODER |= GPIO_MODER_MODER3_1;        // Alternate function mode MODERy: 10
  GPIOB->OTYPER |= GPIO_OTYPER_OT_3;          // Output open-drain OTy: 1
  GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3_0; // Medium Speed OSPEEDRy: 01
  // PUPDR - No pull-up, pull-down PUPDRy: 00
  GPIOB->AFR[0] |= GPIO_AFRH_AFRH3_0; // Alternate function 1 AFRy: 0001
  // printf("TIMER 2.B:\n - MODER: %X\n - OTYPER: %X\n - OSPEEDR: %X\n - AFR: %X\n", GPIOB->MODER,
  // GPIOB->OTYPER, GPIOB->OSPEEDR, GPIOB->AFR[0]);

  TIM2->CR1 = 0;
  TIM2->CR2 = 0;
  TIM2->PSC = 0;
  TIM2->ARR = 0xFFFF;
  TIM2->RCR = 0;
  // IC1 mapped to TI1, IC2 mapped to TI2, filter = 0x0F on both, no prescaler
  TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | (0x0F << 4) | TIM_CCMR1_CC2S_0 | (0x0F << 12);
// set TIM_CCER_CC1P for inverting polarity -> reads positive velocity for motor in clockwise
// direction
#if defined(DIRECT_TRANSMISSION)
  TIM2->CCER =
      TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC1P; // enable input capture, rising polarity
#else
  TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E; // enable input capture, rising polarity
#endif
  TIM2->DIER = 0;
  TIM2->SMCR = 3; // encoder mode 3, count all flanks
  TIM2->CR1 = TIM_CR1_CEN;
}

void QEI::init_enc_timer8() // PC_6 PC_7 - Encoder of M3
{
  // Enable registers manipulation for Timer8 and GPIOC
  RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

  // // GPIO Init for C ports 6 and 7
  GPIOC->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; // Alternate function mode MODERy: 10
  GPIOC->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;      // Output open-drain OTy: 1
  GPIOC->OSPEEDR |=
      GPIO_OSPEEDER_OSPEEDR6_0 | GPIO_OSPEEDER_OSPEEDR7_0; // Medium Speed OSPEEDRy: 01
  // PUPDR - No pull-up, pull-down PUPDRy: 00
  GPIOC->AFR[0] |= GPIO_AFRH_AFRH6_1 | GPIO_AFRH_AFRH6_0 | GPIO_AFRH_AFRH7_1 |
                   GPIO_AFRH_AFRH7_0; // Alternate function 3 AFRy: 0011

  // printf("TIMER 8:\n - MODER: %X\n - OTYPER: %X\n - OSPEEDR: %X\n - AFR: %X\n", GPIOC->MODER,
  // GPIOC->OTYPER, GPIOC->OSPEEDR, GPIOC->AFR[0]);
  TIM8->CR1 = 0;
  TIM8->CR2 = 0;
  TIM8->PSC = 0;
  TIM8->ARR = 0xFFFF;
  TIM8->RCR = 0;
  // IC1 mapped to TI1, IC2 mapped to TI2, filter = 0x0F on both, no prescaler
  TIM8->CCMR1 = TIM_CCMR1_CC1S_0 | (0x0F << 4) | TIM_CCMR1_CC2S_0 | (0x0F << 12);
// set TIM_CCER_CC1P for inverting polarity -> reads positive velocity for motor in clockwise
// direction
#if defined(DIRECT_TRANSMISSION)
  TIM8->CCER =
      TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC1P; // enable input capture, rising polarity
#else
  TIM8->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E; // enable input capture, rising polarity
#endif
  TIM8->DIER = 0;
  TIM8->SMCR = 3; // encoder mode 3, count all flanks
  TIM8->CR1 = TIM_CR1_CEN;
}

void QEI::init_enc_timer3() // PA_6 PB_5 - Encoder of M4
{
  // Enable registers manipulation for Timer3 and GPIOA
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

  // // GPIO Init for A port 6
  GPIOA->MODER |= GPIO_MODER_MODER6_1;        // Alternate function mode MODERy: 10
  GPIOA->OTYPER |= GPIO_OTYPER_OT_6;          // Output open-drain OTy: 1
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_0; // Medium Speed OSPEEDRy: 01
  // PUPDR - No pull-up, pull-down PUPDRy: 00
  GPIOA->AFR[0] |= GPIO_AFRH_AFRH6_1; // Alternate function 2 AFRy: 0010

  // printf("TIMER 3.A:\n - MODER: %X\n - OTYPER: %X\n - OSPEEDR: %X\n - AFR: %X\n", GPIOA->MODER,
  // GPIOA->OTYPER, GPIOA->OSPEEDR, GPIOA->AFR[0]);
  // // GPIO Init for B port 5
  GPIOB->MODER |= GPIO_MODER_MODER5_1;        // Alternate function mode MODERy: 10
  GPIOB->OTYPER |= GPIO_OTYPER_OT_5;          // Output open-drain OTy: 1
  GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5_0; // Medium Speed OSPEEDRy: 01
  // PUPDR - No pull-up, pull-down PUPDRy: 00
  GPIOB->AFR[0] |= GPIO_AFRH_AFRH5_1; // Alternate function 2 AFRy: 0010

  // printf("TIMER 3.B:\n - MODER: %X\n - OTYPER: %X\n - OSPEEDR: %X\n - AFR: %X\n", GPIOB->MODER,
  // GPIOB->OTYPER, GPIOB->OSPEEDR, GPIOB->AFR[0]);
  TIM3->CR1 = 0;
  TIM3->CR2 = 0;
  TIM3->PSC = 0;
  TIM3->ARR = 0xFFFF;
  TIM3->RCR = 0;
  // IC1 mapped to TI1, IC2 mapped to TI2, filter = 0x0F on both, no prescaler
  TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | (0x0F << 4) | TIM_CCMR1_CC2S_0 | (0x0F << 12);
// set TIM_CCER_CC1P for inverting polarity -> reads positive velocity for motor in clockwise
// direction
#if defined(DIRECT_TRANSMISSION)
  TIM3->CCER =
      TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC1P; // enable input capture, rising polarity
#else
  TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E; // enable input capture, rising polarity
#endif
  TIM3->DIER = 0;
  TIM3->SMCR = 3; // encoder mode 3, count all flanks
  TIM3->CR1 = TIM_CR1_CEN;
}
