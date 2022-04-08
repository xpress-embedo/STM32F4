# FreeRTOS Notes

### Time Base Source Selection
* FreeRTOS uses ARM Cortex Mx Processor's internal SysTick timer as its timer base (RTOS ticking).  
* STM32 Cube HAL Layer by default also uses the SysTick timer as its base time source.  
* If we are using both FreeRTOS and STM32Cube HAL Layer in our project, there will be a conflict to use a time source.  
* To resolve this conflict, it is strongly recommended to use STM32 Cube HAL Layer time-base source to some other timer peripheral of the micro-controller.  