# FreeRTOS Notes

## Time Base Source Selection
* FreeRTOS uses ARM Cortex Mx Processor's internal SysTick timer as its timer base (RTOS ticking).  
* STM32 Cube HAL Layer by default also uses the SysTick timer as its base time source.  
* If we are using both FreeRTOS and STM32Cube HAL Layer in our project, there will be a conflict to use a time source.  
* To resolve this conflict, it is strongly recommended to use STM32 Cube HAL Layer time-base source to some other timer peripheral of the micro-controller.  

## Scheduling
* When the tasks are created using `xTaskCreate` function, they are automatically added into the ready task list of the FreeRTOS (ready state).  
* Tasks will be dispatched to run on the CPU by the scheduler.  
* Scheduler is nothing but a piece of code that is part of the FreeRTOS kernel, which runs in the privileged mode of the processor.  
* Firstly the scheduler should be invoked by calling the API provided by FreeRTOS i.e. `vTaskStartScheduler()`  

### Scheduling Policy
Scheduler schedules tasks to run on the CPU according to the scheduling policy configured.  
1) **Preemptive Scheduling** => when `configUSE_PREEMPTION = 1 `  
   Preemption is replacing a running task with another task. During preemption, the running task is made to give up the processor even if it hasn't finished the work. The scheduler does this to run some other tasks of the application.  
   The tasks which gave up the processor simply returns to the ready state.  
   There are further two types of preemptive scheduling as mentioned below.  
   * Round-Robin (Cyclic) Preemptive Scheduling : Fixed time slice is used for each task  
   * Priority Base Preemptive Scheduling  
     In this type of scheduling the tasks are scheduled to run of the CPU based on their priority. As task with higher priority will be made to run on the CPU forever unless the task gets deleted/blocked/suspended of leaves voluntarily to give chance to other tasks.  
2) **Cooperative Scheduling** => when `configUSE_PREEMPTION = 0 `
    * Task cooperates with other tasks by explicitly giving up the processor (processor yielding).
    * There is no preemption of the tasks by the scheduler i.e. the running task will never be interrupted by the scheduler.
    * The RTOS tick interrupt doesn't cause any preemption, but the tick interrupt are still needed to keep track of the kernel's real-time tick value.
    * Tasks give up the CPU when they are done or periodically or blocked or suspended waiting for a resource.


## FreeRTOS API's
### Task Creation API
This API creates a new FreeRTOS task using dynamic memory allocation and adds the newly created task to ready queue of the kernel.  
```
BaseType_t xTaskCreate( TaskFunction_t pvTaskCode,          /* Address of the associated Task Handler : Function Pointer */
                        const char * const pcName,          /* Descriptive name to identify the task */
                        configSTACK_DEPTH_TYPE usStackDepth,/* Amount of Stack Memory allocated to this task */
                        void *pvParameters,                 /* Pointers of the data which needs to be passed to the task handler, once it gets scheduled*/
                        UBaseType_t uxPriority,             /* Task Priority Value */
                        TaskHandle_t *pxCreatedTask );      /* Used to save the Task Handle (an address of the task created) */
```


