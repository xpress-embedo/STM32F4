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

### FreeRTOS Scheduler Implementation
In FreeRTOS the scheduler code is actually a combination of `FreeRTOS Generic Code (tasks.c)` + `Architecture specific codes (ports.c)` .  
#### Architecture Specific Codes
Architecture Specific codes are responsible to achieve the scheduling of the tasks.  
  * All architecture specific codes and configurations are implemented in `port.c` & `portmacro.h`.  
  * If ARM Cortex Mx Processors are used, then we should be able to locate the below three interrupt handlers in `port.c` which are the part of the scheduler implementation of the FreeRTOS.  
    * `vPortSVCHandler()` : Used to launch the very first task, and is triggered by SVC instruction of ARM.  [Check `vTaskStartScheduler` API](#vtaskstartscheduler)  
    * `xPortPendSVHandler()` : Used to achieve the context switching between tasks, and is triggered by pending the PendSV System Exception of the ARM.  
      ![alt text](documentation/PendSV_Handler.png "PendSV Handler on Timeline of SystemView Application")  
    * `xPortSysTickHandler()` : This implements the RTOS Tick management, and is triggered periodically by SysTick Timer of ARM Cortex Mx Processor.  
      ![alt text](documentation/SysTick_Handler.png "SysTick Handler on Timeline of SystemView Application")  



## Idle Task
  * The Idle Task is created automatically when the RTOS scheduler is started to ensure there is always at least one task that is able to run.  
  * It is created at the lowest priority to ensure it does not use any CPU time if there are higher priority application tasks in the ready state.  
  * The Idle Task is responsible for freeing memory allocated by the RTOS to the tasks that have been deleted.  
  * When there are no tasks running, Idle task will always run on the CPU.  
  * We can give an application hook function in the Idle Task to send the CPU to low power mode when there are no useful tasks executing.  

## Timer Services Task (Timer_SVC)
  * This is also called as "Timer Daemon Task"
  * The timer daemon task deals with "Software Timers"
  * The task is created automatically when the scheduler is started and if `configUSER_TIMERS = 1` in the `FreeRTOSConfig.h` file.  
  * The RTOS uses this daemon to manage the FreeRTOS software timers and nothing else.  
  * If we don't use the software timer in our FreeRTOS application then we have disable this using `configUSER_TIMERS = 0` in the `FreeRTOSConfig.h` file.  
  * All Software timer callback functions execute in the context of the timer daemon task.  


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

### vTaskStartScheduler()
* This is implemented in `tasks.c` of the FreeRTOS kernel and used to start the RTOS scheduler.  
* After calling this function only the scheduler code is initialized and all the architecture specific interrupts will be activated.  
* This function also creates the `Idle Task` and the `Timer Daemon Task`.  
* This function calls `xPortStartScheduler()` to do the architecture specific initialization.  
