[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.me/embeddedlab)

# STM32F4 
The following are the projects which doesn't use the STM32CubeMX tools for device configuration
### 01-HelloWorld
**Project Path** - `Without_Cube/01-HelloWorld`  
**Development Environment** - STM32CubeIDE  
This is a simple `Hello World` project, where the main idea is to build the first project and use ITM Unit of the STM32F429I micro-controller to use use `printf` style debugging.  
This is only applicable for Cortex M3/M4/M7 or higher micro-controllers `printf` works over SWO (Serial Wire Output) pin of the SWD interface.  
If we have a board based on Cortex-M0/M0+ then we have to use OpenOCD based semi-hosting technique to use `printf`.  
In ARM Cortex-M4 processor there is a Unit called as ITM. ITM means Instrumentation Trace Macrocell Unit. The ITM is an optional application driven trace source that supports `printf` style debugging to trace operating system and application events, and generates diagnostic system information. And this unit is not available in Cortex-M0/M0+ processor.  

Note: The project is build for STM32F429I-DISCO board, and by default SWO port is not connected, we need to solder the SB9 bridge on the board, please check user manual of the board.  

### 02-HelloWorld-SemiHosting
**Project Path** - `Without_Cube/02-HelloWorld-Semihosting`  
**Development Environment** - STM32CubeIDE  
Implementation of `printf` feature using ARM Cortex M3/M4 ITM functionality is only limited to high-end processors such as Cortex M3/M4/M7.  
So in case we wanted to use the `printf` functionality for debugging on Cortex M0/M0+ we can use OpenOCD based Semi-Hosting. Here we have to update the linker arguments with the following flags.  
```-specs=rdimon.specs -lc -lrdimon```  
Instead of using ST-Link Debugger, we have to use ST-Link (OpenOCD) and then we need to call the function "initialise_monitor_handles()" at start-up and simply use `printf` function with \n character at last.  
Apart from this while debugging the project we have to use ST-Link Open OCD as debugger, and in the startup section of the debugger configuration we have to mention the following command.  
```monitor arm semihosting enable```  
And apart from all this we have to exclude the `syscalls.c` file from the build. If we don't follow all these steps the semihosting will not work properly.  

### 03-HSI_Measurement
**Project Path** - `Without_Cube/03-HSI_Measurement`  
**Development Environment** - STM32CubeIDE  
In this example we will do the HSI (High Speed Internal Oscillator) Clock Measurement, these clocks whether HSI or HSE can be connected to MCO1, which is micro-controller output-1,and then we can use GPIO's alternate function to connect this MCO1 output to PA8 pin of GPIOA.  
It should also output 16MHz (16MHz is internal RC on this discovery board) so it should output the same frequency, but since the Logic Analyzer which I  am using is pretty slow, I will pre-scale the output by 4, and will only get 4MHz on the PA8 pin of GPIOA.  
![alt text](Without_Cube/03-HSI_Measurement/HSI_Measurement.png "HSI Measurement")  

### 04-HSE_Measurement
**Project Path** - `Without_Cube/04-HSE_Measurement`  
**Development Environment** - STM32CubeIDE  
In this example we will do the HSE (High Speed External Clock) Clock Measurement, these clocks whether HSI or HSE can be connected to MCO1, which is micro-controller output-1,and then we can use GPIO's alternate function to connect this MCO1 output to PA8 pin of GPIOA.  
It should also output 8MHz (8MHz is external crystal oscillator connected on the discovery board) so it should output the same frequency, but since the Logic Analyzer which I  am using is pretty slow, I will pre-scale the output by 4, and will only get 2MHz on the PA8 pin of GPIOA.  
![alt text](Without_Cube/04-HSE_Measurement/HSE_Measurement.png "HSE Measurement")  

## FreeRTOS
All the FreeRTOS based projects are started with the name FreeRTOS appened at the starting of the folder.  
I have prepared some notes on FreeRTOS which can be accessed by [clicking here](FreeRTOS_ReadMe).  