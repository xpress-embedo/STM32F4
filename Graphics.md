
## Graphics Application
A microcontroller based interactive application which may involve any of the below components to display on the display device.
* Colours
  * Color of the texts
  * Background Color/Layer Color
* Shapes
  * Box
  * Circle
  * Arrows
  * Lines
* Images
* Texts
* Graphical Effects (Scrolling, Sliding, Swipe, Press, Release)
* Widgets (Simple Button, Radio Button, Check Box, Videos )
* 3D Rendering
* Gaming

## Embedded Graphics System

![Embedded Graphics System](Documentation/EmbeddedGraphicsSystem.PNG)

### Important Part of the Embedded Graphics System
* A Microcontroller (Host)
  * Processor (Executes our code, updates the buffer)
  * RAM (FRAME Buffer)
  * Flash (Static Images, Fonts, Texts etc )
* Display Module
  * Glass (Where we see the graphics)
  * Driver Chip (Interpret the signals sent by the host, generates required electric signals and voltages to lit the pixel of the display panel)

**Frame Buffer: A memory area which holds the pixel values of the frame to be updated on the display**

### Some Other Important Part of the Embedded Graphics System
* Display Controller
  * Present on the Host Side
  * Generates display timing related signals
  * Transmit Pixel Data  
  In STM32 there is a peripheral called LTDC which stands for LCD TFT Display Controller, this is a bus master, this takes directly the pixel data from the frame buffer and it can send it to the display module over LTDC module.
* External Memories
  * External Flash (our code plus graphic components such as images text, this may not fit inside the internal flash)
  * External RAM
* Graphics Library (LVGL, TouchGFX, emWin, Embedded Wizard etc. )
* UI designer tool
  * To create an interactive UI application
* Touch Sensing
  * Touch Panel
  * Touch Screen Controller (Which senses the touch panel and inform the host)
* DMA
  * Helps to transfer frame buffer to display without the intervention of the CPU
  * Helps to transfer the graphics details from the flash to frame buffer without the intervention of the CPU.



